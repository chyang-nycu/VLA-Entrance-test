#!/usr/bin/env python3
"""Phase 3 controller: damped least-squares Cartesian IK + bounded joint-space PD.

Explicit index maps (qpos address, qvel/dof address, actuator id) are built
once per joint name and reused; nothing relies on implicit positional
ordering of MuJoCo's internal arrays.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]
RIGHT_ARM_ACTUATORS = [
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
]
GRIPPER_JOINTS = ["left_finger_joint", "right_finger_joint"]
GRIPPER_ACTUATORS = ["left_finger", "right_finger"]

TCP_SITE = "grasp_tcp"

# Bounded joint-space PD gains (arm joints are torque-actuated per the vendor
# model; gripper joints are lighter and use a softer pair).
#
# Tuning iteration 2: iteration 1 used kp=60/kd=6 with no gravity/Coriolis
# feedforward. That combination tracked a fixed single-joint step target
# stably but far too slowly (~2 s to settle 0.3 rad), so a full 7-joint
# multi-second reach never caught up to its moving IK waypoint (final TCP
# error ~0.13 m, no finger-cube contact at all). Fix: add qfrc_bias
# (gravity + Coriolis) feedforward in bounded_pd_step, and raise gains,
# since steady-state error no longer has to be carried by kp alone.
ARM_KP = 180.0
ARM_KD = 18.0
GRIPPER_KP = 40.0
GRIPPER_KD = 2.0

# Explicit step/velocity bounds applied to the IK target before PD tracks it,
# so no single control step commands an unreachable jump.
ARM_MAX_STEP = 0.03  # rad per control call
GRIPPER_MAX_STEP = 0.01  # m per control call
ARM_MAX_QVEL = 2.5  # rad/s safety clamp before torque is allowed to add energy
GRIPPER_MAX_QVEL = 0.4  # m/s

DLS_DAMPING = 0.05
DLS_MAX_ITERS = 80
DLS_TOL_POS = 1e-4
DLS_MAX_DQ_STEP = 0.05  # rad per IK internal iteration


@dataclass
class JointMap:
    names: list
    qpos_adr: np.ndarray
    dof_adr: np.ndarray
    actuator_id: np.ndarray
    jnt_range: np.ndarray
    ctrl_range: np.ndarray

    @classmethod
    def build(cls, model: mujoco.MjModel, joint_names: list, actuator_names: list) -> "JointMap":
        qpos_adr = []
        dof_adr = []
        actuator_id = []
        jnt_range = []
        ctrl_range = []
        for jname, aname in zip(joint_names, actuator_names):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                raise ValueError(f"joint not found: {jname}")
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
            if aid < 0:
                raise ValueError(f"actuator not found: {aname}")
            qpos_adr.append(int(model.jnt_qposadr[jid]))
            dof_adr.append(int(model.jnt_dofadr[jid]))
            actuator_id.append(aid)
            jnt_range.append(
                tuple(model.jnt_range[jid]) if bool(model.jnt_limited[jid]) else (-np.inf, np.inf)
            )
            ctrl_range.append(
                tuple(model.actuator_ctrlrange[aid])
                if bool(model.actuator_ctrllimited[aid])
                else (-np.inf, np.inf)
            )
        return cls(
            names=list(joint_names),
            qpos_adr=np.array(qpos_adr, dtype=int),
            dof_adr=np.array(dof_adr, dtype=int),
            actuator_id=np.array(actuator_id, dtype=int),
            jnt_range=np.array(jnt_range, dtype=float),
            ctrl_range=np.array(ctrl_range, dtype=float),
        )

    def get_qpos(self, data: mujoco.MjData) -> np.ndarray:
        return data.qpos[self.qpos_adr].copy()

    def get_qvel(self, data: mujoco.MjData) -> np.ndarray:
        return data.qvel[self.dof_adr].copy()

    def set_qpos(self, data: mujoco.MjData, values: np.ndarray) -> None:
        data.qpos[self.qpos_adr] = values

    def set_ctrl(self, data: mujoco.MjData, values: np.ndarray) -> None:
        data.ctrl[self.actuator_id] = values


def quat_to_axis_angle_error(q_current: np.ndarray, q_target: np.ndarray) -> np.ndarray:
    """3-vector orientation error (world frame) driving q_current toward q_target."""
    q_conj = np.array([q_current[0], -q_current[1], -q_current[2], -q_current[3]])
    q_err = np.zeros(4)
    mujoco.mju_mulQuat(q_err, q_target, q_conj)
    if q_err[0] < 0:
        q_err = -q_err
    axis = q_err[1:4]
    sin_half = np.linalg.norm(axis)
    if sin_half < 1e-9:
        return np.zeros(3)
    angle = 2.0 * np.arctan2(sin_half, q_err[0])
    return axis / sin_half * angle


def solve_dls_ik(
    model: mujoco.MjModel,
    scratch_data: mujoco.MjData,
    base_qpos: np.ndarray,
    joint_map: JointMap,
    site_id: int,
    target_pos: np.ndarray,
    target_quat: np.ndarray | None = None,
    damping: float = DLS_DAMPING,
    max_iters: int = DLS_MAX_ITERS,
) -> np.ndarray:
    """Damped least-squares Cartesian IK for the controlled joint subset.

    Uses a caller-owned scratch MjData so the live simulation state is never
    disturbed. Returns a joint target vector (same order as joint_map.names),
    clipped to each joint's physical range.
    """
    scratch_data.qpos[:] = base_qpos
    q = joint_map.get_qpos(scratch_data).copy()
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    ndof = len(joint_map.dof_adr)

    for _ in range(max_iters):
        joint_map.set_qpos(scratch_data, q)
        mujoco.mj_kinematics(model, scratch_data)
        mujoco.mj_comPos(model, scratch_data)
        site_pos = scratch_data.site_xpos[site_id].copy()
        pos_err = target_pos - site_pos

        if target_quat is not None:
            site_mat = scratch_data.site_xmat[site_id].reshape(3, 3)
            site_quat = np.zeros(4)
            mujoco.mju_mat2Quat(site_quat, site_mat.flatten())
            ori_err = quat_to_axis_angle_error(site_quat, target_quat)
            err = np.concatenate([pos_err, ori_err])
        else:
            err = pos_err

        if np.linalg.norm(pos_err) < DLS_TOL_POS and (target_quat is None or np.linalg.norm(ori_err) < 1e-3):
            break

        mujoco.mj_jacSite(model, scratch_data, jacp, jacr, site_id)
        if target_quat is not None:
            J = np.vstack([jacp[:, joint_map.dof_adr], jacr[:, joint_map.dof_adr]])
        else:
            J = jacp[:, joint_map.dof_adr]

        JJt = J @ J.T + (damping ** 2) * np.eye(J.shape[0])
        dq = J.T @ np.linalg.solve(JJt, err)

        step_norm = np.linalg.norm(dq)
        if step_norm > DLS_MAX_DQ_STEP:
            dq = dq * (DLS_MAX_DQ_STEP / step_norm)

        q = q + dq
        q = np.clip(q, joint_map.jnt_range[:, 0], joint_map.jnt_range[:, 1])

    return q


def bounded_pd_step(
    joint_map: JointMap,
    data: mujoco.MjData,
    target: np.ndarray,
    kp: float,
    kd: float,
    max_step: float,
    max_qvel: float,
) -> np.ndarray:
    """One bounded joint-space PD control step. Returns the applied ctrl vector."""
    qpos = joint_map.get_qpos(data)
    qvel = joint_map.get_qvel(data)

    bounded_target = np.clip(target, joint_map.jnt_range[:, 0], joint_map.jnt_range[:, 1])
    delta = bounded_target - qpos
    delta = np.clip(delta, -max_step, max_step)
    step_target = qpos + delta

    gravity_coriolis_ff = data.qfrc_bias[joint_map.dof_adr]
    torque = kp * (step_target - qpos) - kd * qvel + gravity_coriolis_ff

    overspeed = np.abs(qvel) > max_qvel
    same_direction = np.sign(qvel) == np.sign(torque - gravity_coriolis_ff)
    torque = np.where(overspeed & same_direction, gravity_coriolis_ff, torque)

    torque = np.clip(torque, joint_map.ctrl_range[:, 0], joint_map.ctrl_range[:, 1])

    if not np.all(np.isfinite(torque)):
        raise FloatingPointError(f"non-finite control output for joints {joint_map.names}: {torque}")

    joint_map.set_ctrl(data, torque)
    return torque


@dataclass
class G1GraspController:
    model: mujoco.MjModel
    arm_map: JointMap = field(init=False)
    gripper_map: JointMap = field(init=False)
    tcp_site_id: int = field(init=False)
    ik_scratch: mujoco.MjData = field(init=False)

    def __post_init__(self) -> None:
        self.arm_map = JointMap.build(self.model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
        self.gripper_map = JointMap.build(self.model, GRIPPER_JOINTS, GRIPPER_ACTUATORS)
        self.tcp_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
        if self.tcp_site_id < 0:
            raise ValueError(f"site not found: {TCP_SITE}")
        self.ik_scratch = mujoco.MjData(self.model)

    def ik_target_for(self, data: mujoco.MjData, target_pos: np.ndarray, target_quat: np.ndarray | None = None) -> np.ndarray:
        return solve_dls_ik(
            self.model, self.ik_scratch, data.qpos.copy(), self.arm_map, self.tcp_site_id,
            target_pos, target_quat,
        )

    def track_arm(self, data: mujoco.MjData, joint_target: np.ndarray) -> np.ndarray:
        return bounded_pd_step(
            self.arm_map, data, joint_target, ARM_KP, ARM_KD, ARM_MAX_STEP, ARM_MAX_QVEL,
        )

    def track_gripper(self, data: mujoco.MjData, joint_target: np.ndarray) -> np.ndarray:
        return bounded_pd_step(
            self.gripper_map, data, joint_target, GRIPPER_KP, GRIPPER_KD, GRIPPER_MAX_STEP, GRIPPER_MAX_QVEL,
        )

    def tcp_pos(self, data: mujoco.MjData) -> np.ndarray:
        return data.site_xpos[self.tcp_site_id].copy()
