#!/usr/bin/env python3
"""Phase 3 nominal grasp-and-lift trial + 5 deterministic position variants.

Sequence: pregrasp -> approach -> close -> lift -> hold -> lower -> open.
The cube is never welded, attached, teleported, mocap-driven, or given a
direct qpos write once a trial's simulation has started; the only place a
cube qpos is set directly is the trial's own initialization (its spawn
position), before any control step runs, which is the same as choosing an
initial condition, not manipulating the physics during the trial.
"""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place.controller import G1GraspController
from tasks.g1_pick_place.gripper_scene import (
    CUBE_HALF,
    CUBE_POS,
    TABLE_TOP_Z,
    write_grasp_scene,
)

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "logs"

TIMESTEP = 0.002
PREGRASP_DZ = 0.10
LIFT_DZ = 0.12
LIFT_HOLD_S = 2.5
CONTACT_MARGIN = 0.03  # cube counted "lifted off table" once z exceeds rest height by this much

PHASE_DURATIONS_S = {
    "to_pregrasp": 1.0,
    "approach": 1.0,
    "close": 0.6,
    "lift": 1.0,
    "hold": LIFT_HOLD_S,
    "lower": 1.0,
    "open": 0.6,
    "release_settle": 0.8,
}

NOMINAL_REST_Z = CUBE_POS[2]

VARIANT_OFFSETS = [
    (0.03, 0.0),
    (-0.03, 0.0),
    (0.0, 0.03),
    (0.0, -0.03),
    (0.02, -0.02),
]


def _finger_targets(gripper_map, open_: bool) -> np.ndarray:
    if open_:
        return np.array([0.0, 0.0])
    return np.array([gripper_map.jnt_range[0, 0], gripper_map.jnt_range[1, 1]])


def _contacts_between(data: mujoco.MjData, model: mujoco.MjModel, geom_a: int, geom_b: int) -> bool:
    for i in range(data.ncon):
        c = data.contact[i]
        pair = (int(c.geom1), int(c.geom2))
        if geom_a in pair and geom_b in pair:
            return True
    return False


def run_trial(model_path: Path, cube_xy_offset: tuple[float, float] = (0.0, 0.0)) -> dict:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    ctrl = G1GraspController(model=model)

    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")
    cube_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
    cube_qpos_adr = int(model.jnt_qposadr[cube_joint_id])

    mujoco.mj_resetData(model, data)
    cube_x = CUBE_POS[0] + cube_xy_offset[0]
    cube_y = CUBE_POS[1] + cube_xy_offset[1]
    cube_z = CUBE_POS[2]
    data.qpos[cube_qpos_adr : cube_qpos_adr + 3] = [cube_x, cube_y, cube_z]
    data.qpos[cube_qpos_adr + 3 : cube_qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)

    rest_z = float(data.xpos[cube_body_id][2])
    grasp_target = np.array([cube_x, cube_y, cube_z])
    pregrasp_target = grasp_target + np.array([0.0, 0.0, PREGRASP_DZ])
    lift_target = grasp_target + np.array([0.0, 0.0, LIFT_DZ])

    telemetry = {
        "both_pads_contact_cube": False,
        "max_cube_z": rest_z,
        "max_continuous_lifted_s": 0.0,
        "finite_and_bounded": True,
        "released_after_open": None,
        "cube_rest_z": rest_z,
        "time_series": [],
    }

    lifted_since = None
    steps_run = [0]

    def step_toward(target_pos, finger_open: bool, duration_s: float, start_pos=None):
        nonlocal lifted_since
        n = max(1, int(round(duration_s / TIMESTEP)))
        finger_target = _finger_targets(ctrl.gripper_map, finger_open)
        start = start_pos if start_pos is not None else ctrl.tcp_pos(data)
        for i in range(n):
            alpha = (i + 1) / n
            waypoint = start + alpha * (target_pos - start)
            joint_target = ctrl.ik_target_for(data, waypoint)
            arm_torque = ctrl.track_arm(data, joint_target)
            grip_torque = ctrl.track_gripper(data, finger_target)

            if not (np.all(np.isfinite(arm_torque)) and np.all(np.isfinite(grip_torque))):
                telemetry["finite_and_bounded"] = False
            if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
                telemetry["finite_and_bounded"] = False

            mujoco.mj_step(model, data)
            steps_run[0] += 1

            cube_z = float(data.xpos[cube_body_id][2])
            telemetry["max_cube_z"] = max(telemetry["max_cube_z"], cube_z)

            both_contact = _contacts_between(data, model, cube_geom_id, left_pad_id) and _contacts_between(
                data, model, cube_geom_id, right_pad_id
            )
            if both_contact:
                telemetry["both_pads_contact_cube"] = True

            is_lifted = cube_z > (rest_z + CONTACT_MARGIN)
            if is_lifted:
                if lifted_since is None:
                    lifted_since = data.time
                telemetry["max_continuous_lifted_s"] = max(
                    telemetry["max_continuous_lifted_s"], data.time - lifted_since
                )
            else:
                lifted_since = None

            if steps_run[0] % 50 == 0:
                telemetry["time_series"].append(
                    {"t": float(data.time), "cube_z": cube_z, "both_pads_contact": bool(both_contact)}
                )

    step_toward(pregrasp_target, finger_open=True, duration_s=PHASE_DURATIONS_S["to_pregrasp"])
    step_toward(grasp_target, finger_open=True, duration_s=PHASE_DURATIONS_S["approach"])
    step_toward(grasp_target, finger_open=False, duration_s=PHASE_DURATIONS_S["close"])
    step_toward(lift_target, finger_open=False, duration_s=PHASE_DURATIONS_S["lift"])
    step_toward(lift_target, finger_open=False, duration_s=PHASE_DURATIONS_S["hold"])
    step_toward(grasp_target, finger_open=False, duration_s=PHASE_DURATIONS_S["lower"])
    step_toward(grasp_target, finger_open=True, duration_s=PHASE_DURATIONS_S["open"])

    settle_n = int(round(PHASE_DURATIONS_S["release_settle"] / TIMESTEP))
    open_finger_target = _finger_targets(ctrl.gripper_map, True)
    for _ in range(settle_n):
        ctrl.track_gripper(data, open_finger_target)
        arm_hold_target = ctrl.ik_target_for(data, grasp_target)
        ctrl.track_arm(data, arm_hold_target)
        mujoco.mj_step(model, data)
    telemetry["released_after_open"] = not (
        _contacts_between(data, model, cube_geom_id, left_pad_id)
        or _contacts_between(data, model, cube_geom_id, right_pad_id)
    )

    height_gain = telemetry["max_cube_z"] - rest_z
    criteria = {
        "both_pads_contact_cube": telemetry["both_pads_contact_cube"],
        "height_gain_ge_0_08m": height_gain >= 0.08,
        "lifted_ge_2s_continuous": telemetry["max_continuous_lifted_s"] >= 2.0,
        "finite_and_bounded": telemetry["finite_and_bounded"],
        "released_after_open": bool(telemetry["released_after_open"]),
    }

    return {
        "cube_xy_offset": list(cube_xy_offset),
        "cube_spawn_pos": [cube_x, cube_y, cube_z],
        "cube_rest_z": rest_z,
        "height_gain_m": height_gain,
        "max_continuous_lifted_s": telemetry["max_continuous_lifted_s"],
        "criteria": criteria,
        "pass": all(criteria.values()),
        "time_series_sample": telemetry["time_series"],
    }


def main() -> int:
    scene_path = write_grasp_scene()

    nominal = run_trial(scene_path, cube_xy_offset=(0.0, 0.0))
    result = {"scene": str(scene_path.relative_to(ROOT)), "nominal": nominal}

    if not nominal["pass"]:
        result["variants"] = []
        result["variant_success_rate"] = None
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        (LOG_DIR / "phase3_grasp_trials.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps({"nominal_pass": False, "criteria": nominal["criteria"]}, indent=2))
        return 1

    variants = []
    for offset in VARIANT_OFFSETS:
        variants.append(run_trial(scene_path, cube_xy_offset=offset))
    result["variants"] = variants
    result["variant_success_rate"] = sum(1 for v in variants if v["pass"]) / len(variants)

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    (LOG_DIR / "phase3_grasp_trials.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    print(
        json.dumps(
            {
                "nominal_pass": nominal["pass"],
                "variant_success_rate": result["variant_success_rate"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
