#!/usr/bin/env python3
"""Task 3 (Phase 7, articulated manipulation): door-opening.

Scene: write_grasp_scene_5a()'s own output (Task 1's scene, unmodified,
re-used by import), plus one passive hinged door -- a jointless frame body
and a panel body carrying exactly one `hinge` joint, built by re-parsing
that scene's own output file, never by editing gripper_scene.py or
camera_observation.py. This mirrors write_task2_scene()'s exact convention.

Geometry (pivot, radius, swing) is NOT chosen here. It is read from
`logs/phase7a_derived_door_geometry.json`, produced by
`workspace_map.search_largest_arc()` -- the Phase 7A workspace/conditioning
map that measures where this fixed-base arm's reach is well conditioned and
orientation-achievable, and derives the door's shape from that measurement.
See reports/phase7a-workspace-map.md for the full account, including why
the *maximal* admissible arc is deliberately NOT what gets built: the
largest arc hugs the 7deg orientation boundary by construction (it is the
biggest arc that still barely clears the gate), so this module locks a
margin-selected candidate instead -- smaller, with comfortable headroom on
every threshold. The selection function and its criterion are in this file,
not hand-picked, so the choice is reproducible.

Handle: a vertical cylinder of radius CUBE_HALF (0.035m), imported unchanged
from gripper_scene.py. This is deliberate, not a stylistic choice: the
gripper's closed-jaw squeeze overtravel (FINGER_SQUEEZE_MARGIN) was
calibrated in Phase 3C/4E against exactly a 2*CUBE_HALF=0.070m object. A
handle of any other cross-section would be an untested grip geometry: this
one reuses Task 1's already-verified contact mechanics exactly. A vertical
cylinder is also yaw-invariant, so no wrist-yaw tracking is needed as the
handle orbits the pivot.

The door is entirely PASSIVE: no actuator, no equality, no tendon on any
door body. It can only be moved by real contact through the gripper. This
is the structural (not merely measured) proof that opening it requires no
scripted assistance, mirroring the project's standing "never weld/teleport"
rule.
"""

from __future__ import annotations

import inspect
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place.camera_observation import write_grasp_scene_5a
from tasks.g1_pick_place.controller import (
    GRIPPER_ACTUATORS,
    GRIPPER_JOINTS,
    GRIPPER_MAX_QVEL,
    GRIPPER_MAX_STEP,
    RIGHT_ARM_ACTUATORS,
    RIGHT_ARM_JOINTS,
    TCP_SITE,
    JointMap,
    bounded_pd_step,
)
from tasks.g1_pick_place.controller_3c import (
    IK_POS_TOL,
    ORIENT_TOL_RAD,
    solve_ik_waypoint,
    solve_ik_waypoint_oriented,
)
from tasks.g1_pick_place.gripper_scene import (
    CUBE_FRICTION,
    CUBE_HALF,
    CUBE_POS,
    FINGER_CLOSED_Y,
    FINGER_OPEN_Y,
    TABLE_TOP_Z,
    TARGET_POS,
    TARGET_HALF_XY,
    TASK_DIR,
)
from tasks.g1_pick_place.run_grasp_test import _contacts_between
from tasks.g1_pick_place.run_grasp_test_3c import (
    SETTLE_ARM_QVEL_TOL,
    SETTLE_MAX_EXTRA_S,
    SETTLE_TCP_POS_TOL,
    _finger_targets,
)
from tasks.g1_pick_place.run_pick_place import (
    ARM_KP_4B,
    ARM_KV_4B,
    GRIPPER_KD_4E,
    _solve_waypoint,
    relative_slip_m,
    tcp_local_cube_offset,
)
from tasks.g1_pick_place.workspace_map import (
    ARC_HANDLE_Z,
    ARC_SIGMA_MIN_FLOOR,
    GEOMETRY_PATH,
    arc_is_admissible,
    handle_pose,
)

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "logs"
CANONICAL_CONFIG_PATH = ROOT / "data" / "task3_canonical_config.json"

TIMESTEP = 0.002
# Gains as of Phase 8 (reports/phase8-slip-diagnosis.md). Phase 7C's
# original ARM_KP_DOOR=600 / GRIPPER_KP_DOOR=GRIPPER_KP_4E(320) fixed the
# PREGRASP settle droop but left door_pass=False: max slip 22.3mm against
# a 10mm target. Phase 8 traced this to TWO INDEPENDENT, ADDITIVE
# mechanisms -- (1) arm tracking error, present from the first PULL_ARC
# waypoint, demonstrated causal by a monotonic dose-response (arm_kp
# 600->900->1200 cut slip 22.3->20.8->20.7mm and raised contact retention
# 82%->90%); (2) bilateral grip-force decline partway through the pull
# (Phase 7E's finding, gripper_kp 320->480->640 cut slip 22.3->18.7->16.3mm
# and raised retention to 100%) -- and found that COMBINING both closes
# the gap: at arm_kp=2200/gripper_kp=1200, max slip is 8.22mm,
# door_pass=True (all 11 criteria), bit-identical across reruns, and
# actuator force peaks at 63% of the physical torque limit (real margin,
# not pushing past what the arm can actually deliver). Neither lever
# alone reaches this; five weaker hypotheses (wrist-orientation
# constraint, finger-pad contact height, and a conditioning-only
# manipulation via null-space posture gain) were tested and found NOT to
# explain the effect -- tightening orientation control made slip WORSE.
ARM_KP_DOOR = 2200.0
ARM_KV_DOOR = ARM_KV_4B
GRIPPER_KP_DOOR = 1200.0
GRIPPER_KD_DOOR = GRIPPER_KD_4E
PREGRASP_STANDOFF_DEG = 8.0  # in-plane tangential standoff, see diagnose_door_reachability
DRIVE_S_DOOR = {
    # CLOSE is 1.5s, not Task 1's 0.8s: measured directly (bilateral
    # contact detection over a held CLOSE command) that both finger pads
    # only settle into simultaneous contact with the cylindrical handle by
    # ~1.2s -- a slower two-stage closing dynamic than the cube's flat
    # faces, plausibly because the two pads approach a curved surface from
    # slightly different effective gaps. 1.5s adds real margin above the
    # measured 1.2s settle point.
    "PREGRASP_HANDLE": 0.8, "APPROACH_HANDLE": 0.8, "CLOSE": 1.5,
    "OPEN": 0.6, "RELEASE_SETTLE": 0.8,
}
# Swept {(2.0,30), (3.0,45), (4.0,60), (5.0,75), (6.0,90), (8.0,120)}
# (duration_s, n_waypoints), each scaled together to keep the per-waypoint
# sub-step count constant. Slower, finer-grained pulls monotonically
# reached a larger hinge angle with less slip (33 deg/30.6mm slip at the
# fastest, up to the full 45 deg target at 5.0s/75wp with 22.3mm slip) --
# consistent with Phase 4E/4B's own finding that a smoother, more
# finely-chunked drive reduces slip on long moves. 5.0s/75 reaches the
# target angle; slower settings beyond it did not meaningfully improve
# slip further (6.0s: 22.6mm, 8.0s: 23.0mm), so this is the shipped value,
# not a search still in progress. See reports/phase7c-door-motion.md.
PULL_ARC_DRIVE_S = 5.0
PULL_ARC_N_WAYPOINTS = 75
RETREAT_DRIVE_S_DOOR = 0.8
RETREAT_STANDOFF_M = 0.10
HANDLE_GRASP_CORRIDOR_RAD = 0.02  # hinge must not have moved more than this before lift-off is confirmed
DOOR_OPEN_FRACTION = 0.75  # LIBERO-style: "open" means >= 75% of the authorized swing, not the full swing
DOOR_STATIC_QVEL_TOL = 0.10  # rad/s
DOOR_DWELL_S = 2.0
DOOR_DWELL_MAX_WAIT_S = 5.0
HANDLE_SLIP_TOL_M = 0.010  # same bar Task 1 uses for max_slip_while_grasped

# --- Door body dimensions (not the pivot/radius/swing, which are derived) ---
HANDLE_RADIUS_M = CUBE_HALF  # 0.035m -- Task 1's verified squeeze-grip geometry, reused exactly
HANDLE_HALF_LENGTH_M = 0.03  # short vertical cylinder, well inside the finger pads' Z half-extent
PANEL_THICKNESS_M = 0.008
PANEL_HALF_HEIGHT_M = 0.10
PANEL_HANDLE_CLEARANCE_M = 0.06  # real gap between the panel's outer edge and the handle
FRAME_THICKNESS_M = 0.010

# Frictionloss/damping on the hinge -- Phase 1's only tunable, within a
# 3-attempt budget (see select_and_lock_geometry's caller / the report).
# Attempt 1 values; see reports/phase7-door-scene.md for the attempt record.
HINGE_DAMPING = 0.05
HINGE_FRICTIONLOSS = 0.08


RESTING_ARM_LINKS = (
    "right_shoulder_pitch_link", "right_shoulder_roll_link", "right_shoulder_yaw_link",
    "right_elbow_link", "right_wrist_roll_link", "right_wrist_pitch_link", "right_wrist_yaw_link",
    # The gripper's own bodies extend further from the wrist chain than the
    # 7 link bodies above (~0.10m via FINGER_REACH_X and the finger/palm
    # mount offsets). A candidate can clear all 7 wrist-chain links by the
    # required margin and still have its panel/handle overlap the idle
    # FINGERS specifically -- found by direct measurement (tests/
    # test_door_open.py's TestScene.test_no_unwanted_collisions_at_reset
    # caught a real -4.66mm door_panel/left_finger penetration at reset on
    # a candidate that passed the link-only check). Included here so the
    # search rejects such candidates instead of a test catching it after
    # the fact.
    "left_finger", "right_finger", "palm",
)
# Minimum distance from the CLOSED handle position to every resting-arm link
# body's world position at the plain mj_resetData pose. This is checked
# because the workspace region that is well *oriented and conditioned when
# actively driven* turns out to sit close to where the arm rests when it is
# NOT being driven -- confirmed by direct measurement: at reset, an early
# candidate's door_panel/door_handle geoms penetrated right_elbow_link by
# 55mm and right_shoulder_yaw_link by 2mm, while Task 1's own scene has zero
# such contacts at reset (only the pre-existing, unrelated ankle/table
# artifact). A door must not occupy the space the idle arm already fills.
# 0.15m was calibrated against the 7 large wrist-chain link bodies only.
# Adding the finger/palm bodies (below) made 0.15m geometrically
# infeasible everywhere in the CLASS-A region -- direct measurement found
# the best achievable single-point clearance is 0.106m, and once combined
# with select_door_geometry's OTHER margin requirements over a full arc
# (not just one point), the best surviving candidate has only 0.0838m.
# The finger/palm bodies are also physically much smaller (a few cm) than
# the link bodies they're compared against, so a smaller floor is
# appropriate for them, not a compromise. 0.06m sits below that measured
# ceiling (so real candidates exist) while still giving >10x margin over
# the -4.66mm penetration that motivated adding these bodies at all.
REST_CLEARANCE_MIN_M = 0.06


def _resting_arm_link_positions() -> np.ndarray:
    """World-frame positions of the right-arm link bodies at the plain
    mj_resetData pose (no control applied) -- the configuration the arm is
    actually in before any trial issues its first command."""
    scene_path = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return np.array([
        data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)]
        for name in RESTING_ARM_LINKS
    ])


def clears_resting_arm(closed_handle_pos: np.ndarray, resting_pts: np.ndarray,
                       min_clearance_m: float = REST_CLEARANCE_MIN_M) -> bool:
    return bool(np.min(np.linalg.norm(resting_pts - closed_handle_pos, axis=1)) >= min_clearance_m)


def select_door_geometry(
    *,
    min_orient_margin_deg: float = 3.0,
    min_sigma_margin: float = 0.03,
    min_resid_margin_m: float = 0.003,
    candidates_path: Path = GEOMETRY_PATH,
) -> dict:
    """Pick a door geometry with real safety margin, not the single largest
    arc Phase 7A's raw search returned.

    Phase 7A's `search_largest_arc` sorts by chord length, so its top
    candidates are, by construction, the biggest arc that still barely
    clears the admissibility gate -- they sit within a degree of the 7deg
    orientation wall. This function re-scores the SAME admissibility
    predicate (`arc_is_admissible`, imported unchanged) but selects for
    margin on every threshold simultaneously, at reduced but still
    substantial swing (45deg / 60deg instead of the maximal ~90deg).

    Deterministic: no RNG, small fixed grid, sorted output. Raises if
    Phase 7A's gate was NO_GO (this function must not be called in that
    case -- the caller falls back to a drawer instead).

    Known limitation: `clears_resting_arm` checks only the single closed
    HANDLE point, not the panel/frame's actual box/cylinder footprint.
    For a large enough radius that footprint can itself overlap a
    resting-arm body even when the handle point clears it by the required
    margin -- found by direct measurement (a re-run of this function
    after RESTING_ARM_LINKS was extended to include the finger/palm
    bodies selected a geometry with up to -25mm of panel/frame
    penetration against the idle wrist chain, worse than the point check
    alone predicted). `logs/phase7b_selected_door_geometry.json`'s
    currently locked geometry was therefore chosen by directly building
    the scene and sweeping the real hinge range for contacts (see
    reports/phase7d-door-tests.md), not by trusting this function's
    output as final -- treat this function's result as a strong candidate
    to verify, not a verified answer.
    """
    derived = json.loads(candidates_path.read_text())
    if derived["gate_outcome"] != "GO_HINGE":
        raise RuntimeError(
            f"Phase 7A gate is {derived['gate_outcome']!r}, not GO_HINGE -- "
            "a hinged door was not licensed by the workspace study; see "
            "reports/phase7a-workspace-map.md for the drawer fallback."
        )

    from tasks.g1_pick_place import workspace_map as wm

    model, data, arm_map, site_id = wm._build_env()
    scratch = mujoco.MjData(model)
    base_qpos = data.qpos.copy()
    resting_pts = _resting_arm_link_positions()

    best = None
    n_margin_ok_but_rest_blocked = 0
    for px in np.arange(0.16, 0.441, 0.02):
        for py in np.arange(-0.32, 0.021, 0.02):
            for radius in (0.08, 0.10, 0.12, 0.14):
                for phi0 in np.arange(0.0, 360.0, 15.0):
                    for theta in (45.0, 60.0):
                        ok, worst = arc_is_admissible(
                            model, scratch, base_qpos, arm_map, site_id,
                            (float(px), float(py)), radius, float(phi0), theta,
                            step_deg=7.5,
                        )
                        if not ok:
                            continue
                        orient_margin = np.degrees(ORIENT_TOL_RAD) - worst["orientation_residual_deg"]
                        sigma_margin = worst["min_singular_value"] - ARC_SIGMA_MIN_FLOOR
                        resid_margin = IK_POS_TOL - worst["ik_residual_m"]
                        if (orient_margin < min_orient_margin_deg
                                or sigma_margin < min_sigma_margin
                                or resid_margin < min_resid_margin_m):
                            continue
                        closed_handle = handle_pose((px, py), radius, phi0, ARC_HANDLE_Z)
                        if not clears_resting_arm(closed_handle, resting_pts):
                            n_margin_ok_but_rest_blocked += 1
                            continue
                        chord = 2 * radius * np.sin(np.radians(theta) / 2)
                        cand = {
                            "pivot_xy": [round(float(px), 4), round(float(py), 4)],
                            "radius_m": radius,
                            "phi0_deg": float(phi0),
                            "theta_deg": theta,
                            "chord_m": float(chord),
                            "handle_z": ARC_HANDLE_Z,
                            "orient_margin_deg": float(orient_margin),
                            "sigma_margin": float(sigma_margin),
                            "resid_margin_m": float(resid_margin),
                            "rest_clearance_m": float(np.min(np.linalg.norm(resting_pts - closed_handle, axis=1))),
                            "worst": worst,
                        }
                        if best is None or chord > best["chord_m"]:
                            best = cand
    if best is None:
        raise RuntimeError(
            "no arc meets the margin AND resting-arm-clearance criteria within "
            f"the search grid ({n_margin_ok_but_rest_blocked} candidates met the "
            "margin criteria alone but were rejected for overlapping the idle "
            "arm's resting position) -- widen the grid, relax margins, or "
            "increase REST_CLEARANCE_MIN_M with a disclosed rationale"
        )
    return best


def check_clearance_from_task1_objects(geometry: dict, min_clearance_m: float = 0.03) -> dict:
    """Confirm the door's swept footprint does not overlap Task 1's
    (inert, still-present) cube or target pad, in XY or in Z. The cube and
    pad stay in the scene -- they are not removed -- so this is a real
    geometric check, not an assumption.
    """
    pivot_xy = np.array(geometry["pivot_xy"])
    radius = geometry["radius_m"]
    phi0, theta = geometry["phi0_deg"], geometry["theta_deg"]
    z = geometry.get("handle_z", ARC_HANDLE_Z)
    angles = np.radians(phi0 + np.linspace(0.0, theta, 19))
    sweep_xy = pivot_xy + radius * np.stack([np.cos(angles), np.sin(angles)], axis=1)

    cube_xy = np.array(CUBE_POS[:2])
    cube_top_z = CUBE_POS[2] + CUBE_HALF
    pad_xy = np.array(TARGET_POS)
    pad_top_z = TABLE_TOP_Z + 0.005

    cube_xy_clear = float(np.min(np.linalg.norm(sweep_xy - cube_xy, axis=1))) >= (CUBE_HALF + min_clearance_m)
    pad_xy_clear = float(np.min(np.linalg.norm(sweep_xy - pad_xy, axis=1))) >= (TARGET_HALF_XY + min_clearance_m)
    cube_z_clear = (z - cube_top_z) >= min_clearance_m
    pad_z_clear = (z - pad_top_z) >= min_clearance_m

    return {
        "cube_clear": bool(cube_xy_clear or cube_z_clear),
        "target_pad_clear": bool(pad_xy_clear or pad_z_clear),
        "cube_xy_margin_m": float(np.min(np.linalg.norm(sweep_xy - cube_xy, axis=1)) - CUBE_HALF),
        "target_pad_xy_margin_m": float(np.min(np.linalg.norm(sweep_xy - pad_xy, axis=1)) - TARGET_HALF_XY),
        "vertical_clearance_from_cube_m": float(z - cube_top_z),
        "vertical_clearance_from_pad_m": float(z - pad_top_z),
    }


def _override_right_arm_gains(root: ET.Element, arm_kp: float, arm_kv: float) -> None:
    """Patch the `kp`/`kv` attributes of the 7 right-arm `<position>`
    actuators IN THIS TREE ONLY. Deliberately a separate post-processing
    step, applied strictly AFTER write_grasp_scene_5a's file has already
    been written and re-parsed -- write_grasp_scene_5a/_4b are always
    called with the unchanged ARM_KP_4B/ARM_KV_4B (see write_door_scene),
    so the shared g1_grasp_scene_4b.xml/_5a.xml files this task-local
    override must NOT touch are never regenerated with a different gain.
    """
    from tasks.g1_pick_place.gripper_scene import RIGHT_ARM_JOINT_ACTUATOR_PAIRS

    actuator = root.find("actuator")
    if actuator is None:
        raise RuntimeError("expected actuator section")
    names = {name for _, name, _ in RIGHT_ARM_JOINT_ACTUATOR_PAIRS}
    patched = 0
    for el in actuator.findall("position"):
        if el.get("name") in names:
            el.set("kp", f"{arm_kp:.6f}")
            el.set("kv", f"{arm_kv:.6f}")
            patched += 1
    if patched != len(names):
        raise RuntimeError(f"expected to patch {len(names)} right-arm position actuators, patched {patched}")


def _override_finger_closing_range(root: ET.Element, extra_closing_m: float) -> None:
    """Attempt 2 of the CLOSE-phase calibration budget (attempt 1 was the
    arm-gain increase; see ARM_KP_DOOR). Measured directly: approaching the
    handle through the real PREGRASP_HANDLE -> APPROACH_HANDLE chain (not
    a single-shot solve from RESET) lands at a slightly different
    IK-redundancy configuration than a direct solve, leaving the two
    finger pads asymmetric by roughly 0.6mm at first contact -- one pad
    reaches the handle, the other stops just short of it, and the
    resulting one-sided force alone drives the door instead of a bilateral
    grip. Giving the fingers a small extra amount of commanded closing
    travel (task-local to this scene only, gripper_scene.py's own
    FINGER_CLOSED_Y is never changed) reliably closes that sub-millimetre
    gap on both sides. Chosen conservatively generous (5mm) against a
    measured ~0.6mm shortfall.
    """
    left = None
    right = None
    for el in root.iter("joint"):
        if el.get("name") == "left_finger_joint":
            left = el
        elif el.get("name") == "right_finger_joint":
            right = el
    if left is None or right is None:
        raise RuntimeError("expected left_finger_joint/right_finger_joint not found")
    span = FINGER_OPEN_Y - FINGER_CLOSED_Y + extra_closing_m
    left.set("range", f"{-span:.4f} 0")
    right.set("range", f"0 {span:.4f}")


FINGER_EXTRA_CLOSING_M = 0.005


def _override_finger_pad_height(root: ET.Element, pad_half_z: float) -> None:
    """H2 ablation knob: finger pad Z half-extent, task-local to this
    scene copy only (gripper_scene.py's FINGER_PAD_HALF/LEGACY_FINGER_PAD_HALF
    are never redefined). Mirrors Task 1's own Phase 4E attempt 1 (taller
    pads to keep the grasped object's center inside the pads' vertical
    span despite wrist-roll drift) -- reused here as a direct, motivated
    test of the same "grasp contact geometry" hypothesis for the door
    handle, one factor at a time.
    """
    patched = 0
    for el in root.iter("geom"):
        name = el.get("name")
        if name in ("left_finger_pad", "right_finger_pad"):
            size = el.get("size", "").split()
            if len(size) != 3:
                raise RuntimeError(f"unexpected {name} size attribute: {el.get('size')!r}")
            size[2] = f"{pad_half_z:.6f}"
            el.set("size", " ".join(size))
            patched += 1
    if patched != 2:
        raise RuntimeError(f"expected to patch 2 finger pad geoms, patched {patched}")


def write_door_scene(
    geometry: dict,
    arm_kp: float = ARM_KP_DOOR,
    arm_kv: float = ARM_KV_DOOR,
    hinge_damping: float = HINGE_DAMPING,
    hinge_frictionloss: float = HINGE_FRICTIONLOSS,
    finger_pad_half_z: float | None = None,
    scene_name: str = "g1_grasp_scene_door.xml",
) -> Path:
    """Task 3 scene: write_grasp_scene_5a()'s own output, plus a passive
    hinged door. `geometry` is a dict shaped like select_door_geometry()'s
    return value (pivot_xy, radius_m, phi0_deg, theta_deg, handle_z).

    write_grasp_scene_5a/_4b are ALWAYS called with the standard, unchanged
    ARM_KP_4B/ARM_KV_4B -- never with this function's own arm_kp/arm_kv --
    so g1_grasp_scene_4b.xml and g1_grasp_scene_5a.xml (which
    write_grasp_scene_5a regenerates internally at a HARDCODED filename
    regardless of this function's own `scene_name`) are never rewritten
    with a different gain. This function's arm_kp/arm_kv are instead
    applied as a task-local patch (_override_right_arm_gains) on THIS
    scene's own copy of the tree, after re-parsing, following the same
    "never touch the shared upstream file" discipline
    task2_language_selection.write_task2_scene established for its second
    cube.
    """
    base = write_grasp_scene_5a(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml")
    tree = ET.parse(base)
    root = tree.getroot()
    if (arm_kp, arm_kv) != (ARM_KP_4B, ARM_KV_4B):
        _override_right_arm_gains(root, arm_kp, arm_kv)
    _override_finger_closing_range(root, FINGER_EXTRA_CLOSING_M)
    if finger_pad_half_z is not None:
        _override_finger_pad_height(root, finger_pad_half_z)

    pivot_x, pivot_y = geometry["pivot_xy"]
    radius = geometry["radius_m"]
    phi0 = geometry["phi0_deg"]
    theta = geometry["theta_deg"]
    z = geometry.get("handle_z", ARC_HANDLE_Z)
    panel_center_z = z  # handle sits at panel mid-height; panel spans +/- PANEL_HALF_HEIGHT_M

    asset = root.find("asset")
    if asset is None:
        raise RuntimeError("expected asset section from write_grasp_scene_5a()")
    ET.SubElement(asset, "material", {"name": "door_frame_mat", "rgba": "0.35 0.35 0.38 1"})
    ET.SubElement(asset, "material", {"name": "door_panel_mat", "rgba": "0.75 0.72 0.62 1"})
    ET.SubElement(asset, "material", {"name": "door_handle_mat", "rgba": "0.85 0.55 0.10 1"})

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("expected worldbody from write_grasp_scene_5a()")

    # Frame: a single jointless jamb post at the pivot, world-fixed by
    # having no <joint> child at all -- the same "no joint = rigidly welded
    # to its parent" pattern _add_target_pad already uses for the target
    # pad, just applied to a body instead of a bare geom.
    frame = ET.SubElement(
        worldbody, "body",
        {"name": "door_frame", "pos": f"{pivot_x} {pivot_y} {TABLE_TOP_Z}"},
    )
    ET.SubElement(
        frame, "geom",
        {
            "name": "door_frame_jamb", "type": "cylinder",
            "size": f"{FRAME_THICKNESS_M} {PANEL_HALF_HEIGHT_M + 0.02}",
            "pos": f"0 0 {PANEL_HALF_HEIGHT_M + 0.02}",
            "material": "door_frame_mat", "contype": "1", "conaffinity": "1",
        },
    )

    # Panel: exactly one hinge joint about the world +Z axis through the
    # pivot, at the closed angle phi0. No actuator anywhere -- opening it
    # requires real contact force through the gripper, structurally, not
    # just by omission of a control signal.
    panel = ET.SubElement(
        worldbody, "body",
        {
            "name": "door_panel",
            "pos": f"{pivot_x} {pivot_y} {panel_center_z}",
            "euler": f"0 0 {np.radians(phi0):.6f}",  # compiler angle="radian" -- phi0 is in degrees
        },
    )
    ET.SubElement(
        panel, "joint",
        {
            "name": "door_hinge", "type": "hinge", "axis": "0 0 1",
            "pos": "0 0 0", "range": f"0 {np.radians(theta):.6f}",
            "damping": str(hinge_damping), "frictionloss": str(hinge_frictionloss),
        },
    )
    # The panel deliberately stops well short of the handle's own position
    # (a real clearance gap, not just a thin handle standoff) -- measured
    # necessary: an earlier version extended the panel box all the way to
    # the handle, and it collided with the closing gripper's fingers
    # (up to -9.6mm penetration against the idle finger across the hinge
    # sweep). Mirrors how a real cabinet door mounts its handle on a
    # bracket set back from the panel edge, not flush with it. The
    # handle's own position is untouched -- it stays exactly on the
    # Phase 7A-derived circle -- only the panel's box shrinks.
    panel_half_x = max(0.02, (radius - HANDLE_RADIUS_M - PANEL_HANDLE_CLEARANCE_M) / 2.0)
    ET.SubElement(
        panel, "geom",
        {
            "name": "door_panel_geom", "type": "box",
            "pos": f"{panel_half_x} 0 0",
            "size": f"{panel_half_x} {PANEL_THICKNESS_M} {PANEL_HALF_HEIGHT_M}",
            "mass": "0.4", "material": "door_panel_mat",
            "contype": "1", "conaffinity": "1", "friction": "0.5 0.005 0.0001",
        },
    )
    ET.SubElement(
        panel, "geom",
        {
            "name": "door_handle_geom", "type": "cylinder",
            "pos": f"{radius} 0 0",
            "size": f"{HANDLE_RADIUS_M} {HANDLE_HALF_LENGTH_M}",
            "mass": "0.05", "material": "door_handle_mat",
            "contype": "1", "conaffinity": "1", "friction": CUBE_FRICTION,
        },
    )

    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    ET.SubElement(contact, "exclude", {"body1": "door_frame", "body2": "door_panel"})

    out = TASK_DIR / scene_name
    tree.write(out, encoding="utf-8", xml_declaration=False)
    return out


def diagnose_door_reachability(
    model, arm_map: JointMap, site_id: int, base_qpos: np.ndarray,
    geometry: dict, n_samples: int = 13,
) -> dict:
    """Pre-simulation, warm-chained reachability check along the door's
    closed-to-open arc plus PREGRASP/RETREAT standoffs -- shaped exactly
    like run_pick_place.diagnose_pick_place_reachability, reused as the
    zero-physics-cost reject filter for this task instead of a new solver.
    """
    scratch = mujoco.MjData(model)
    nominal_q = np.zeros(len(arm_map.names))
    pivot_xy = geometry["pivot_xy"]
    radius = geometry["radius_m"]
    phi0 = geometry["phi0_deg"]
    theta = geometry["theta_deg"]
    z = geometry.get("handle_z", ARC_HANDLE_Z)

    closed = handle_pose(pivot_xy, radius, phi0, z)
    # PREGRASP stands off ALONG THE ARC's own tangent (a few degrees before
    # the closed angle, same radius, same z) rather than lifting vertically.
    # Phase 7A found this workspace's orientation conditioning is sharply
    # height-dependent -- a vertical standoff from z=0.90 (the well-behaved
    # band) quickly re-enters the poorly-conditioned region above it. An
    # in-plane standoff stays inside the same band the whole arc already
    # occupies, and is also the physically natural way to approach a
    # cylindrical handle that the gripper will close around.
    pregrasp = handle_pose(pivot_xy, radius, phi0 - 8.0, z)
    waypoints = {"PREGRASP_HANDLE": pregrasp, "APPROACH_HANDLE": closed}
    for i in range(n_samples):
        s = theta * i / (n_samples - 1)
        waypoints[f"ARC_{i:02d}"] = handle_pose(pivot_xy, radius, phi0 + s, z)
    open_pose = handle_pose(pivot_xy, radius, phi0 + theta, z)
    waypoints["RETREAT"] = open_pose + np.array([-0.10, 0.0, 0.0])

    report = {}
    q_prev = base_qpos.copy()
    for name, target in waypoints.items():
        q, resid, iters, orient_resid = solve_ik_waypoint_oriented(
            model, scratch, q_prev, arm_map, site_id, target, nominal_q
        )
        reachable = resid < IK_POS_TOL
        oriented_ok = orient_resid < ORIENT_TOL_RAD
        report[name] = {
            "target_pos": target.tolist(),
            "residual_m": resid,
            "orientation_residual_rad": orient_resid,
            "orientation_residual_deg": float(np.degrees(orient_resid)),
            "iterations": iters,
            "reachable_within_tol": bool(reachable),
            "orientation_within_tol": bool(oriented_ok),
        }
        scratch.qpos[:] = q_prev
        arm_map.set_qpos(scratch, q)
        q_prev = scratch.qpos.copy()
    report["all_reachable"] = all(
        v["reachable_within_tol"] for v in report.values() if isinstance(v, dict)
    )
    report["all_position_and_orientation_reachable"] = all(
        v["reachable_within_tol"] and v["orientation_within_tol"]
        for v in report.values() if isinstance(v, dict)
    )
    return report


class HingeInitGuard:
    """Anti-teleport guard for a single hinge DOF, mirroring
    run_grasp_test.CubeInitGuard's contract exactly but sized for a 1-qpos/
    1-dof hinge instead of a 7-qpos/6-dof free joint: unlimited writes
    before lock(), RuntimeError after. Deliberately a separate class rather
    than a generalisation of CubeInitGuard -- the slice widths differ, and
    silently writing a scalar into what a reader expects to be a 7-wide
    slice is exactly the kind of bug a shared base class would invite.
    """

    def __init__(self, data: "mujoco.MjData", qpos_adr: int, dof_adr: int):
        self._data = data
        self._qpos_adr = qpos_adr
        self._dof_adr = dof_adr
        self._locked = False

    def set_initial_angle(self, theta: float) -> None:
        if self._locked:
            raise RuntimeError("hinge initialization boundary violated: set after lock()")
        self._data.qpos[self._qpos_adr] = theta

    def set_initial_velocity(self, omega: float = 0.0) -> None:
        if self._locked:
            raise RuntimeError("hinge initialization boundary violated: set after lock()")
        self._data.qvel[self._dof_adr] = omega

    def lock(self) -> None:
        self._locked = True


def _drive_arc_smooth(
    model, data, ik_scratch, arm_map, gripper_map, site_id, nominal_q,
    telemetry, steps_run, guard, frame_callback,
    pivot_xy, radius, phi_start_deg, phi_end_deg, z, finger_open, phase,
    total_duration_s, n_waypoints, use_oriented_ik, carrying=None,
) -> np.ndarray:
    """Arc analogue of run_pick_place._drive_smooth. Deliberately kept as a
    SEPARATE function with the SAME body shape rather than a modification
    of _drive_smooth (which is a closure nested inside run_trial_pick_place
    and cannot be imported/extended from another module). The only
    substantive difference from _drive_smooth is the one line computing
    `waypoint`: a straight Cartesian interpolation there, a point on the
    hinge's own circle here. Everything else -- the per-sub-waypoint IK
    chaining and the linear joint-reference ramp
    (`ramped_target = q_prev_target + beta*(q_i - q_prev_target)`), which
    Phase 4E's evidence showed is what prevents centimetres of slip on long
    moves -- is reproduced verbatim. `test_drive_arc_matches_task1_drive_smooth_shape`
    in tests/test_door_open.py checks this textually.
    """
    q_prev_target = arm_map.get_qpos(data).copy()
    seg_duration = total_duration_s / n_waypoints
    n_substeps = max(1, int(round(seg_duration / TIMESTEP)))
    q_final = None
    for i in range(n_waypoints):
        alpha = (i + 1) / n_waypoints
        phi = phi_start_deg + alpha * (phi_end_deg - phi_start_deg)
        waypoint = handle_pose(pivot_xy, radius, phi, z)
        q_i, _, _, orient_resid = _solve_waypoint(
            model, ik_scratch, data.qpos.copy(), arm_map, site_id, waypoint, nominal_q, use_oriented_ik
        )
        qpos_errs = []
        for s in range(n_substeps):
            beta = (s + 1) / n_substeps
            ramped_target = q_prev_target + beta * (q_i - q_prev_target)
            _door_step_once(
                model, data, arm_map, gripper_map, site_id, ramped_target, finger_open,
                f"{phase}_wp{i}", telemetry, steps_run, guard, frame_callback, carrying=carrying,
            )
            qpos_errs.append(np.abs(ramped_target - arm_map.get_qpos(data)))
        qpos_errs = np.array(qpos_errs)
        telemetry["arm_tracking_error_rms"][f"{phase}_wp{i}"] = float(np.sqrt(np.mean(np.array(qpos_errs) ** 2)))
        telemetry["arm_tracking_error_max"][f"{phase}_wp{i}"] = float(np.max(qpos_errs))
        telemetry["orientation_residual_rad"][f"{phase}_wp{i}"] = orient_resid
        q_prev_target = q_i
        q_final = q_i
    return q_final


def _door_step_once(
    model, data, arm_map, gripper_map, site_id, arm_ctrl_target, finger_open, phase,
    telemetry, steps_run, guard, frame_callback, carrying=None,
) -> None:
    """One physics step plus the door task's telemetry accumulation --
    shaped like run_pick_place._step_once, adapted from cube-pose tracking
    to handle/hinge tracking. A module-level function (not a closure) so
    _drive_arc_smooth and the segment/settle helpers below can all share it
    without re-deriving per-call state.
    """
    arm_map.set_ctrl(data, arm_ctrl_target)
    bounded_pd_step(
        gripper_map, data, _finger_targets(gripper_map, finger_open),
        GRIPPER_KP_DOOR, GRIPPER_KD_DOOR, GRIPPER_MAX_STEP, GRIPPER_MAX_QVEL,
    )
    if not np.all(np.isfinite(data.ctrl)):
        telemetry["finite_and_bounded"] = False
    mujoco.mj_step(model, data)
    if frame_callback is not None:
        frame_callback(phase, model, data)
    steps_run[0] += 1
    if steps_run[0] == 1:
        guard.lock()

    if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
        telemetry["finite_and_bounded"] = False

    handle_pos = data.geom_xpos[telemetry["_handle_geom_id"]].copy()
    hinge_qpos = float(data.qpos[telemetry["_hinge_qpos_adr"]])
    hinge_qvel = float(data.qvel[telemetry["_hinge_dof_adr"]])
    telemetry["max_hinge_qpos"] = max(telemetry["max_hinge_qpos"], hinge_qpos)
    telemetry["final_hinge_qpos"] = hinge_qpos
    telemetry["final_hinge_qvel"] = hinge_qvel

    left_contact = _contacts_between(data, model, telemetry["_handle_geom_id"], telemetry["_left_pad_id"])
    right_contact = _contacts_between(data, model, telemetry["_handle_geom_id"], telemetry["_right_pad_id"])
    both_now = left_contact and right_contact
    if both_now:
        telemetry["both_pads_contact_handle"] = True
    if carrying in ("full", "grip_only") and not both_now:
        telemetry["contact_lost_during_arc"] = True

    if carrying in ("full", "grip_only"):
        tcp_rot = data.site_xmat[site_id].reshape(3, 3)
        for pad_id in (telemetry["_left_pad_id"], telemetry["_right_pad_id"]):
            for ci in range(data.ncon):
                con = data.contact[ci]
                pair = (int(con.geom1), int(con.geom2))
                if telemetry["_handle_geom_id"] not in pair or pad_id not in pair:
                    continue
                force6 = np.zeros(6)
                mujoco.mj_contactForce(model, data, ci, force6)
                normal_n = float(abs(force6[0]))
                telemetry["min_bilateral_normal_force_n"] = min(
                    telemetry["min_bilateral_normal_force_n"], normal_n
                )
        if telemetry["grasp_offset_ref"] is not None:
            tcp_pos = data.site_xpos[site_id]
            local_offset_now = tcp_local_cube_offset(tcp_pos, tcp_rot, handle_pos)
            slip = relative_slip_m(local_offset_now, telemetry["grasp_offset_ref"])
            telemetry["max_handle_slip_m"] = max(telemetry["max_handle_slip_m"], slip)

    if telemetry.get("_cube_body_id") is not None:
        cube_xy_now = data.xpos[telemetry["_cube_body_id"]][:2].copy()
        disp = float(np.linalg.norm(cube_xy_now - telemetry["_cube_initial_xy"]))
        telemetry["inert_cube_max_displacement_m"] = max(
            telemetry.get("inert_cube_max_displacement_m", 0.0), disp
        )


def _door_drive_segment(
    model, data, arm_map, gripper_map, site_id, target_pos, finger_open, phase, duration_s,
    joint_target, telemetry, steps_run, guard, frame_callback, carrying=None,
) -> None:
    n = max(1, int(round(duration_s / TIMESTEP)))
    qpos_errs = []
    for _ in range(n):
        _door_step_once(
            model, data, arm_map, gripper_map, site_id, joint_target, finger_open, phase,
            telemetry, steps_run, guard, frame_callback, carrying=carrying,
        )
        qpos_errs.append(np.abs(joint_target - arm_map.get_qpos(data)))
    qpos_errs = np.array(qpos_errs)
    telemetry["arm_tracking_error_rms"][phase] = float(np.sqrt(np.mean(qpos_errs ** 2)))
    telemetry["arm_tracking_error_max"][phase] = float(np.max(qpos_errs))


def _door_settle(
    model, data, arm_map, gripper_map, site_id, target_pos, finger_open, phase, joint_target,
    telemetry, steps_run, guard, frame_callback, carrying=None,
) -> bool:
    max_steps = int(round(SETTLE_MAX_EXTRA_S / TIMESTEP))
    settled = False
    n_steps = 0
    for _ in range(max_steps):
        _door_step_once(
            model, data, arm_map, gripper_map, site_id, joint_target, finger_open, phase,
            telemetry, steps_run, guard, frame_callback, carrying=carrying,
        )
        n_steps += 1
        tcp_pos = data.site_xpos[site_id]
        pos_err = float(np.linalg.norm(target_pos - tcp_pos))
        joint_speed = float(np.max(np.abs(arm_map.get_qvel(data))))
        if pos_err <= SETTLE_TCP_POS_TOL and joint_speed <= SETTLE_ARM_QVEL_TOL:
            settled = True
            break
    telemetry["settle_extra_s"][phase] = n_steps * TIMESTEP
    return settled


def _door_open_dwell(
    model, data, arm_map, gripper_map, site_id, joint_target, telemetry, steps_run,
    guard, frame_callback, open_threshold_rad,
) -> bool:
    """Continuous-streak success pattern, mirroring
    run_pick_place._task_success_dwell exactly: hold RETREAT's pose and
    require every door-open condition to hold CONTINUOUSLY for
    DOOR_DWELL_S, not merely once. Any single-step failure resets the
    streak.
    """
    max_steps = int(round(DOOR_DWELL_MAX_WAIT_S / TIMESTEP))
    need_steps = int(round(DOOR_DWELL_S / TIMESTEP))
    streak = 0
    n_steps = 0
    for _ in range(max_steps):
        _door_step_once(
            model, data, arm_map, gripper_map, site_id, joint_target, True, "VERIFY_TASK_SUCCESS",
            telemetry, steps_run, guard, frame_callback, carrying=None,
        )
        n_steps += 1
        hinge_qpos = telemetry["final_hinge_qpos"]
        hinge_qvel = telemetry["final_hinge_qvel"]
        released = not (
            _contacts_between(data, model, telemetry["_handle_geom_id"], telemetry["_left_pad_id"])
            or _contacts_between(data, model, telemetry["_handle_geom_id"], telemetry["_right_pad_id"])
        )
        ok = (
            hinge_qpos >= open_threshold_rad
            and abs(hinge_qvel) <= DOOR_STATIC_QVEL_TOL
            and released
        )
        streak = streak + 1 if ok else 0
        if streak >= need_steps:
            telemetry["door_open_dwell_achieved_s"] = streak * TIMESTEP
            return True
    telemetry["door_open_dwell_achieved_s"] = streak * TIMESTEP
    return False


def run_trial_door_open(
    model_path: Path,
    geometry: dict,
    initial_hinge_angle_rad: float = 0.0,
    use_oriented_ik: bool = True,
    frame_callback=None,
) -> dict:
    """One door-opening trial: PREGRASP_HANDLE -> ... -> DONE/FAILED.

    Shaped like run_pick_place.run_trial_pick_place -- same closure-free,
    module-level-helper structure, same _fail/_finalize pattern -- but
    reads a hinge instead of a free-body cube, and its success criteria
    (criteria_door) are a LIBERO-style joint-position threshold rather than
    a placement-XY margin. `use_oriented_ik` defaults to True here (unlike
    Task 1's False): Phase 7A/7B's own reachability check
    (diagnose_door_reachability) already confirmed the locked arc meets
    both the position AND orientation tolerance at every sampled waypoint,
    which Task 1's table-height grasp never achieves anywhere in the
    measured workspace (reports/phase7a-workspace-map.md).
    """
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    ik_scratch = mujoco.MjData(model)

    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    gripper_map = JointMap.build(model, GRIPPER_JOINTS, GRIPPER_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    nominal_q = np.zeros(len(RIGHT_ARM_JOINTS))

    handle_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "door_handle_geom")
    left_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
    right_pad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")
    hinge_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "door_hinge")
    hinge_qpos_adr = int(model.jnt_qposadr[hinge_id])
    hinge_dof_adr = int(model.jnt_dofadr[hinge_id])
    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")

    pivot_xy = tuple(geometry["pivot_xy"])
    radius = geometry["radius_m"]
    phi0 = geometry["phi0_deg"]
    theta = geometry["theta_deg"]
    z = geometry.get("handle_z", ARC_HANDLE_Z)
    open_threshold_rad = DOOR_OPEN_FRACTION * np.radians(theta)

    # --- RESET ---
    mujoco.mj_resetData(model, data)
    guard = HingeInitGuard(data, hinge_qpos_adr, hinge_dof_adr)
    guard.set_initial_angle(initial_hinge_angle_rad)
    mujoco.mj_forward(model, data)
    steps_run = [0]
    reset_qpos = data.qpos.copy()
    # Read the door's LIVE hinge angle at reset, exactly as Task 1/2 read
    # the live cube pose (data.xpos[cube_body_id]) rather than assume a
    # nominal spawn position. Every waypoint below targets the handle's
    # ACTUAL current position (phi0 + this offset), not the geometry's
    # nominal closed angle -- found necessary by direct measurement: with
    # waypoints fixed to the nominal phi0 regardless of a non-zero
    # initial_hinge_angle_rad, the closing grip's contact dynamics
    # mechanically dragged an already-ajar door back toward the nominal
    # angle before VERIFY_BILATERAL_HANDLE_CONTACT's corridor check ever
    # saw it, silently defeating that check's purpose for every tested
    # offset (0.04/0.08/0.12 rad all read back as "was closed"). See
    # reports/phase7d-door-tests.md.
    phi0_live_deg = phi0 + np.degrees(float(data.qpos[hinge_qpos_adr]))

    telemetry = {
        "states_entered": ["RESET"],
        "failure_state": None, "failure_reason": None,
        "finite_and_bounded": True,
        "both_pads_contact_handle": False,
        "contact_lost_during_arc": False,
        "max_hinge_qpos": float(data.qpos[hinge_qpos_adr]),
        "final_hinge_qpos": float(data.qpos[hinge_qpos_adr]),
        "final_hinge_qvel": 0.0,
        "hinge_qpos_at_close": None,
        "min_bilateral_normal_force_n": float("inf"),
        "max_handle_slip_m": 0.0,
        "grasp_offset_ref": None,
        "released_after_open": None,
        "door_open_dwell_achieved_s": 0.0,
        "arm_tracking_error_rms": {}, "arm_tracking_error_max": {},
        "settle_extra_s": {}, "orientation_residual_rad": {},
        "_handle_geom_id": handle_geom_id, "_left_pad_id": left_pad_id, "_right_pad_id": right_pad_id,
        "_hinge_qpos_adr": hinge_qpos_adr, "_hinge_dof_adr": hinge_dof_adr,
        "_cube_body_id": cube_body_id if cube_body_id >= 0 else None,
        "_cube_initial_xy": data.xpos[cube_body_id][:2].copy() if cube_body_id >= 0 else None,
        "inert_cube_max_displacement_m": 0.0,
    }

    def _fail(state: str, reason: str) -> dict:
        telemetry["failure_state"] = state
        telemetry["failure_reason"] = reason
        return _finalize(task_success=False)

    def _finalize(task_success: bool) -> dict:
        criteria_door = {
            "hinge_qpos_ge_open_threshold": telemetry["max_hinge_qpos"] >= open_threshold_rad,
            "door_open_held_ge_2s_continuous": telemetry["door_open_dwell_achieved_s"] >= DOOR_DWELL_S,
            "door_closed_at_verify_contact": (
                # Displacement from THIS TRIAL's own declared starting
                # angle, not from an assumed zero -- a trial can legally
                # start already-ajar (the diagnostic probes in
                # DOOR_EVAL_INITIAL_ANGLES_RAD do exactly that). Comparing
                # against absolute zero would silently pass any trial
                # whose approach happened to drag the door back near
                # zero before this check ran, which measurement showed
                # really happens (an open gripper can brush the panel
                # during PREGRASP/APPROACH). This is the same "compare to
                # this trial's own initial condition" pattern
                # HANDLE_GRASP_CORRIDOR_RAD's CLOSE-phase check already
                # uses via hinge_before_close, made consistent here.
                telemetry["hinge_qpos_at_close"] is not None
                and abs(telemetry["hinge_qpos_at_close"] - initial_hinge_angle_rad) <= HANDLE_GRASP_CORRIDOR_RAD
            ),
            "both_pads_contact_handle_at_close": telemetry["both_pads_contact_handle"],
            "bilateral_contact_retained_through_arc": not telemetry["contact_lost_during_arc"],
            "max_handle_slip_le_10mm": telemetry["max_handle_slip_m"] <= HANDLE_SLIP_TOL_M,
            "normal_forces_positive_and_finite": (
                np.isfinite(telemetry["min_bilateral_normal_force_n"])
                and telemetry["min_bilateral_normal_force_n"] > 0.0
            ),
            "released_after_open": bool(telemetry["released_after_open"]) if telemetry["released_after_open"] is not None else False,
            "door_static_after_release": abs(telemetry["final_hinge_qvel"]) <= DOOR_STATIC_QVEL_TOL,
            "door_did_not_reclose_during_retreat": (
                telemetry["max_hinge_qpos"] - telemetry["final_hinge_qpos"]
            ) <= 0.05,
            "finite_and_bounded": telemetry["finite_and_bounded"],
        }
        door_pass = all(criteria_door.values()) and task_success
        result = {
            "criteria_door": criteria_door,
            "door_pass": door_pass,
            "task_success": task_success,
            "geometry": geometry,
            "initial_hinge_angle_rad": initial_hinge_angle_rad,
            "open_threshold_rad": float(open_threshold_rad),
        }
        # Strip internal bookkeeping keys (leading underscore) before
        # exposing telemetry, and drop the (large, redundant) per-waypoint
        # dicts' internal-only entries -- everything else is reported.
        clean = {k: v for k, v in telemetry.items() if not k.startswith("_")}
        if isinstance(clean.get("grasp_offset_ref"), np.ndarray):
            clean["grasp_offset_ref"] = clean["grasp_offset_ref"].tolist()
        result["telemetry"] = clean
        return result

    step_args = (model, data, arm_map, gripper_map, site_id)
    common = (telemetry, steps_run, guard, frame_callback)

    # --- PREGRASP_HANDLE ---
    telemetry["states_entered"].append("PREGRASP_HANDLE")
    pregrasp_pos = handle_pose(pivot_xy, radius, phi0_live_deg - PREGRASP_STANDOFF_DEG, z)
    q_pregrasp, _, _, orient_pg = _solve_waypoint(
        model, ik_scratch, reset_qpos, arm_map, site_id, pregrasp_pos, nominal_q, use_oriented_ik
    )
    telemetry["orientation_residual_rad"]["PREGRASP_HANDLE"] = orient_pg
    _door_drive_segment(*step_args, pregrasp_pos, True, "PREGRASP_HANDLE", DRIVE_S_DOOR["PREGRASP_HANDLE"], q_pregrasp, *common)

    telemetry["states_entered"].append("SETTLE_PREGRASP")
    if not _door_settle(*step_args, pregrasp_pos, True, "SETTLE_PREGRASP", q_pregrasp, *common):
        return _fail("SETTLE_PREGRASP", "TCP did not settle within tolerance before APPROACH_HANDLE")

    # --- APPROACH_HANDLE ---
    telemetry["states_entered"].append("APPROACH_HANDLE")
    closed_pos = handle_pose(pivot_xy, radius, phi0_live_deg, z)
    # Solved from reset_qpos (a fixed, consistent reference), NOT from the
    # arm's current post-PREGRASP configuration. Measured directly: warm-
    # starting this solve from the chained PREGRASP result lands in a
    # different (still position-valid) redundancy resolution than a fresh
    # solve, off by up to 0.04 rad across the 7 joints -- enough that one
    # finger pad reaches the cylindrical handle and the other stops just
    # short of it, an asymmetry the arm-gain and finger-travel attempts
    # (see ARM_KP_DOOR, FINGER_EXTRA_CLOSING_M) did not fix because the
    # actuator is force- not position-limited. Solving both discrete
    # pre-grasp waypoints from the same reset reference (attempt 3 of the
    # calibration budget) reliably reproduced bilateral contact.
    q_approach, _, _, orient_ap = _solve_waypoint(
        model, ik_scratch, reset_qpos, arm_map, site_id, closed_pos, nominal_q, use_oriented_ik
    )
    telemetry["orientation_residual_rad"]["APPROACH_HANDLE"] = orient_ap
    _door_drive_segment(*step_args, closed_pos, True, "APPROACH_HANDLE", DRIVE_S_DOOR["APPROACH_HANDLE"], q_approach, *common)

    telemetry["states_entered"].append("SETTLE_APPROACH")
    if not _door_settle(*step_args, closed_pos, True, "SETTLE_APPROACH", q_approach, *common):
        return _fail("SETTLE_APPROACH", "TCP did not settle within tolerance before CLOSE")

    # --- CLOSE ---
    telemetry["states_entered"].append("CLOSE")
    hinge_before_close = float(data.qpos[hinge_qpos_adr])
    _door_drive_segment(*step_args, closed_pos, False, "CLOSE", DRIVE_S_DOOR["CLOSE"], q_approach, *common)

    # --- VERIFY_BILATERAL_HANDLE_CONTACT ---
    telemetry["states_entered"].append("VERIFY_BILATERAL_HANDLE_CONTACT")
    left_now = _contacts_between(data, model, handle_geom_id, left_pad_id)
    right_now = _contacts_between(data, model, handle_geom_id, right_pad_id)
    finger_qvel = gripper_map.get_qvel(data)
    closing_settled = bool(np.all(np.abs(finger_qvel) < 0.03))
    hinge_displacement = abs(float(data.qpos[hinge_qpos_adr]) - hinge_before_close)
    telemetry["hinge_qpos_at_close"] = float(data.qpos[hinge_qpos_adr])
    if not (left_now and right_now):
        return _fail("VERIFY_BILATERAL_HANDLE_CONTACT", f"no simultaneous bilateral contact at verify time (left={left_now}, right={right_now})")
    if not closing_settled:
        return _fail("VERIFY_BILATERAL_HANDLE_CONTACT", f"finger closing velocity not settled: {finger_qvel.tolist()}")
    if hinge_displacement > HANDLE_GRASP_CORRIDOR_RAD:
        return _fail("VERIFY_BILATERAL_HANDLE_CONTACT", f"hinge displaced {hinge_displacement:.4f} rad > corridor {HANDLE_GRASP_CORRIDOR_RAD} rad before pull")
    telemetry["both_pads_contact_handle"] = True
    tcp_rot0 = data.site_xmat[site_id].reshape(3, 3)
    handle_pos0 = data.geom_xpos[handle_geom_id].copy()
    telemetry["grasp_offset_ref"] = tcp_local_cube_offset(data.site_xpos[site_id].copy(), tcp_rot0, handle_pos0)

    # --- PULL_ARC ---
    telemetry["states_entered"].append("PULL_ARC")
    q_open = _drive_arc_smooth(
        model, data, ik_scratch, arm_map, gripper_map, site_id, nominal_q,
        telemetry, steps_run, guard, frame_callback,
        pivot_xy, radius, phi0_live_deg, phi0 + theta, z, False, "PULL_ARC",
        PULL_ARC_DRIVE_S, PULL_ARC_N_WAYPOINTS, use_oriented_ik, carrying="grip_only",
    )
    if telemetry["contact_lost_during_arc"]:
        return _fail("PULL_ARC", "lost bilateral handle contact during the pull")

    # --- SETTLE_OPEN ---
    telemetry["states_entered"].append("SETTLE_OPEN")
    open_pos = handle_pose(pivot_xy, radius, phi0 + theta, z)
    if not _door_settle(*step_args, open_pos, False, "SETTLE_OPEN", q_open, *common, carrying="grip_only"):
        return _fail("SETTLE_OPEN", "TCP did not settle at the open handle position")

    # --- VERIFY_DOOR_OPEN ---
    telemetry["states_entered"].append("VERIFY_DOOR_OPEN")
    if telemetry["max_hinge_qpos"] < open_threshold_rad:
        return _fail(
            "VERIFY_DOOR_OPEN",
            f"hinge reached {telemetry['max_hinge_qpos']:.4f} rad, below the "
            f"{open_threshold_rad:.4f} rad open threshold",
        )

    # --- OPEN (fingers) ---
    telemetry["states_entered"].append("OPEN")
    _door_drive_segment(*step_args, open_pos, True, "OPEN", DRIVE_S_DOOR["OPEN"], q_open, *common)
    _door_drive_segment(*step_args, open_pos, True, "RELEASE_SETTLE", DRIVE_S_DOOR["RELEASE_SETTLE"], q_open, *common)

    # --- VERIFY_RELEASE ---
    telemetry["states_entered"].append("VERIFY_RELEASE")
    released = not (
        _contacts_between(data, model, handle_geom_id, left_pad_id)
        or _contacts_between(data, model, handle_geom_id, right_pad_id)
    )
    telemetry["released_after_open"] = released
    if not released:
        return _fail("VERIFY_RELEASE", "handle still in contact with a finger pad after open + settle")

    # --- RETREAT ---
    telemetry["states_entered"].append("RETREAT")
    retreat_pos = open_pos + np.array([-RETREAT_STANDOFF_M, 0.0, 0.0])
    q_retreat, _, _, _ = _solve_waypoint(
        model, ik_scratch, data.qpos.copy(), arm_map, site_id, retreat_pos, nominal_q, use_oriented_ik
    )
    _door_drive_segment(*step_args, retreat_pos, True, "RETREAT", RETREAT_DRIVE_S_DOOR, q_retreat, *common)

    # --- VERIFY_TASK_SUCCESS ---
    telemetry["states_entered"].append("VERIFY_TASK_SUCCESS")
    dwell_ok = _door_open_dwell(*step_args, q_retreat, telemetry, steps_run, guard, frame_callback, open_threshold_rad)
    if not dwell_ok:
        return _fail(
            "VERIFY_TASK_SUCCESS",
            f"door-open conditions did not hold continuously for {DOOR_DWELL_S}s "
            f"within {DOOR_DWELL_MAX_WAIT_S}s (best streak {telemetry['door_open_dwell_achieved_s']:.3f}s)",
        )

    telemetry["states_entered"].append("DONE")
    return _finalize(task_success=True)


# --- initialization-boundary self-audit -------------------------------------
# Mirrors run_pick_place._assert_run_trial_pick_place_has_no_direct_cube_state_write
# exactly in spirit (assignment, not comparison, and not a read -- `=(?!=)`
# lets `...].copy()` and other read-only slices through), but with one
# structural difference and one addition:
#
# STRUCTURAL DIFFERENCE: run_trial_pick_place's step/drive/settle helpers
# are closures NESTED inside it, so scanning its own source already covers
# them. This task's equivalents (_door_step_once, _door_drive_segment,
# _door_settle, _door_open_dwell, _drive_arc_smooth) are module-level
# functions instead (closures can't be shared with run_trial_door_open's
# own scope the way _drive_smooth's body is reused here), so a bug hidden
# in one of them would NOT show up if only run_trial_door_open's own
# source were scanned. All of them are scanned here.
#
# ADDITION: a free-body cube is only cheatable via a direct qpos/qvel
# write or xfrc_applied on its body. A 1-DOF hinge is ALSO trivially
# cheatable via qfrc_applied on its own dof (a generalized force on a
# single scalar coordinate) -- a cheat surface a free joint's 6 dofs don't
# reduce to as simply. That pattern is included below; Task 1's own guard
# does not need it, because CubeInitGuard's contract has no equivalent.
def _assert_door_trial_functions_have_no_direct_hinge_state_write() -> None:
    src = "\n".join(
        inspect.getsource(fn) for fn in (
            run_trial_door_open, _door_step_once, _door_drive_segment,
            _door_settle, _door_open_dwell, _drive_arc_smooth,
        )
    )
    forbidden_write_patterns = [
        r"data\.qpos\[.*hinge_qpos_adr[^\]]*\]\s*=(?!=)",
        r"data\.qvel\[.*hinge_dof_adr[^\]]*\]\s*=(?!=)",
        r"qfrc_applied\[.*hinge_dof_adr[^\]]*\]\s*=(?!=)",
        r"xfrc_applied\[.*door_panel[^\]]*\]\s*=(?!=)",
    ]
    for pattern in forbidden_write_patterns:
        if re.search(pattern, src):
            raise AssertionError(
                f"a door-trial function contains a direct hinge/panel-state write "
                f"(matches {pattern!r}) outside HingeInitGuard -- "
                "initialization boundary violated"
            )


_assert_door_trial_functions_have_no_direct_hinge_state_write()


# Configuration axis: initial hinge angle, set once through HingeInitGuard
# strictly before the first mj_step -- a legitimate initial condition,
# exactly the CubeInitGuard precedent Task 1/2 use for cube pose. 0.0 is
# the genuinely-closed nominal case; the other three are ALREADY-AJAR
# probes at and beyond HANDLE_GRASP_CORRIDOR_RAD (0.02), which the
# VERIFY_BILATERAL_HANDLE_CONTACT anti-cheat check must correctly reject
# (door_closed_at_verify_contact=False) rather than let something that
# was never really closed count as an opened door.
DOOR_EVAL_INITIAL_ANGLES_RAD = (0.00, 0.04, 0.08, 0.12)


def evaluate_door_configurations(scene_path: Path, geometry: dict, n_trials_per_config: int = 3) -> list[dict]:
    """Exactly the 4 configurations x n_trials_per_config deterministic
    repeats each (12 trials at the default). Same scene/controller
    parameters for every trial -- no per-configuration tuning of any kind.
    """
    all_results = []
    for initial_angle in DOOR_EVAL_INITIAL_ANGLES_RAD:
        for trial_index in range(n_trials_per_config):
            r = run_trial_door_open(scene_path, geometry, initial_hinge_angle_rad=initial_angle)
            r["trial_index"] = trial_index
            all_results.append(r)
    return all_results


def main() -> int:
    geometry = select_door_geometry()
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    (LOG_DIR / "phase7b_selected_door_geometry.json").write_text(
        json.dumps(geometry, indent=2, sort_keys=True)
    )
    print("selected geometry:", json.dumps(geometry, indent=2))

    scene_path = write_door_scene(geometry)
    print(f"scene written: {scene_path}")

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    mujoco.mj_resetData(model, data)
    report = diagnose_door_reachability(model, arm_map, site_id, data.qpos.copy(), geometry)
    print(f"all_reachable: {report['all_reachable']}")
    print(f"all_position_and_orientation_reachable: {report['all_position_and_orientation_reachable']}")
    (LOG_DIR / "phase7b_door_reachability_check.json").write_text(
        json.dumps(report, indent=2, sort_keys=True)
    )
    return 0 if report["all_reachable"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
