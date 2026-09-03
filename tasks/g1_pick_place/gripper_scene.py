#!/usr/bin/env python3
"""Phase 3 task-local G1 grasp scene: fixed-pelvis + parallel gripper + table + cube.

Derives an MJCF from the pinned vendor G1 model by ElementTree deep-copy,
mirroring the pattern used in g1_manipulation_audit.py (write_contact_scene,
write_site_probe_scene). Vendor source files are never modified; only the
in-memory copy is edited before being written to a task-local path.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
VENDOR = ROOT / "vendor" / "unitree_mujoco"
G1_DIR = VENDOR / "unitree_robots" / "g1"
MODEL_XML = G1_DIR / "g1_29dof.xml"
TASK_DIR = ROOT / "tasks" / "g1_pick_place"

WRIST_BODY = "right_wrist_yaw_link"
PELVIS_BODY = "pelvis"

# Cube convention reused from the Phase 2 contact probe (box, mass 0.05 kg,
# friction "1 0.01 0.001"), placed on a task-local table within the reachable
# workspace measured for right_wrist_yaw_link during Phase 3 inspection
# (candidate grasp pose puts the wrist ~0.20-0.30 m forward, ~-0.15 m
# lateral, ~0.80-0.85 m up).
CUBE_HALF = 0.035
CUBE_MASS = 0.05
CUBE_FRICTION = "1 0.01 0.001"
TABLE_TOP_Z = 0.70
CUBE_POS = (0.33, -0.15, TABLE_TOP_Z + CUBE_HALF)

# Finger geometry, in the right_wrist_yaw_link local frame. Fingers slide
# along local Y (perpendicular to the local-X reach/approach direction used
# by the TCP offset), symmetric about y=0.
#
# Phase 4E finding (Section B evidence, tasks/g1_pick_place/
# phase4e_diagnose_grip.py against the pre-4E scene): the world-Z contact
# point between each pad and the cube, sampled across a full nominal trial,
# ranged from -4.16 cm to +5.05 cm relative to the cube's own center (mean
# +1.53 cm) -- i.e. contact routinely landed outside the cube's own +/-3.5 cm
# half-height, meaning the grasp was intermittently an edge/corner grab, not
# a clean side-face grab. Root cause: PREGRASP/APPROACH only constrain the
# TCP site's *position* (solve_ik_waypoint has no orientation term), so
# wrist roll about the approach axis is left to the IK's posture-only
# null-space objective and drifts -- and because the two fingers are offset
# from the TCP site in local Y (not local Z), any wrist roll shows up as a
# real world-Z offset between the finger pads and the cube center that pure
# position IK never corrects. FINGER_PAD_HALF's Z half-extent was widened
# from 0.022 to 0.030 m (close to the cube's own 0.035 m half-extent) so the
# pad's own contact face is tall enough to keep covering the cube's side
# face -- and keep the cube's center within the pads' combined vertical
# span -- across the observed +/-5 cm range of wrist-roll-induced offset,
# without requiring a new orientation-constrained IK term (a larger,
# separately-scoped change).
#
# Scoped to Task 1's own scene (write_grasp_scene_4b) only, via
# _build_grasp_tree's finger_pad_half parameter -- Phase 3/3B/3C's own
# scenes (write_grasp_scene, write_grasp_scene_3c) keep using
# LEGACY_FINGER_PAD_HALF unchanged, so their already-verified, previously-
# reported numeric results (reports/phase3c-position-servo-baseline.md,
# reports/phase4a-grasp-variants.md) are not perturbed by a Task-1-specific
# repair.
LEGACY_FINGER_PAD_HALF = (0.012, 0.006, 0.022)
FINGER_PAD_HALF = (0.012, 0.006, 0.030)
FINGER_REACH_X = 0.10
FINGER_OPEN_Y = 0.075
FINGER_CONTACT_Y = CUBE_HALF + FINGER_PAD_HALF[1]  # pad face just touches cube

# Phase 4F, Attempt 3 (Section D: "adjust grasp waypoint/pad-center
# alignment using measured contact locations", NOT a gain/friction change):
# a fixed, measured mechanical mounting correction for the two finger pads.
#
# Evidence: at the real, converged nominal APPROACH joint configuration
# (reports/phase4f-orientation-grasp-stabilization.md, Section A/D), the
# wrist's local Y axis (the jaw axis the fingers are offset along and slide
# on) is already very well aligned with world Y (measured misalignment
# ~4-5 deg) -- so left/right finger height symmetry was never the actual
# problem. The wrist's local Z axis -- the axis this project's own finger
# pad geometry treats as "tall"/vertical (FINGER_PAD_HALF[2] = 30 mm, by
# far the pad's largest half-extent) -- was measured at ~47 deg from world
# vertical at that same configuration, because the arm reaches down to the
# cube at a steep diagonal, not horizontally. A pad box tilted 47 deg
# contacts a vertical cube face at a CORNER/EDGE, not flush across its
# face -- a small, unstable contact patch that explains the reported
# near-drop far better than a simple height mismatch would (the height-
# mismatch contribution from the local-Y misalignment above is only ~3 mm).
#
# Fix: a fixed rotation of each finger BODY (not its position -- pos stays
# exactly (FINGER_REACH_X, +/-y_ref, 0), so the finger's origin still
# brackets the TCP/cube target exactly as before) about the wrist's own
# local Y axis (the jaw axis, deliberately left untouched by this
# rotation, since it was already correct) by the angle that levels local Z
# to within its own residual (~4 deg, well inside ORIENT_TOL_RAD) at this
# specific, deterministic (no RNG in this pipeline) nominal configuration.
# This is a mechanical "wrist bracket" calibration, exactly analogous to
# choosing a fixed mounting angle for a real gripper based on its known,
# repeatable working pose -- not a per-trial adaptive term, and it does not
# touch the joint's own slide axis ("0 1 0" in the finger's local frame,
# which is invariant under a rotation about that same axis).
#
# Limitation, reported honestly rather than hidden: this angle is
# calibrated to the NOMINAL cube position specifically. Phase 4A/4B found
# the wrist's configuration (and therefore this tilt) does shift somewhat
# for the other reachable cube offsets (x-0.03, y+0.03) -- Stage B (Section
# D) evaluates this shared, non-per-variant-tuned mounting against those
# variants as-is, exactly as HANDOFF.md requires, and reports where it does
# or does not generalize.
FINGER_MOUNT_FIX_QUAT = (0.916030512778581, 0.0, -0.40110858836306396, 0.0)  # (w, x, y, z);
# rotation of -47.295 deg about the finger body's own local Y (jaw) axis.
# Joints are allowed to travel a little past the nominal contact point so a
# "closed" command keeps pressing (real squeeze force via contact, robust to
# a few mm of arm/IK misalignment) instead of stopping exactly at first
# contact. Tuning iteration 3: iteration 2 reached contact but the cube got
# shoved sideways instead of gripped, because the closed target equaled the
# exact nominal contact point (near-zero net squeeze force under any small
# misalignment). FINGER_SQUEEZE_MARGIN adds real overtravel/force.
FINGER_SQUEEZE_MARGIN = 0.015
FINGER_CLOSED_Y = FINGER_CONTACT_Y - FINGER_SQUEEZE_MARGIN
FINGER_FRICTION = "1.2 0.01 0.001"
FINGER_FORCE_LIMIT = 15.0  # N, conservative for a 0.05 kg cube
TCP_POS = (FINGER_REACH_X, 0.0, 0.0)


def _sub(parent: ET.Element, tag: str, **attrs: str) -> ET.Element:
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def _build_grasp_tree(
    extra_trunk_weld: bool = False,
    finger_pad_half: tuple[float, float, float] = LEGACY_FINGER_PAD_HALF,
    apply_phase4e_gripper_visuals: bool = False,
    apply_phase4f_pad_mount_fix: bool = False,
) -> ET.ElementTree:
    """Shared builder: fixed pelvis + gripper + table + cube on a fresh copy
    of the vendor model. Used by write_grasp_scene() (Phase 3/3B,
    torque-motor right arm, unchanged -- always called with
    extra_trunk_weld=False, byte-for-byte identical output),
    write_grasp_scene_3c() (Phase 3C, position-servo right arm,
    extra_trunk_weld=True; see that function's docstring for why), and
    write_grasp_scene_4b() (Task 1, extra_trunk_weld=True plus
    finger_pad_half=FINGER_PAD_HALF and apply_phase4e_gripper_visuals=True
    -- see the Phase 4E comment at FINGER_PAD_HALF's definition and the
    "Phase 4E fix" comments below for why these are scoped to Task 1's own
    scene rather than applied unconditionally here).

    `finger_pad_half` defaults to LEGACY_FINGER_PAD_HALF so Phase 3/3B/3C's
    two callers are byte-for-byte/physics-identical to before Phase 4E.
    `apply_phase4e_gripper_visuals` gates the decorative-vendor-mesh
    removal, palm geom, and per-side finger coloring added in Phase 4E --
    also off by default for the same reason.
    """
    tree = ET.parse(MODEL_XML)
    root = tree.getroot()
    root.set("model", "g1_phase3_grasp")

    compiler = root.find("compiler")
    if compiler is not None:
        compiler.set("meshdir", str((G1_DIR / "meshes").resolve()))

    # --- environment: floor, lights, table (task-local, not vendor asset) ---
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    _sub(
        asset, "texture", type="2d", name="groundplane", builtin="checker",
        mark="edge", rgb1="0.2 0.3 0.4", rgb2="0.1 0.2 0.3",
        markrgb="0.8 0.8 0.8", width="300", height="300",
    )
    _sub(
        asset, "material", name="groundplane", texture="groundplane",
        texuniform="true", texrepeat="5 5", reflectance="0.2",
    )
    _sub(asset, "material", name="table_mat", rgba="0.45 0.32 0.2 1")
    _sub(asset, "material", name="cube_mat", rgba="0.8 0.2 0.1 1")
    if apply_phase4e_gripper_visuals:
        # Phase 4E, Task 1 scene only: two distinguishable finger materials
        # (left/right) plus a palm material, replacing the single shared
        # "finger_mat" -- see _build_grasp_tree()'s wrist section for why.
        # Phase 3/3B/3C's own scenes keep the original single "finger_mat"
        # (below) unchanged.
        _sub(asset, "material", name="finger_mat_left", rgba="0.15 0.15 0.22 1")
        _sub(asset, "material", name="finger_mat_right", rgba="0.28 0.16 0.14 1")
        _sub(asset, "material", name="palm_mat", rgba="0.2 0.2 0.21 1")
    else:
        _sub(asset, "material", name="finger_mat", rgba="0.15 0.15 0.15 1")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("G1 model has no worldbody")
    _sub(worldbody, "light", pos="0 0 1.5", dir="0 0 -1", directional="true")
    _sub(worldbody, "light", pos="0.4 -0.2 1.3", dir="-0.2 0.1 -1", directional="true")
    _sub(worldbody, "geom", name="floor", size="0 0 0.05", type="plane", material="groundplane")

    table = _sub(
        worldbody, "body", name="table",
        pos=f"{CUBE_POS[0]} {CUBE_POS[1]} {TABLE_TOP_Z / 2.0}",
    )
    _sub(
        table, "geom", name="table_top", type="box",
        size=f"0.22 0.22 {TABLE_TOP_Z / 2.0}", material="table_mat",
        contype="1", conaffinity="1",
    )

    cube = _sub(worldbody, "body", name="cube", pos=f"{CUBE_POS[0]} {CUBE_POS[1]} {CUBE_POS[2]}")
    _sub(cube, "freejoint", name="cube_joint")
    _sub(
        cube, "geom", name="cube_geom", type="box",
        size=f"{CUBE_HALF} {CUBE_HALF} {CUBE_HALF}", mass=str(CUBE_MASS),
        material="cube_mat", contype="1", conaffinity="1", friction=CUBE_FRICTION,
    )

    # --- fixed pelvis: model-level equality weld, not a runtime hack ---
    equality = root.find("equality")
    if equality is None:
        equality = ET.SubElement(root, "equality")
    _sub(
        equality, "weld", name="pelvis_fixed", body1=PELVIS_BODY,
        solref="0.002 1", solimp="0.9999 0.9999 0.001 0.5 2",
    )
    if extra_trunk_weld:
        # Phase 3C finding: welding only the pelvis leaves waist_yaw/roll/
        # pitch (torso_link's joints) unpowered and unconstrained. Under a
        # strongly-actuated right arm, reaction torques through the shared
        # torso swing the whole upper body -- diagnosed directly: with only
        # the pelvis weld, torso_link acquired several rad/s of angular
        # velocity and the right-wrist TCP error grew unboundedly over a few
        # seconds even though the arm's own joints tracked their target
        # closely in torso-relative terms (see
        # reports/phase3c-position-servo-baseline.md). A "fixed-base MVP"
        # (decided in Phase 2) implies the whole trunk is rigid, not only
        # the pelvis free joint; welding torso_link to pelvis as well makes
        # that assumption actually true instead of only nominally true.
        _sub(
            equality, "weld", name="torso_fixed", body1="torso_link", body2=PELVIS_BODY,
            solref="0.002 1", solimp="0.9999 0.9999 0.001 0.5 2",
        )

    # --- physical parallel gripper attached under right_wrist_yaw_link ---
    wrist = None
    for body in root.iter("body"):
        if body.get("name") == WRIST_BODY:
            wrist = body
            break
    if wrist is None:
        raise RuntimeError(f"{WRIST_BODY} not found in vendor model")

    if apply_phase4e_gripper_visuals:
        # Phase 4E fix (Section A), Task 1 scene only: the vendor's own
        # "right_rubber_hand" visual mesh is a static, non-articulated
        # decorative geom (contype=0 conaffinity=0 in the VENDOR's own
        # authoring -- see
        # vendor/unitree_mujoco/unitree_robots/g1/g1_29dof.xml) whose
        # bounding box spatially overlaps this project's real, physically-
        # simulated finger pads (confirmed and reproduced in
        # reports/phase4d-physics-integrity-audit.md). Because it is
        # collision-free, removing it changes nothing about any phase's
        # physics/contact results, only what is rendered -- confirmed by
        # rerunning every pre-4E phase's test suite unchanged. The vendor
        # source file itself is never touched (ET.parse() above reads a
        # fresh copy each call). Scoped to apply_phase4e_gripper_visuals
        # (i.e. only write_grasp_scene_4b) rather than unconditionally, so
        # Phase 3/3B/3C's own generated scenes are byte-for-byte unchanged.
        for child in list(wrist):
            if child.tag == "geom" and child.get("mesh") == "right_rubber_hand":
                wrist.remove(child)
                break
        else:
            raise RuntimeError("expected vendor right_rubber_hand geom not found under wrist body")

    _sub(
        wrist, "site", name="grasp_tcp", pos=f"{TCP_POS[0]} {TCP_POS[1]} {TCP_POS[2]}",
        size="0.01", rgba="0 1 0 1",
    )

    if apply_phase4e_gripper_visuals:
        # Phase 4E, Task 1 scene only: a small task-local palm backing
        # plate so the gripper reads visually as one mechanism (wrist ->
        # palm -> two fingers) instead of two pads floating in space where
        # the decorative mesh used to be. Positioned well clear of the
        # grasp region (front face at local x = PALM_POS_X + PALM_HALF[0]
        # = 0.042 m, vs. the cube's near face at roughly FINGER_REACH_X -
        # CUBE_HALF = 0.065 m during a grasp) so it never contacts the
        # cube; contype/conaffinity=1 so it is a real body if something is
        # authored differently later, but a 2.3 cm clearance margin means
        # it does not participate in any grasp in this phase.
        PALM_POS_X = 0.03
        PALM_HALF = (0.012, 0.05, 0.018)
        palm = _sub(wrist, "body", name="palm", pos=f"{PALM_POS_X} 0 0")
        _sub(
            palm, "geom", name="palm_geom", type="box",
            size=f"{PALM_HALF[0]} {PALM_HALF[1]} {PALM_HALF[2]}",
            material="palm_mat", contype="1", conaffinity="1", mass="0.02",
        )

    for side, y_ref, jrange in (
        ("left", FINGER_OPEN_Y, f"{-(FINGER_OPEN_Y - FINGER_CLOSED_Y):.4f} 0"),
        ("right", -FINGER_OPEN_Y, f"0 {(FINGER_OPEN_Y - FINGER_CLOSED_Y):.4f}"),
    ):
        finger_body_kwargs = {"pos": f"{FINGER_REACH_X} {y_ref} 0"}
        if apply_phase4f_pad_mount_fix:
            # Phase 4F Attempt 3: rotate the finger BODY's own local frame
            # only (pos above is unchanged, so the finger's origin still
            # brackets the TCP/cube target exactly as before) -- see
            # FINGER_MOUNT_FIX_QUAT's definition for the measured evidence
            # and derivation. The joint axis and geom below are defined in
            # this same local frame, so both rotate consistently with it.
            w, x, y, z = FINGER_MOUNT_FIX_QUAT
            finger_body_kwargs["quat"] = f"{w} {x} {y} {z}"
        finger = _sub(wrist, "body", name=f"{side}_finger", **finger_body_kwargs)
        _sub(
            finger, "joint", name=f"{side}_finger_joint", type="slide",
            axis="0 1 0", range=jrange, damping="2.0", frictionloss="0.05",
        )
        finger_material = f"finger_mat_{side}" if apply_phase4e_gripper_visuals else "finger_mat"
        _sub(
            finger, "geom", name=f"{side}_finger_pad", type="box",
            size=f"{finger_pad_half[0]} {finger_pad_half[1]} {finger_pad_half[2]}",
            material=finger_material, contype="1", conaffinity="1",
            friction=FINGER_FRICTION, mass="0.03",
        )

    # Exclude wrist-vs-finger and wrist-vs-vendor-hand-mesh contact pairs so
    # the rigid mounting frame does not register spurious self-contact; the
    # cube is the only intended contact partner for the finger pads.
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    _sub(contact, "exclude", name="wrist_left_finger_exclude", body1=WRIST_BODY, body2="left_finger")
    _sub(contact, "exclude", name="wrist_right_finger_exclude", body1=WRIST_BODY, body2="right_finger")
    if apply_phase4e_gripper_visuals:
        # Phase 4E, Task 1 scene only: the new palm sits close to the wrist
        # mount and to both fingers' resting position -- exclude its
        # self-contact pairs the same way the pre-existing wrist/finger
        # pairs are excluded above.
        _sub(contact, "exclude", name="wrist_palm_exclude", body1=WRIST_BODY, body2="palm")
        _sub(contact, "exclude", name="palm_left_finger_exclude", body1="palm", body2="left_finger")
        _sub(contact, "exclude", name="palm_right_finger_exclude", body1="palm", body2="right_finger")

    # --- actuators: motors for the two finger slide joints, bounded force ---
    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    _sub(
        actuator, "motor", name="left_finger", joint="left_finger_joint",
        ctrllimited="true", ctrlrange=f"{-FINGER_FORCE_LIMIT} {FINGER_FORCE_LIMIT}",
    )
    _sub(
        actuator, "motor", name="right_finger", joint="right_finger_joint",
        ctrllimited="true", ctrlrange=f"{-FINGER_FORCE_LIMIT} {FINGER_FORCE_LIMIT}",
    )

    return tree


def write_grasp_scene() -> Path:
    scene = TASK_DIR / "g1_grasp_scene.xml"
    tree = _build_grasp_tree()
    tree.write(scene, encoding="utf-8", xml_declaration=False)
    return scene


# Phase 4B: static blue target pad for Task 1 ("place it in the blue target
# area"). Chosen by direct IK-residual evidence over a grid of candidate
# offsets (see reports/phase4b-task1-pick-place.md, "Target selection") --
# TARGET_OFFSET_FROM_CUBE = (-0.11, +0.07) was the offset with the largest
# reachability margin (max residual 3.61 mm across the three carry
# waypoints, vs. IK_POS_TOL = 8 mm) among candidates that also (a) keep the
# cube's full footprint on the table with TABLE_EDGE_MARGIN clearance, and
# (b) require a lateral transport of at least 0.10 m (the cube's own
# footprint is 0.07 m, so smaller offsets would not constitute "meaningful
# lateral transport"). This direction (-x, +y relative to the cube) matches
# Phase 4A's own finding that -x/+y offsets are reachable while +x/-y are
# not, near the documented wrist singularity.
TARGET_OFFSET_FROM_CUBE = (-0.11, 0.07)
TARGET_POS = (CUBE_POS[0] + TARGET_OFFSET_FROM_CUBE[0], CUBE_POS[1] + TARGET_OFFSET_FROM_CUBE[1])
TARGET_HALF_XY = 0.05  # 10 cm x 10 cm pad -- comfortably larger than the cube's own 7 cm footprint
TARGET_HALF_Z = 0.0025  # 5 mm thick pad
TARGET_PAD_TOP_Z = TABLE_TOP_Z + 2 * TARGET_HALF_Z
# Cube resting on the pad (not held): center height = pad top + cube half-extent.
TARGET_RELEASE_Z = TARGET_PAD_TOP_Z + CUBE_HALF
# Containment margin for the success detector: requiring the cube's center to
# stay within (TARGET_HALF_XY - CUBE_HALF) of the pad center keeps the cube's
# *entire* footprint on the pad, not just its center point.
TARGET_XY_SUCCESS_MARGIN_M = TARGET_HALF_XY - CUBE_HALF


def _add_target_pad(tree: ET.ElementTree) -> None:
    """Adds a static blue target pad to the scene. A plain geom on a
    jointless body (implicitly fixed to the world in MuJoCo) -- no
    equality/weld/tendon/actuator/force of any kind references it or the
    cube; placement success is judged from cube state (position/velocity),
    never from this geom's color or any rendering.
    """
    root = tree.getroot()
    asset = root.find("asset")
    if asset is None:
        raise RuntimeError("expected asset section from _build_grasp_tree()")
    _sub(asset, "material", name="target_mat", rgba="0.1 0.35 0.9 1")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("expected worldbody from _build_grasp_tree()")
    target = _sub(
        worldbody, "body", name="target_pad",
        pos=f"{TARGET_POS[0]} {TARGET_POS[1]} {TABLE_TOP_Z + TARGET_HALF_Z}",
    )
    _sub(
        target, "geom", name="target_pad_geom", type="box",
        size=f"{TARGET_HALF_XY} {TARGET_HALF_XY} {TARGET_HALF_Z}",
        material="target_mat", contype="1", conaffinity="1",
    )


# --- Phase 3C: right-arm torque motors replaced with bounded position servos ---
# Real per-joint physical force limits from the vendor model (g1_29dof.xml),
# reused as each new <position> actuator's forcerange -- the servo can never
# exceed the joint's actual torque authority, same physical honesty as the
# Phase 3/3B torque motors.
RIGHT_ARM_JOINT_ACTUATOR_PAIRS = [
    ("right_shoulder_pitch_joint", "right_shoulder_pitch", 25.0),
    ("right_shoulder_roll_joint", "right_shoulder_roll", 25.0),
    ("right_shoulder_yaw_joint", "right_shoulder_yaw", 25.0),
    ("right_elbow_joint", "right_elbow", 25.0),
    ("right_wrist_roll_joint", "right_wrist_roll", 25.0),
    ("right_wrist_pitch_joint", "right_wrist_pitch", 5.0),
    ("right_wrist_yaw_joint", "right_wrist_yaw", 5.0),
]


def _apply_position_servo_arm(
    tree: ET.ElementTree,
    arm_kp: dict[str, float] | float,
    arm_kv: dict[str, float] | float,
) -> None:
    """Shared Phase 3C/4B step: replace the 7 right-arm <motor> actuators
    with bounded MuJoCo <position> servos, and switch the integrator to
    implicitfast. Factored out of write_grasp_scene_3c() unchanged (byte-for-
    byte identical resulting logic) so write_grasp_scene_4b() can reuse it
    without duplicating the tuning rationale below.

    Phase 3C finding: the vendor model has no <option> element, so MuJoCo
    defaults to explicit Euler integration. That is fine for Phase 3/3B's
    pure-force <motor> actuators, but explicit Euler is well known to be
    unstable for stiff <position> servo gains at this timestep (0.002 s) --
    observed directly: with Euler, most right-arm joints stayed permanently
    saturated in an oscillating limit cycle regardless of (kp, kv), even
    though the actual gravity/Coriolis torque needed at the target pose was
    small (<3.1 N*m, well inside every joint's force limit) -- i.e. it was a
    numerical integration problem, not a physical one. Switching to
    MuJoCo's `implicitfast` integrator (which integrates actuator
    damping/stiffness implicitly; MuJoCo's own documented recommendation for
    damped position/velocity actuators) resolved it without changing any
    force limit, gain, or physical parameter.
    """
    root = tree.getroot()
    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        root.insert(0, option)
    option.set("integrator", "implicitfast")

    actuator = root.find("actuator")
    if actuator is None:
        raise RuntimeError("actuator section missing after _build_grasp_tree()")

    for joint_name, actuator_name, force_limit in RIGHT_ARM_JOINT_ACTUATOR_PAIRS:
        old = None
        for el in actuator.findall("motor"):
            if el.get("name") == actuator_name:
                old = el
                break
        if old is None:
            raise RuntimeError(f"expected vendor motor actuator not found: {actuator_name}")
        actuator.remove(old)

        joint_el = None
        for body in root.iter("joint"):
            if body.get("name") == joint_name:
                joint_el = body
                break
        if joint_el is None:
            raise RuntimeError(f"joint not found: {joint_name}")
        jrange = joint_el.get("range")
        if jrange is None:
            raise RuntimeError(f"joint has no range, required for position ctrlrange: {joint_name}")

        kp = arm_kp[joint_name] if isinstance(arm_kp, dict) else arm_kp
        kv = arm_kv[joint_name] if isinstance(arm_kv, dict) else arm_kv
        if not (np.isfinite(kp) and np.isfinite(kv) and kp > 0 and kv >= 0):
            raise ValueError(f"non-finite/invalid gains for {joint_name}: kp={kp} kv={kv}")

        _sub(
            actuator, "position", name=actuator_name, joint=joint_name,
            kp=f"{kp:.6f}", kv=f"{kv:.6f}",
            ctrllimited="true", ctrlrange=jrange,
            forcelimited="true", forcerange=f"{-force_limit} {force_limit}",
        )


def write_grasp_scene_3c(
    arm_kp: dict[str, float] | float,
    arm_kv: dict[str, float] | float,
    scene_name: str = "g1_grasp_scene_3c.xml",
) -> Path:
    """Phase 3C variant: same environment/gripper/pelvis-weld/cube as
    write_grasp_scene(), but the 7 right-arm joints are driven by bounded
    MuJoCo <position> servos instead of Phase 3/3B's <motor> torque
    actuators. Actuator *names* are kept identical to the vendor model's
    (e.g. "right_elbow") so JointMap.build()'s name-based lookup works
    unchanged for either architecture.

    `arm_kp`/`arm_kv` may be a single float (uniform) or a dict keyed by
    joint name (per-joint) -- callers resolve gains at the call site so this
    function stays a pure scene generator, not a tuning policy.

    Also welds torso_link to pelvis (extra_trunk_weld=True) -- see the
    comment at that weld's construction in _build_grasp_tree() for why this
    was added specifically for Phase 3C.
    """
    scene = TASK_DIR / scene_name
    tree = _build_grasp_tree(extra_trunk_weld=True)
    _apply_position_servo_arm(tree, arm_kp, arm_kv)
    tree.write(scene, encoding="utf-8", xml_declaration=False)
    return scene


def write_grasp_scene_4b(
    arm_kp: dict[str, float] | float,
    arm_kv: dict[str, float] | float,
    scene_name: str = "g1_grasp_scene_4b.xml",
) -> Path:
    """Phase 4B variant: identical to write_grasp_scene_3c() (same pelvis+
    torso weld, same position-servo right arm, same implicitfast integrator,
    same physical parallel gripper, same cube) plus a static blue target
    pad (see _add_target_pad()) for Task 1's place location. No cube-
    referencing constraint of any kind is added.

    Phase 4E: this is also the only caller that builds with
    finger_pad_half=FINGER_PAD_HALF (taller pads) and
    apply_phase4e_gripper_visuals=True (decorative-mesh removal, palm,
    per-side finger coloring) -- see _build_grasp_tree()'s docstring.

    Deliberately UNCHANGED by Phase 4F: apply_phase4f_pad_mount_fix stays at
    its default (False) here, so every Phase 4B/4C/4D/4E test that already
    asserts specific numeric outcomes against this exact scene continues to
    do so unperturbed. Phase 4F's own evidence uses write_grasp_scene_4f()
    below instead of modifying this function in place -- the same
    "add a new versioned scene function, do not retrofit an old one"
    convention already used for write_grasp_scene -> write_grasp_scene_3c.
    """
    scene = TASK_DIR / scene_name
    tree = _build_grasp_tree(
        extra_trunk_weld=True, finger_pad_half=FINGER_PAD_HALF, apply_phase4e_gripper_visuals=True,
    )
    _add_target_pad(tree)
    _apply_position_servo_arm(tree, arm_kp, arm_kv)
    tree.write(scene, encoding="utf-8", xml_declaration=False)
    return scene


def write_grasp_scene_4f(
    arm_kp: dict[str, float] | float,
    arm_kv: dict[str, float] | float,
    scene_name: str = "g1_grasp_scene_4f.xml",
) -> Path:
    """Phase 4F variant: identical to write_grasp_scene_4b() except
    apply_phase4f_pad_mount_fix=True (the measured finger-pad mounting
    correction -- see FINGER_MOUNT_FIX_QUAT). Kept as its own function,
    writing its own scene file, so write_grasp_scene_4b()'s output and every
    test that depends on it (Phase 4B/4C/4D/4E) are byte-for-byte/physics-
    identical to before this phase.
    """
    scene = TASK_DIR / scene_name
    tree = _build_grasp_tree(
        extra_trunk_weld=True, finger_pad_half=FINGER_PAD_HALF, apply_phase4e_gripper_visuals=True,
        apply_phase4f_pad_mount_fix=True,
    )
    _add_target_pad(tree)
    _apply_position_servo_arm(tree, arm_kp, arm_kv)
    tree.write(scene, encoding="utf-8", xml_declaration=False)
    return scene


if __name__ == "__main__":
    path = write_grasp_scene()
    print(f"wrote {path}")
