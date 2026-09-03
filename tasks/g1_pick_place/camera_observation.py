#!/usr/bin/env python3
"""Phase 5A: task-local onboard RGB camera for a future VLA policy
observation.

Adds a single `<camera>` element, mounted rigidly on the G1's `torso_link`
body (the vendor model has no separate head/neck body or joint -- the head
mesh itself is just a static geom on `torso_link`; see the module docstring
of `_add_head_camera` below), to a copy of Phase 4B's scene
(`write_grasp_scene_4b()`), never to the vendor XML and never to
`write_grasp_scene_4b()`'s own file. This mirrors the isolation pattern
already used for Phase 4E/4F's own scene variants: `write_grasp_scene_5a()`
is a new, additive function; every existing scene generator and every
existing test that depends on one is untouched (verified in
`tests/test_phase5a_onboard_camera.py`).

The existing third-person evidence camera used throughout Phase 4C-4F
(`record_*_episode.py`) is a free-floating `mujoco.MjvCamera` constructed at
render time in Python -- it has no `<camera>` element in the model at all,
and is completely unaffected by this module. The onboard camera added here
is a genuinely different thing: a real MJCF `<camera>` element rigidly
attached to a moving body, whose world pose is therefore a function of the
robot's own kinematics (`data.cam_xpos`/`data.cam_xmat`), not a
user-controlled orbit camera.

Camera pose derivation (documented, not guessed): the vendor `head_link`
mesh's own local-frame bounding box (computed directly from the STL,
`vendor/unitree_mujoco/unitree_robots/g1/meshes/head_link.STL`) is
x in [-0.066, 0.074], y in [-0.078, 0.078], z in [0.325, 0.530]. Composed
with the vendor MJCF's own geom offset for that mesh (`pos="0.0039635 0
-0.054"` within `torso_link`) and `torso_link`'s own world pose at reset
under this project's fixed pelvis+torso weld (`(-0.0039635, 0, 0.847)`,
identity orientation), the head mesh occupies world z in [1.118, 1.323] --
i.e. eye height for this seated/fixed-base configuration is roughly
z=1.19-1.22m. Placing a camera literally inside that mesh volume (early
iteration, `CAM_POS_WORLD_ITER1 = (0.02, 0, 1.20)`) produced heavy visual
self-occlusion by the robot's own head geometry; the final pose
(`CAM_POS_WORLD`) sits forward of the head mesh's own local-x extent
(x=0.11 > the mesh's own x_max=0.074) so the lens looks out from in front
of the face, not from inside the head volume. This iteration is recorded
in `reports/phase5a-onboard-camera.md`, Section C.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place.gripper_scene import CUBE_POS, TARGET_POS, TABLE_TOP_Z, TASK_DIR, write_grasp_scene_4b

HEAD_CAM_NAME = "head_cam"
HEAD_CAM_PARENT_BODY = "torso_link"

# torso_link's own world pose at reset, under this project's fixed pelvis+
# torso weld (identity orientation -- confirmed by direct query, see
# reports/phase5a-onboard-camera.md Section B). A `<camera>` child's `pos`
# is expressed in the PARENT body's local frame, not world -- this constant
# is what let the pose below be *designed* in world coordinates and then
# converted, rather than guessed directly in the local frame.
TORSO_WORLD_POS_AT_RESET = np.array([-0.0039635, 0.0, 0.847])

# Final camera pose, designed in world coordinates (see module docstring
# for the derivation and the self-occlusion iteration that motivated it).
CAM_POS_WORLD = np.array([0.11, 0.0, 1.19])
_LOOK_AT_TARGET_WORLD = np.array([0.29, -0.13, 0.72])  # midpoint of cube/target workspace, table height

CAM_RESOLUTION = (160, 120)  # (width, height) -- conservative per HANDOFF.md Phase 5A spec
CAM_FOVY_DEG = 90.0
CAM_WIDTH, CAM_HEIGHT = CAM_RESOLUTION


def _look_at_quat(cam_pos: np.ndarray, target: np.ndarray, up_ref: np.ndarray = np.array([0.0, 0.0, 1.0])) -> np.ndarray:
    """Returns a MuJoCo (w, x, y, z) quaternion for a camera at `cam_pos`
    looking at `target`, given a world-frame reference "up" direction.

    MuJoCo camera convention: the camera looks down its own local -Z axis,
    with local +Y as "up" in the rendered image and local +X as "right".
    So the camera's local axes, expressed as columns of a world-frame
    rotation matrix, are [right, true_up, -forward] where `forward` points
    from the camera to the target.
    """
    forward = np.asarray(target) - np.asarray(cam_pos)
    forward = forward / np.linalg.norm(forward)
    if abs(np.dot(forward, up_ref)) > 0.99:
        up_ref = np.array([0.0, 1.0, 0.0])
    right = np.cross(forward, up_ref)
    right = right / np.linalg.norm(right)
    true_up = np.cross(right, forward)
    rot = np.column_stack([right, true_up, -forward])
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, rot.flatten())
    return quat


CAM_QUAT = _look_at_quat(CAM_POS_WORLD, _LOOK_AT_TARGET_WORLD)
CAM_POS_LOCAL = CAM_POS_WORLD - TORSO_WORLD_POS_AT_RESET  # torso_link has identity quat at reset, so this subtraction is valid


def _add_head_camera(tree: ET.ElementTree) -> None:
    """Adds the onboard RGB camera as a child of `torso_link`. Read-only
    with respect to `MjData` at simulation time (a `<camera>` element only
    ever affects rendering, never physics) -- confirmed empirically in
    `tests/test_phase5a_onboard_camera.py` by comparing a full trial's
    physics trajectory with and without the camera/rendering calls active.
    """
    root = tree.getroot()
    torso = None
    for body in root.iter("body"):
        if body.get("name") == HEAD_CAM_PARENT_BODY:
            torso = body
            break
    if torso is None:
        raise RuntimeError(f"expected body '{HEAD_CAM_PARENT_BODY}' from _build_grasp_tree()")
    ET.SubElement(
        torso, "camera",
        name=HEAD_CAM_NAME,
        pos=f"{CAM_POS_LOCAL[0]} {CAM_POS_LOCAL[1]} {CAM_POS_LOCAL[2]}",
        quat=f"{CAM_QUAT[0]} {CAM_QUAT[1]} {CAM_QUAT[2]} {CAM_QUAT[3]}",
        fovy=str(CAM_FOVY_DEG),
    )


def write_grasp_scene_5a(arm_kp, arm_kv, scene_name: str = "g1_grasp_scene_5a.xml") -> Path:
    """Phase 5A variant: identical to `write_grasp_scene_4b()`'s own output
    plus the onboard head camera. Built by re-parsing
    `write_grasp_scene_4b()`'s written file (not by re-deriving the tree),
    so any future change to that function's own scene is automatically
    picked up here without duplicating its logic -- and, critically, so
    `write_grasp_scene_4b()` itself is never called with different
    arguments or modified in any way; every existing Phase 4B/4C/4D/4E test
    that depends on its exact output is unaffected (confirmed: zero diff on
    that scene file across this phase, see the report).
    """
    base = write_grasp_scene_4b(arm_kp=arm_kp, arm_kv=arm_kv, scene_name="g1_grasp_scene_4b.xml")
    tree = ET.parse(base)
    _add_head_camera(tree)
    out = TASK_DIR / scene_name
    tree.write(out, encoding="utf-8", xml_declaration=False)
    return out


def camera_intrinsics(width: int = CAM_WIDTH, height: int = CAM_HEIGHT, fovy_deg: float = CAM_FOVY_DEG) -> dict:
    """Pinhole intrinsic parameters for MuJoCo's default camera model
    (square pixels, principal point at image center, vertical FOV given by
    `fovy`). fx = fy = height / (2 tan(fovy/2)); horizontal FOV follows
    from the aspect ratio at that same focal length, not from a separately
    specified fovx.
    """
    fovy_rad = np.radians(fovy_deg)
    fy = height / (2.0 * np.tan(fovy_rad / 2.0))
    fx = fy
    cx, cy = width / 2.0, height / 2.0
    fovx_deg = float(np.degrees(2.0 * np.arctan(width / (2.0 * fy))))
    return {
        "width": width, "height": height, "fx": float(fx), "fy": float(fy),
        "cx": float(cx), "cy": float(cy), "fovy_deg": float(fovy_deg), "fovx_deg": fovx_deg,
        "matrix": [[float(fx), 0.0, float(cx)], [0.0, float(fy), float(cy)], [0.0, 0.0, 1.0]],
    }


def camera_extrinsic(model: mujoco.MjModel, data: mujoco.MjData, cam_id: int) -> dict:
    """World-frame extrinsic (position + rotation matrix) of the camera at
    the current `data` state -- read from `data.cam_xpos`/`data.cam_xmat`,
    not the static MJCF pos/quat, since the camera sits on a moving body
    (rigid in this fixed-base configuration at reset, but the *mechanism*
    generalizes to a future non-fixed-base torso).
    """
    pos = data.cam_xpos[cam_id].copy()
    rot = data.cam_xmat[cam_id].reshape(3, 3).copy()
    return {"position_world": pos.tolist(), "rotation_world": rot.tolist()}


def red_cube_mask(frame: np.ndarray) -> np.ndarray:
    """Smoke-test diagnostic ONLY -- not task-success logic (see HANDOFF.md
    Phase 5A Section D). Simple RGB thresholding calibrated against a
    sampled onboard frame (see reports/phase5a-onboard-camera.md): the
    rendered cube is a saturated red/orange under this scene's lighting,
    clearly separated from the tan table and blue sky/target by a large
    R-G and R-B gap.
    """
    r = frame[..., 0].astype(int)
    g = frame[..., 1].astype(int)
    b = frame[..., 2].astype(int)
    return (r > 140) & (r - g > 50) & (r - b > 50)


def blue_target_mask(frame: np.ndarray) -> np.ndarray:
    """Smoke-test diagnostic ONLY -- not task-success logic. Tighter than a
    naive "is it blue" threshold: MuJoCo's default skybox is itself a
    medium blue-gray (sampled ~(90,135,182) in this scene), so the mask
    additionally requires a low red channel and a large B-R gap to isolate
    the saturated target-pad blue (rgba 0.1 0.35 0.9) from the sky.
    """
    r = frame[..., 0].astype(int)
    b = frame[..., 2].astype(int)
    return (b > 150) & (b - r > 110) & (r < 60)
