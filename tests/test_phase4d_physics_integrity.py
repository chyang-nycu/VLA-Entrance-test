#!/usr/bin/env python3
"""Phase 4D: physics-integrity investigation tests.

Context: a user visual inspection of the committed Task 1 pipeline (commit
dfeec9e) reported two apparent defects: (1) the hand/fingers visibly pass
through the cube, (2) the cube visibly falls instead of being lifted. This
module is a skeptical, from-scratch re-check of both claims against the
real, unmodified simulation -- not a defense of the existing (already
passing) test suite.

Finding, in one line: defect (1) is CONFIRMED and reproduced here (see
Phase4DDecorativeHandOverlapTest -- this test class is EXPECTED TO FAIL
against current code, honestly, since the defect has not been fixed in
this phase). Defect (2) is NOT reproduced by direct instrumentation (see
Phase4DTableSupportTest and Phase4DLiftIsGenuineTest, both of which pass,
because table-support and lift physics are correct). Full writeup:
reports/phase4d-physics-integrity-audit.md.

Per HANDOFF.md Phase 4D scope: no fix is implemented or committed in this
phase. Do not "fix" Phase4DDecorativeHandOverlapTest to make it pass by
loosening its tolerance -- that would misrepresent an unfixed defect as
resolved. It exists to keep the defect visible in the default test run
until a real fix (suppressing/hiding the vendor's decorative mesh, or
removing it from the task-local scene) is authorized and implemented in a
later phase.
"""

from __future__ import annotations

import struct
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from tasks.g1_pick_place import gripper_scene
from tasks.g1_pick_place.gripper_scene import CUBE_HALF, CUBE_MASS, write_grasp_scene_4b
from tasks.g1_pick_place.run_pick_place import ARM_KP_4B, ARM_KV_4B, run_trial_pick_place

ROOT = Path(__file__).resolve().parents[1]
VENDOR_MESH_STL = (
    ROOT / "vendor" / "unitree_mujoco" / "unitree_robots" / "g1" / "meshes" / "right_rubber_hand.STL"
)
RUBBER_HAND_LOCAL_OFFSET = np.array([0.0415, -0.003, 0.0])
FINGER_REACH_X = 0.10
FINGER_PAD_HALF_X = 0.012


def _stl_bbox(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with open(path, "rb") as f:
        f.read(80)
        count = struct.unpack("<I", f.read(4))[0]
        verts = np.zeros((count * 3, 3), dtype=np.float64)
        idx = 0
        for _ in range(count):
            data = f.read(50)
            vals = struct.unpack("<12f", data[:48])
            verts[idx] = vals[3:6]
            idx += 1
            verts[idx] = vals[6:9]
            idx += 1
            verts[idx] = vals[9:12]
            idx += 1
    return verts.min(axis=0), verts.max(axis=0)


class Phase4DCubeIdentityTest(unittest.TestCase):
    """Section B: the tracked body/geom must be the true rendered dynamic
    cube -- not a probe object, the target pad, the TCP site, or a finger
    body. This class is expected to PASS: cube identity tracking itself is
    not the defect."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
        cls.model = mujoco.MjModel.from_xml_path(str(cls.scene))

    def test_cube_joint_is_a_free_joint(self) -> None:
        jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
        self.assertEqual(int(self.model.jnt_type[jid]), int(mujoco.mjtJoint.mjJNT_FREE))

    def test_tracked_geom_is_not_target_pad_or_finger_pad(self) -> None:
        cube_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
        target_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "target_pad_geom")
        left_pad_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
        right_pad_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")
        self.assertNotEqual(cube_geom_id, target_id)
        self.assertNotEqual(cube_geom_id, left_pad_id)
        self.assertNotEqual(cube_geom_id, right_pad_id)

    def test_geom_xpos_matches_body_xpos_every_step(self) -> None:
        """cube_geom has zero local offset in the cube body frame, so its
        rendered world position must always exactly equal the body's -- a
        fresh, real-simulation check, not an assumption."""
        data = mujoco.MjData(self.model)
        cube_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        cube_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
        mujoco.mj_resetData(self.model, data)
        mujoco.mj_forward(self.model, data)
        for _ in range(50):
            mujoco.mj_step(self.model, data)
            self.assertTrue(
                np.allclose(data.xpos[cube_body_id], data.geom_xpos[cube_geom_id], atol=1e-12),
                "rendered cube geom position diverged from the tracked body position",
            )


class Phase4DTableSupportTest(unittest.TestCase):
    """Section C: isolated cube/table settling test, real scene, no robot
    motion. Expected to PASS -- table-support physics is not the defect."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
        cls.model = mujoco.MjModel.from_xml_path(str(cls.scene))
        cls.data = mujoco.MjData(cls.model)
        mujoco.mj_resetData(cls.model, cls.data)
        mujoco.mj_forward(cls.model, cls.data)
        cls.cube_body_id = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        cls.cube_geom_id = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
        cls.table_geom_id = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")
        cube_joint_id = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_JOINT, "cube_joint")
        cls.dof_adr = int(cls.model.jnt_dofadr[cube_joint_id])
        cls.table_top_world_z = float(cls.data.geom_xpos[cls.table_geom_id][2]) + float(
            cls.model.geom_size[cls.table_geom_id][2]
        )
        cls.z_trace = []
        cls.contact_forces = []  # per-contact-point samples (a box-on-box
        # rest typically reports multiple simultaneous contact points, e.g.
        # near the 4 corners, each carrying only a fraction of the total
        # weight -- see per_step_total_forces below for the physically
        # meaningful "does this balance the cube's weight" check).
        cls.per_step_total_forces = []
        for _ in range(int(round(3.0 / cls.model.opt.timestep))):
            mujoco.mj_step(cls.model, cls.data)
            cls.z_trace.append(float(cls.data.xpos[cls.cube_body_id][2]))
            step_total = 0.0
            for c in range(cls.data.ncon):
                con = cls.data.contact[c]
                pair = (int(con.geom1), int(con.geom2))
                if cls.cube_geom_id in pair and cls.table_geom_id in pair:
                    f = np.zeros(6)
                    mujoco.mj_contactForce(cls.model, cls.data, c, f)
                    force_mag = float(np.linalg.norm(f[:3]))
                    cls.contact_forces.append(force_mag)
                    step_total += force_mag
            if step_total > 0:
                cls.per_step_total_forces.append(step_total)

    def test_cube_never_falls_through_table(self) -> None:
        self.assertGreater(min(self.z_trace) - CUBE_HALF, self.table_top_world_z - 0.005)

    def test_cube_settles_within_tight_tolerance_of_table_top(self) -> None:
        final_bottom = self.z_trace[-1] - CUBE_HALF
        self.assertAlmostEqual(final_bottom, self.table_top_world_z, delta=0.001)

    def test_cube_vertical_velocity_settles_near_zero(self) -> None:
        data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, data)
        mujoco.mj_forward(self.model, data)
        for _ in range(int(round(3.0 / self.model.opt.timestep))):
            mujoco.mj_step(self.model, data)
        vz = float(data.qvel[self.dof_adr + 2])
        self.assertLess(abs(vz), 1e-6)

    def test_cube_table_contact_has_positive_normal_force(self) -> None:
        self.assertGreater(len(self.contact_forces), 0)
        self.assertGreater(min(self.contact_forces), 0.0)

    def test_summed_contact_force_balances_cube_weight_at_rest(self) -> None:
        # A box resting flat typically reports several simultaneous contact
        # points (near its corners); each carries only a fraction of the
        # weight, but their per-step sum must balance gravity once settled.
        self.assertGreater(len(self.per_step_total_forces), 0)
        settled_totals = self.per_step_total_forces[-200:]
        self.assertAlmostEqual(float(np.mean(settled_totals)), CUBE_MASS * 9.81, delta=0.05)


class Phase4DLiftIsGenuineTest(unittest.TestCase):
    """Section D/G gate 6-7: re-verify, from a fresh instrumented rerun (not
    a cached log), that the cube genuinely rises and is genuinely held by
    real nonzero contact force -- not merely a passed boolean. Expected to
    PASS."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.scene = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
        cls.model = mujoco.MjModel.from_xml_path(str(cls.scene))
        cube_geom_id = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")
        left_pad_id = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_GEOM, "left_finger_pad")
        right_pad_id = mujoco.mj_name2id(cls.model, mujoco.mjtObj.mjOBJ_GEOM, "right_finger_pad")
        cls.hold_forces = []

        def cb(phase, m, d):
            if phase != "HOLD":
                return
            for i in range(d.ncon):
                c = d.contact[i]
                pair = (int(c.geom1), int(c.geom2))
                if cube_geom_id in pair and (left_pad_id in pair or right_pad_id in pair):
                    f = np.zeros(6)
                    mujoco.mj_contactForce(m, d, i, f)
                    cls.hold_forces.append(float(np.linalg.norm(f[:3])))

        cls.result = run_trial_pick_place(cls.scene, frame_callback=cb)

    def test_task_passes_on_fresh_rerun(self) -> None:
        self.assertTrue(self.result["task_pass"])

    def test_height_gain_at_least_8cm_fresh_rerun(self) -> None:
        self.assertGreaterEqual(self.result["height_gain_m"], 0.08)

    def test_hold_phase_has_nonzero_bilateral_contact_force_not_merely_a_boolean(self) -> None:
        self.assertGreater(len(self.hold_forces), 0)
        self.assertGreater(min(self.hold_forces), 0.0)


class Phase4DDecorativeHandOverlapTest(unittest.TestCase):
    """Root-cause confirmation for the reported 'hand/fingers pass through
    the cube' defect.

    Originally (Phase 4D, commit e212777) this class's headline test
    asserted a STATIC geometric fact -- that the vendor mesh's own STL
    bounding box, at the vendor's own fixed local offset, overlaps
    FINGER_REACH_X's local-x range -- and was deliberately left FAILING to
    document an unfixed defect. That static fact is a permanent property of
    the vendor STL and offset; it stays true forever regardless of what
    this project's own scene generator does with it, so it could never
    become a meaningful "is it fixed now" signal by itself.

    Phase 4E (reports/phase4e-gripper-integrity-repair.md) fixed the actual
    defect by removing the right_rubber_hand geom from the task-local
    generated scene entirely (tasks/g1_pick_place/gripper_scene.py,
    _build_grasp_tree()) -- so the real, rendered scene no longer contains
    a decorative geom that could clip through anything, irrespective of the
    vendor STL's own static geometry. This test is updated, per Phase 4E's
    explicit instruction, to check that real condition directly against
    the actual generated scene -- a stronger, more direct verification of
    the fix than the original static computation, not a weaker one. The
    static overlap computation is retained below as a documented,
    permanently-true fact about the vendor asset (useful context, not a
    pass/fail condition).
    """

    def test_task_local_scene_omits_the_decorative_hand_geom(self) -> None:
        """Genuine post-fix check: the actual generated Task 1 scene
        (write_grasp_scene_4b's output, the same generator every Phase 4B/
        4C/4D test in this file already uses) must not contain a
        right_rubber_hand geom under right_wrist_yaw_link at all -- not
        merely "collision-free", genuinely absent from what gets rendered.
        """
        scene_path = write_grasp_scene_4b(arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_4b.xml")
        tree = ET.parse(scene_path)
        found = False
        for body in tree.getroot().iter("body"):
            if body.get("name") == "right_wrist_yaw_link":
                for g in body.iter("geom"):
                    if g.get("mesh") == "right_rubber_hand":
                        found = True
        self.assertFalse(
            found,
            "right_rubber_hand geom still present in the generated task-local scene -- "
            "the Phase 4E fix (removing it in gripper_scene._build_grasp_tree) is not in effect.",
        )

    def test_vendor_stl_bbox_still_overlaps_finger_range_context_only(self) -> None:
        """Documents, for context, that the vendor mesh's static geometry
        would still overlap the finger range if it were ever re-added --
        this is why removing the geom (not merely disabling its render
        group) was the correct fix, and is not itself a pass/fail signal
        about whether the defect is currently fixed (see class docstring).
        """
        rubber_min, rubber_max = _stl_bbox(VENDOR_MESH_STL)
        rubber_x_lo = RUBBER_HAND_LOCAL_OFFSET[0] + rubber_min[0]
        rubber_x_hi = RUBBER_HAND_LOCAL_OFFSET[0] + rubber_max[0]
        finger_x_lo = FINGER_REACH_X - FINGER_PAD_HALF_X
        finger_x_hi = FINGER_REACH_X + FINGER_PAD_HALF_X
        overlaps = rubber_x_lo <= finger_x_hi and rubber_x_hi >= finger_x_lo
        self.assertTrue(overlaps, "expected static vendor-STL overlap fact to still hold (context check)")

    def test_vendor_decorative_hand_mesh_geom_is_collision_free_by_vendor_authoring(self) -> None:
        """Confirms the mesh is collision-free by the VENDOR's own MJCF
        (not something this project's scene generator disabled) -- this is
        a factual/diagnostic check, expected to pass, establishing that the
        overlap above produces a purely visual defect (no interpenetration
        force), not a physics/contact-solver defect."""
        tree = ET.parse(ROOT / "vendor" / "unitree_mujoco" / "unitree_robots" / "g1" / "g1_29dof.xml")
        found = False
        for body in tree.getroot().iter("body"):
            if body.get("name") == "right_wrist_yaw_link":
                for g in body.findall("geom"):
                    if g.get("mesh") == "right_rubber_hand":
                        found = True
                        self.assertEqual(g.get("contype"), "0")
                        self.assertEqual(g.get("conaffinity"), "0")
        self.assertTrue(found, "right_rubber_hand geom not found under right_wrist_yaw_link in vendor model")


if __name__ == "__main__":
    unittest.main()
