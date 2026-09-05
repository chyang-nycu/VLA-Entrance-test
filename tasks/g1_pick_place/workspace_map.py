"""Phase 7A: TCP workspace and conditioning map (physics-free).

Phase 4A recorded that "a full Jacobian-conditioning map over the workspace
was not computed in this phase -- out of scope". This module computes it.

Motivation: the reachable envelope quoted throughout this project
(`cube_dx in [-0.035,-0.005]`, `cube_dy in [-0.01,0.035]`) comes from
`logs/phase5e_pilot_ik_grid.json`, an 81-point grid over *cube positions*
evaluated through all 7 pick-place waypoints. That grid stops at
`cube_dx=-0.035` because that was the sampling range of interest, not
because anything failed there -- every point in that column is reachable.
So the -x extent of the workspace (toward the robot) has never been
measured, while the +x limit is a genuine kinematic wall (the whole
`cube_dx=+0.005` row fails, consistent with Phase 4A's 27.1mm rejection).

Any task that moves the TCP along an extended path -- such as following a
door hinge's arc -- needs that map before its geometry can be chosen. This
sweep therefore measures single-waypoint TCP reachability directly, rather
than inferring it from a whole pick-place chain.

Method notes:
- Physics-free. `mujoco.mj_step` is never called; only `mj_kinematics` /
  `mj_comPos` / `mj_jacSite` on a scratch MjData, exactly as
  `solve_ik_waypoint` already does internally.
- Every grid point is solved **cold**, warm-started from the same
  `mj_resetData` base pose rather than from its neighbour's solution. That
  makes the map order-independent and bit-reproducible, at the cost of not
  reflecting the path-dependence a real trajectory would see.
  `diagnose_pick_place_reachability` deliberately does the opposite
  (chained warm starts), because it models a continuous motion.
- Uses the shipped `solve_ik_waypoint` and its shipped tolerance
  (`IK_POS_TOL`), not a re-implementation, so "reachable" here means
  exactly what it means everywhere else in this project.
"""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from .camera_observation import write_grasp_scene_5a
from .controller import (
    RIGHT_ARM_ACTUATORS,
    RIGHT_ARM_JOINTS,
    TCP_SITE,
    JointMap,
)
from .controller_3c import (
    IK_POS_TOL,
    ORIENT_TOL_RAD,
    orientation_residual_rad,
    solve_ik_waypoint,
)
from .run_pick_place import ARM_KP_4B, ARM_KV_4B

REPO_ROOT = Path(__file__).resolve().parents[2]
LOG_PATH = REPO_ROOT / "logs" / "phase7a_tcp_workspace_map.json"

# Grid bounds. x is swept from just inside the table's near edge (0.11) out
# past the known +x wall (~0.335) so BOTH boundaries are located by
# measurement rather than assumed. y spans the table's usable width; z
# covers the table top (0.70) up to comfortable lift height.
GRID_X = np.round(np.arange(0.100, 0.4251, 0.025), 4)
GRID_Y = np.round(np.arange(-0.300, 0.0751, 0.025), 4)
GRID_Z = np.round(np.arange(0.750, 0.9501, 0.050), 4)


def _build_env():
    """Same construction as replay_demonstration_v3._build_env, minus the
    cube bookkeeping this sweep does not need."""
    scene_path = write_grasp_scene_5a(
        arm_kp=ARM_KP_4B, arm_kv=ARM_KV_4B, scene_name="g1_grasp_scene_5a.xml"
    )
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    arm_map = JointMap.build(model, RIGHT_ARM_JOINTS, RIGHT_ARM_ACTUATORS)
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE)
    mujoco.mj_resetData(model, data)
    return model, data, arm_map, site_id


def evaluate_point(
    model, scratch, base_qpos, arm_map: JointMap, site_id: int, target_pos
) -> dict:
    """Solve IK for one TCP target and characterise the solved configuration.

    Returns residual/reachability plus the three conditioning signals the
    door-geometry derivation needs: Yoshikawa manipulability, Jacobian
    condition number (which is what actually locates the wrist singularity
    Phase 3C and 4A both ran into), and the natural wrist orientation
    residual at that pose.
    """
    target = np.asarray(target_pos, dtype=float)
    nominal_q = np.zeros(len(arm_map.names))
    q, resid, iters = solve_ik_waypoint(
        model, scratch, base_qpos, arm_map, site_id, target, nominal_q
    )

    # Re-establish the solved configuration on the scratch data so the
    # Jacobian and site frame below describe that pose, not the last IK
    # iterate's.
    scratch.qpos[:] = base_qpos
    arm_map.set_qpos(scratch, q)
    mujoco.mj_kinematics(model, scratch)
    mujoco.mj_comPos(model, scratch)

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, scratch, jacp, jacr, site_id)
    jac = jacp[:, arm_map.dof_adr]  # 3 x 7 position Jacobian for the arm

    gram = jac @ jac.T
    det = float(np.linalg.det(gram))
    manipulability = float(np.sqrt(max(det, 0.0)))
    svals = np.linalg.svd(jac, compute_uv=False)
    smin = float(svals.min())
    cond = float(svals.max() / smin) if smin > 1e-12 else float("inf")

    orient_resid = float(orientation_residual_rad(scratch.site_xmat[site_id]))

    lo = arm_map.jnt_range[:, 0]
    hi = arm_map.jnt_range[:, 1]
    span = np.maximum(hi - lo, 1e-9)
    margin = float(np.min(np.minimum(q - lo, hi - q) / span))

    return {
        "target_pos": target.tolist(),
        "ik_residual_m": float(resid),
        "iterations": int(iters),
        "reachable": bool(resid < IK_POS_TOL),
        "manipulability": manipulability,
        "condition_number": cond,
        "min_singular_value": smin,
        "orientation_residual_rad": orient_resid,
        "orientation_residual_deg": float(np.degrees(orient_resid)),
        "joint_limit_margin_frac": margin,
    }


def sweep() -> dict:
    """Sweep the full grid. Deterministic: no RNG, cold start per point."""
    model, data, arm_map, site_id = _build_env()
    scratch = mujoco.MjData(model)
    base_qpos = data.qpos.copy()

    points = []
    for z in GRID_Z:
        for x in GRID_X:
            for y in GRID_Y:
                rec = evaluate_point(
                    model, scratch, base_qpos, arm_map, site_id, (x, y, z)
                )
                rec["x"], rec["y"], rec["z"] = float(x), float(y), float(z)
                points.append(rec)

    reachable = [p for p in points if p["reachable"]]
    result = {
        "phase": "7A",
        "description": "TCP workspace and conditioning map (physics-free, cold-started per point)",
        "ik_pos_tol_m": float(IK_POS_TOL),
        "grid": {
            "x": GRID_X.tolist(),
            "y": GRID_Y.tolist(),
            "z": GRID_Z.tolist(),
            "n_points": len(points),
        },
        "summary": {
            "n_reachable": len(reachable),
            "n_total": len(points),
            "reachable_fraction": len(reachable) / len(points) if points else 0.0,
        },
        "points": points,
    }
    return result


# --- Phase 7A geometry derivation -------------------------------------------
#
# The door's geometry is an OUTPUT of the map, not an input. Pre-registered
# before the sweep ran (see the plan): a hinged door is adopted only if an arc
# of at least ARC_MIN_THETA_DEG at radius at least ARC_MIN_RADIUS_M fits
# entirely inside the well-conditioned region; otherwise the task falls back to
# a drawer (a straight segment) and the substitution is disclosed.

ARC_MIN_THETA_DEG = 30.0
ARC_MIN_RADIUS_M = 0.06

# Admissibility is defined against thresholds this project already uses, not
# against numbers invented for this task:
#
#   * position residual  -> IK_POS_TOL           (8mm, shipped, imported)
#   * orientation        -> ORIENT_TOL_RAD       (7 deg, shipped, imported)
#   * conditioning floor -> the smallest singular value of the position
#     Jacobian at `x_minus_0.03`, the best-conditioned cube position at which
#     Task 1's grasp is *already physically verified*. The door must be at
#     least as well conditioned as somewhere the arm demonstrably works.
#
# The sweep's own headline finding is that the height matters more than the
# footprint: at the table height Task 1 grasps at (z=0.735-0.75), NO point
# anywhere in the workspace meets ORIENT_TOL_RAD, whereas at z=0.90 a large
# region does. Phase 4F's orientation failure is therefore a property of
# reaching at table height, not of this arm.
ARC_SIGMA_MIN_FLOOR = 0.0527
ARC_HANDLE_Z = 0.90

TABLE_X_RANGE = (0.11, 0.55)
TABLE_Y_RANGE = (-0.37, 0.07)


def point_metrics(model, scratch, base_qpos, arm_map, site_id, x, y, z) -> dict:
    """Thin wrapper over evaluate_point for arc queries."""
    return evaluate_point(model, scratch, base_qpos, arm_map, site_id, (x, y, z))


def handle_pose(pivot_xy, radius, phi_deg, z=ARC_HANDLE_Z) -> np.ndarray:
    """Handle position at hinge angle phi. Pure; the door's geometry lives
    entirely in this one expression."""
    a = np.radians(phi_deg)
    return np.array([pivot_xy[0] + radius * np.cos(a),
                     pivot_xy[1] + radius * np.sin(a), z])


def arc_is_admissible(model, scratch, base_qpos, arm_map, site_id,
                      pivot_xy, radius, phi0_deg, theta_deg, step_deg=5.0) -> tuple[bool, dict]:
    """True when every sampled handle pose on the arc is reachable, oriented
    within the shipped tolerance, well conditioned, and over the table."""
    worst = {"min_singular_value": float("inf"), "orientation_residual_deg": 0.0,
             "ik_residual_m": 0.0, "manipulability": float("inf")}
    n = int(round(theta_deg / step_deg))
    for i in range(n + 1):
        h = handle_pose(pivot_xy, radius, phi0_deg + i * step_deg)
        if not (TABLE_X_RANGE[0] <= h[0] <= TABLE_X_RANGE[1]
                and TABLE_Y_RANGE[0] <= h[1] <= TABLE_Y_RANGE[1]):
            return False, worst
        m = evaluate_point(model, scratch, base_qpos, arm_map, site_id, h)
        if (not m["reachable"]
                or m["min_singular_value"] < ARC_SIGMA_MIN_FLOOR
                or m["orientation_residual_rad"] > ORIENT_TOL_RAD):
            return False, worst
        worst["min_singular_value"] = min(worst["min_singular_value"], m["min_singular_value"])
        worst["manipulability"] = min(worst["manipulability"], m["manipulability"])
        worst["orientation_residual_deg"] = max(
            worst["orientation_residual_deg"], m["orientation_residual_deg"])
        worst["ik_residual_m"] = max(worst["ik_residual_m"], m["ik_residual_m"])
    return True, worst


def search_largest_arc(model, scratch, base_qpos, arm_map, site_id) -> list[dict]:
    """Brute-force the largest admissible hinge arc. Deterministic, no RNG.

    Returns candidates sorted by chord length (longest first). The door's
    pivot / radius / swing are read off the top entry -- they are an OUTPUT
    of the measurement, never chosen first and validated afterwards.
    """
    out = []
    for px in np.arange(0.14, 0.4201, 0.02):
        for py in np.arange(-0.30, 0.0201, 0.02):
            for radius in (0.06, 0.08, 0.10, 0.12, 0.14):
                for phi0 in np.arange(0, 360, 30):
                    theta = 0.0
                    worst = {}
                    for cand in np.arange(5.0, 95.0, 5.0):
                        ok, w = arc_is_admissible(
                            model, scratch, base_qpos, arm_map, site_id,
                            (px, py), radius, float(phi0), float(cand))
                        if not ok:
                            break
                        theta, worst = float(cand), w
                    if theta >= ARC_MIN_THETA_DEG and radius >= ARC_MIN_RADIUS_M:
                        out.append({
                            "pivot_xy": [round(float(px), 4), round(float(py), 4)],
                            "radius_m": radius,
                            "phi0_deg": float(phi0),
                            "theta_deg": theta,
                            "chord_m": float(2 * radius * np.sin(np.radians(theta) / 2)),
                            "worst": worst,
                        })
    out.sort(key=lambda c: (-c["chord_m"], -c["theta_deg"]))
    return out


GEOMETRY_PATH = REPO_ROOT / "logs" / "phase7a_derived_door_geometry.json"


def main() -> int:
    result = sweep()
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    LOG_PATH.write_text(json.dumps(result, indent=2, sort_keys=True))
    s = result["summary"]
    print(f"points: {s['n_total']}  reachable: {s['n_reachable']} ({s['reachable_fraction']:.1%})")
    print(f"written: {LOG_PATH}")

    model, data, arm_map, site_id = _build_env()
    scratch = mujoco.MjData(model)
    candidates = search_largest_arc(model, scratch, data.qpos.copy(), arm_map, site_id)
    gate = "GO_HINGE" if candidates else "NO_GO_FALL_BACK_TO_DRAWER"
    derived = {
        "phase": "7A",
        "gate_outcome": gate,
        "criteria": {
            "ik_pos_tol_m": float(IK_POS_TOL),
            "orient_tol_rad": float(ORIENT_TOL_RAD),
            "orient_tol_deg": float(np.degrees(ORIENT_TOL_RAD)),
            "sigma_min_floor": ARC_SIGMA_MIN_FLOOR,
            "sigma_min_floor_source": "min singular value at x_minus_0.03, the best-conditioned cube position with a physically verified Task 1 grasp",
            "handle_z": ARC_HANDLE_Z,
            "min_theta_deg": ARC_MIN_THETA_DEG,
            "min_radius_m": ARC_MIN_RADIUS_M,
        },
        "n_admissible_arcs": len(candidates),
        "top_candidates": candidates[:10],
        "locked": candidates[0] if candidates else None,
    }
    GEOMETRY_PATH.write_text(json.dumps(derived, indent=2, sort_keys=True))
    print(f"gate: {gate}  admissible arcs: {len(candidates)}")
    if candidates:
        c = candidates[0]
        print(f"best: chord {c['chord_m']*100:.1f}cm  theta {c['theta_deg']:.0f}deg  r {c['radius_m']}m  pivot {c['pivot_xy']}")
    print(f"written: {GEOMETRY_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
