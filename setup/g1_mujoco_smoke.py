#!/usr/bin/env python3
"""Headless MuJoCo smoke test for the official Unitree G1 scene."""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "vendor" / "unitree_mujoco" / "unitree_robots" / "g1" / "scene.xml"
ARTIFACT_DIR = ROOT / "artifacts"
LOG_DIR = ROOT / "logs"


def named_qpos(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str) -> float:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if joint_id < 0:
        raise RuntimeError(f"Joint not found: {joint_name}")
    qpos_addr = model.jnt_qposadr[joint_id]
    return float(data.qpos[qpos_addr])


def main() -> int:
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    LOG_DIR.mkdir(parents=True, exist_ok=True)

    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    results: dict[str, object] = {
        "started_at": started_at,
        "scene": str(SCENE),
        "mujoco_version": mujoco.__version__,
        "numpy_version": np.__version__,
        "checks": {},
    }

    if not SCENE.exists():
        raise FileNotFoundError(SCENE)

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    results["model"] = {
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nu": int(model.nu),
        "nbody": int(model.nbody),
        "njnt": int(model.njnt),
        "nsensor": int(model.nsensor),
    }
    results["checks"]["model_loads"] = True

    initial_time = float(data.time)
    for _ in range(20):
        mujoco.mj_step(model, data)
    advanced_time = float(data.time)
    results["checks"]["simulation_advances"] = advanced_time > initial_time
    results["initial_time"] = initial_time
    results["advanced_time"] = advanced_time

    joint_name = "left_shoulder_pitch_joint"
    before_reset_joint = named_qpos(model, data, joint_name)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    after_reset_time = float(data.time)
    after_reset_joint = named_qpos(model, data, joint_name)
    results["checks"]["reset_works"] = after_reset_time == 0.0
    results["joint_read"] = {
        "joint": joint_name,
        "before_reset_qpos": before_reset_joint,
        "after_reset_qpos": after_reset_joint,
    }
    results["checks"]["joint_state_can_be_read"] = bool(np.isfinite(after_reset_joint))

    actuator_name = "left_shoulder_pitch"
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
    if actuator_id < 0:
        raise RuntimeError(f"Actuator not found: {actuator_name}")

    baseline = named_qpos(model, data, joint_name)
    data.ctrl[actuator_id] = 0.2
    for _ in range(200):
        mujoco.mj_step(model, data)
    commanded = named_qpos(model, data, joint_name)
    delta = commanded - baseline
    results["command"] = {
        "actuator": actuator_name,
        "ctrl": 0.2,
        "joint": joint_name,
        "baseline_qpos": baseline,
        "commanded_qpos": commanded,
        "delta_qpos": delta,
    }
    results["checks"]["safe_joint_command_changes_state"] = abs(delta) > 1e-6

    screenshot_path = ARTIFACT_DIR / "g1_scene_smoke.png"
    try:
        renderer = mujoco.Renderer(model, height=480, width=640)
        try:
            renderer.update_scene(data)
            pixels = renderer.render()
        finally:
            renderer.close()
        try:
            import imageio.v3 as iio
        except Exception:
            iio = None
        if iio is None:
            results["checks"]["screenshot_captured"] = False
            results["screenshot_error"] = "imageio.v3 is not installed"
        else:
            iio.imwrite(screenshot_path, pixels)
            results["checks"]["screenshot_captured"] = screenshot_path.exists()
            results["screenshot"] = str(screenshot_path)
    except Exception as exc:
        results["checks"]["screenshot_captured"] = False
        results["screenshot_error"] = repr(exc)

    log_path = LOG_DIR / "g1_mujoco_smoke.json"
    log_path.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")

    print(json.dumps(results, indent=2))
    required = [
        "model_loads",
        "simulation_advances",
        "reset_works",
        "joint_state_can_be_read",
        "safe_joint_command_changes_state",
    ]
    return 0 if all(results["checks"].get(name) for name in required) else 1


if __name__ == "__main__":
    os.environ.setdefault("MUJOCO_GL", "glfw")
    raise SystemExit(main())
