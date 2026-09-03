"""Phase 5B: canonical data-generation manifest loader/verifier.

`data/task1_canonical_config.json` is the single source of truth for which
scene generator, controller path, gains, camera, and success thresholds are
authorized for VLA demonstration collection. The collector, validator, and
replay tool all import `load_manifest()` / `verify_environment_matches_manifest()`
from this module -- there is no second copy of these numbers anywhere else
in the Phase 5B code.

Verification is fail-loud: any live value that differs from the manifest
raises `ManifestMismatchError`, it never warns-and-continues. This is a
deliberate anti-drift guard -- Task 1's controller/gains/thresholds must not
silently diverge between what a human authorized for data collection and
what the collector actually runs.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST_PATH = ROOT / "data" / "task1_canonical_config.json"


class ManifestMismatchError(RuntimeError):
    """Raised when the live environment does not match the canonical manifest."""


def compute_manifest_hash(manifest: dict) -> str:
    """sha256 over canonical JSON (sorted keys, no whitespace) of the
    manifest with its own "hash" key removed -- so the hash is a hash of
    everything ELSE in the manifest, not of itself.
    """
    subset = {k: v for k, v in manifest.items() if k != "hash"}
    blob = json.dumps(subset, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()


def load_manifest(path: Path = MANIFEST_PATH) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        manifest = json.load(f)
    stored_hash = manifest.get("hash", {}).get("value")
    live_hash = compute_manifest_hash(manifest)
    if stored_hash in (None, "COMPUTED_AT_WRITE_TIME_PLACEHOLDER"):
        raise ManifestMismatchError(
            f"{path} has no computed hash stored (value={stored_hash!r}); "
            "run tasks/g1_pick_place/canonical_config.py as a script to stamp it, "
            "or call write_manifest_hash()."
        )
    if stored_hash != live_hash:
        raise ManifestMismatchError(
            f"{path}'s stored hash {stored_hash} does not match its own live content hash "
            f"{live_hash} -- the manifest file was edited without re-stamping its hash. "
            "Re-run write_manifest_hash() after any intentional edit."
        )
    return manifest


def write_manifest_hash(path: Path = MANIFEST_PATH) -> str:
    """Recompute and persist the manifest's own content hash. Use only when
    deliberately authoring/editing the manifest itself -- never called by
    the collector/validator/replay tool at runtime.
    """
    with open(path, "r", encoding="utf-8") as f:
        manifest = json.load(f)
    live_hash = compute_manifest_hash(manifest)
    manifest["hash"]["value"] = live_hash
    with open(path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)
        f.write("\n")
    return live_hash


def manifest_hash(path: Path = MANIFEST_PATH) -> str:
    """The manifest's stored (verified) content hash, for stamping into
    HDF5 episode metadata."""
    return load_manifest(path)["hash"]["value"]


def verify_environment_matches_manifest(
    *,
    scene_generator_name: str,
    use_oriented_ik: bool,
    arm_kp: float,
    arm_kv: float,
    gripper_kp: float,
    gripper_kd: float,
    camera_parent_body: str,
    camera_resolution_wh: tuple[int, int],
    manifest: dict | None = None,
) -> dict:
    """Fail-loud check that the live collection configuration matches the
    canonical manifest. Raises ManifestMismatchError on any mismatch;
    returns the manifest dict on success (so callers can also read fields
    like the instruction string or thresholds from the same verified copy).
    """
    m = manifest if manifest is not None else load_manifest()
    errors = []

    expected_scene_fn = m["scene"]["generator_function"].rsplit(".", 1)[-1]
    if scene_generator_name != expected_scene_fn:
        errors.append(f"scene generator: expected {expected_scene_fn!r}, got {scene_generator_name!r}")

    if bool(use_oriented_ik) != bool(m["controller"]["use_oriented_ik"]):
        errors.append(f"use_oriented_ik: expected {m['controller']['use_oriented_ik']!r}, got {use_oriented_ik!r}")

    expected_arm = m["controller"]["arm_gains"]
    if float(arm_kp) != float(expected_arm["kp"]) or float(arm_kv) != float(expected_arm["kv"]):
        errors.append(f"arm gains: expected kp={expected_arm['kp']} kv={expected_arm['kv']}, got kp={arm_kp} kv={arm_kv}")

    expected_grip = m["controller"]["gripper_gains"]
    if float(gripper_kp) != float(expected_grip["kp"]) or float(gripper_kd) != float(expected_grip["kd"]):
        errors.append(f"gripper gains: expected kp={expected_grip['kp']} kd={expected_grip['kd']}, got kp={gripper_kp} kd={gripper_kd}")

    if camera_parent_body != m["camera"]["parent_body"]:
        errors.append(f"camera parent body: expected {m['camera']['parent_body']!r}, got {camera_parent_body!r}")

    expected_res = tuple(m["camera"]["resolution_wh"])
    if tuple(camera_resolution_wh) != expected_res:
        errors.append(f"camera resolution: expected {expected_res}, got {tuple(camera_resolution_wh)}")

    if errors:
        raise ManifestMismatchError(
            "Live environment does not match data/task1_canonical_config.json:\n  - "
            + "\n  - ".join(errors)
        )
    return m


if __name__ == "__main__":
    h = write_manifest_hash()
    print(f"stamped manifest hash: {h}")
