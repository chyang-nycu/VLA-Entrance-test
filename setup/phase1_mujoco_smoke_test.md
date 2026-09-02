# Phase 1 MuJoCo Smoke Test

Date: 2026-09-02

## Target

- Simulator source: official Unitree `unitree_mujoco`
- Robot: Unitree G1
- Scene tested: `vendor/unitree_mujoco/unitree_robots/g1/scene.xml`
- MuJoCo Python package: `mujoco==3.3.6`
- Python: project-local `.venv` using Python 3.12.14
- Pinned repository commit: `4134cb5dc7ff1ba7f484deda48b5274b58694519`

## Commands

```bash
./setup/preflight_macos.sh
.venv/bin/python setup/g1_mujoco_smoke.py
```

## Results

The smoke test was executed with the official G1 `scene.xml`.

| Check | Result |
| --- | --- |
| Model loads | Pass |
| Simulation advances | Pass |
| Reset works | Pass |
| Joint state can be read | Pass |
| Small safe joint command changes simulated state | Pass |
| Screenshot captured | Not captured |

## Evidence

- JSON log: `logs/g1_mujoco_smoke.json`
- Preflight log: `logs/preflight_macos.log`

## Recorded Versions

Host tools:

- macOS 12.7.6 (`21H1320`)
- Architecture: `x86_64`
- RAM: 17179869184 bytes
- Xcode Command Line Tools: `/Library/Developer/CommandLineTools`
- Apple clang 14.0.0
- Homebrew 6.0.21
- CMake 4.4.3
- Apple Git 2.37.1
- System `python3`: 3.14.7
- Project Python: 3.12.14

Project Python packages:

- `absl-py==2.5.0`
- `etils==1.14.0`
- `fsspec==2026.7.0`
- `glfw==2.10.2`
- `ImageIO==2.37.4`
- `mujoco==3.3.6`
- `numpy==2.5.2`
- `pillow==12.3.0`
- `PyOpenGL==3.1.10`
- `typing_extensions==4.16.0`
- `zipp==4.1.0`

Observed model dimensions:

- `nq`: 36
- `nv`: 35
- `nu`: 29
- `nbody`: 31
- `njnt`: 30
- `nsensor`: 95

Safe command used:

- Actuator: `left_shoulder_pitch`
- Joint: `left_shoulder_pitch_joint`
- Control: `0.2`
- Observed joint delta: `-0.11684565300949047`

## Screenshot Status

Screenshot capture was attempted with MuJoCo's offscreen renderer. It did not complete in the Codex terminal session because macOS reported `CGLError('invalid CoreGraphics connection')`. The physics smoke checks do not depend on this graphics path.

## Notes

- The official README checked on 2026-09-02 is Linux-first for the C++ simulator and recommends apt packages plus `unitree_sdk2`: https://github.com/unitreerobotics/unitree_mujoco/blob/main/readme.md
- The Python simulator path documents `unitree_sdk2_python`, `mujoco`, and `pygame`.
- G1 uses the `unitree_hg` IDL for DDS integration. This Phase 1 smoke test did not install or validate DDS messaging because direct MuJoCo model/physics validation is the minimum required simulator component set for the requested checks.
- An unpinned `pip install mujoco` selected `mujoco-3.12.0` source distribution and failed with `MUJOCO_PATH environment variable is not set`. Pinning to the README-aligned `mujoco==3.3.6` installed a binary wheel successfully.
- Cloning on the default macOS case-insensitive filesystem reports a Go2w asset collision between `terrain.STL` and `terrain.stl`. This does not affect the G1 scene tested here.
