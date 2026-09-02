#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${ROOT_DIR}/.venv"
REPO_DIR="${ROOT_DIR}/vendor/unitree_mujoco"
REPO_URL="https://github.com/unitreerobotics/unitree_mujoco.git"
PINNED_MUJOCO="3.3.6"

mkdir -p "${ROOT_DIR}/vendor" "${ROOT_DIR}/logs" "${ROOT_DIR}/artifacts"

if [[ ! -d "${REPO_DIR}/.git" ]]; then
  git clone "${REPO_URL}" "${REPO_DIR}"
fi

if command -v python3.12 >/dev/null 2>&1; then
  PYTHON_BIN="python3.12"
else
  echo "python3.12 is required for this bootstrap on Intel macOS." >&2
  echo "Install Python 3.12 first, then rerun this script." >&2
  exit 1
fi

"${PYTHON_BIN}" -m venv "${VENV_DIR}"
PIP_CACHE_DIR="${ROOT_DIR}/.cache/pip" "${VENV_DIR}/bin/python" -m pip install --upgrade pip setuptools wheel
PIP_CACHE_DIR="${ROOT_DIR}/.cache/pip" "${VENV_DIR}/bin/python" -m pip install --only-binary=:all: \
  "mujoco==${PINNED_MUJOCO}" \
  numpy \
  imageio

"${VENV_DIR}/bin/python" "${ROOT_DIR}/setup/g1_mujoco_smoke.py"
