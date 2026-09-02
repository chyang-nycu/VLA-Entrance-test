#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${ROOT_DIR}/logs"
LOG_FILE="${LOG_DIR}/preflight_macos.log"

mkdir -p "${LOG_DIR}"
: > "${LOG_FILE}"

run_check() {
  local label="$1"
  shift
  {
    printf '\n## %s\n' "${label}"
    printf '$'
    printf ' %q' "$@"
    printf '\n'
    "$@"
    printf 'exit=%s\n' "$?"
  } 2>&1 | tee -a "${LOG_FILE}"
}

run_check "macOS version" sw_vers
run_check "Architecture" uname -m
run_check "RAM bytes" sysctl hw.memsize
run_check "Workspace disk" df -h "${ROOT_DIR}"
run_check "Xcode Command Line Tools path" xcode-select -p
run_check "Clang" clang --version
run_check "Homebrew" brew --version
run_check "CMake" cmake --version
run_check "Git" git --version
run_check "Python 3" python3 --version

if command -v python3.12 >/dev/null 2>&1; then
  run_check "Python 3.12" python3.12 --version
fi

if [[ -x "${ROOT_DIR}/.venv/bin/python" ]]; then
  run_check "Project Python" "${ROOT_DIR}/.venv/bin/python" --version
  run_check "Project packages" "${ROOT_DIR}/.venv/bin/python" -m pip freeze
fi

if [[ -d "${ROOT_DIR}/vendor/unitree_mujoco/.git" ]]; then
  run_check "unitree_mujoco commit" git -C "${ROOT_DIR}/vendor/unitree_mujoco" log -1 --format=%H%n%cI%n%s
  run_check "unitree_mujoco status" git -C "${ROOT_DIR}/vendor/unitree_mujoco" status --short
fi
