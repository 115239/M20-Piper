#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${ISAAC_SIM_PYTHON:-}" ]]; then
  exec "$ISAAC_SIM_PYTHON" "$repo_dir/simulation.py" "$@"
elif [[ -n "${ISAAC_SIM_ROOT:-}" && -x "$ISAAC_SIM_ROOT/python.sh" ]]; then
  exec "$ISAAC_SIM_ROOT/python.sh" "$repo_dir/simulation.py" "$@"
else
  echo '请设置 ISAAC_SIM_ROOT=/path/to/isaac-sim 或 ISAAC_SIM_PYTHON=/path/to/isaac-sim/python.sh' >&2
  exit 2
fi
