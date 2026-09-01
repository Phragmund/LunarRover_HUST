#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv="$repo_root/.deps/target_perception_venv"

python3 -m venv --system-site-packages "$venv"
"$venv/bin/python" -m pip install --upgrade pip wheel
"$venv/bin/python" -m pip install --index-url https://download.pytorch.org/whl/cpu torch torchvision
"$venv/bin/python" -m pip install -r "$repo_root/dependencies/target_perception.txt"

cat <<EOF
Target-perception environment is ready at:
  $venv

Activate it before training or launching the Ultralytics backend:
  source $venv/bin/activate
EOF
