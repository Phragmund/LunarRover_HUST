#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
echo "M3508 RX monitor: http://localhost:8089/m3508_rx_monitor.html"
python3 -m http.server 8089

