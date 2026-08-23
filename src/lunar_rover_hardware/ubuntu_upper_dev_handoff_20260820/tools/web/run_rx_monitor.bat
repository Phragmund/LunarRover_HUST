@echo off
cd /d "%~dp0"
echo M3508 RX monitor: http://localhost:8089/m3508_rx_monitor.html
python -m http.server 8089
