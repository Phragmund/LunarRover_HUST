@echo off
cd /d "%~dp0"
echo Rover web controller: http://localhost:8088/rover_web_controller.html
echo M3508-only page: http://localhost:8088/m3508_web_controller.html
python -m http.server 8088
