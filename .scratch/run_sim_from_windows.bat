@echo off
REM Launch the AeroTHON Mission 2 SITL stack in WSL from Windows.
REM Double-click this file, or run it from cmd.
REM
REM The repo lives on the D: drive, which WSL mounts at /mnt/d. The path has
REM spaces in it, so the bash side is single-quoted.
setlocal
set "REPO=/mnt/d/MY DOCUMENTS/VIT/Team Rotor Fpv/AEROTHON"
echo === preflight ===
wsl.exe -- bash -lc "cd '%REPO%' && scripts/preflight_stack.sh"
if errorlevel 1 (
  echo.
  echo Preflight failed. Fix the FAIL lines above before launching.
  pause
  exit /b 1
)
echo === build ===
wsl.exe -- bash -lc "cd '%REPO%' && colcon build --symlink-install"
if errorlevel 1 ( pause & exit /b 1 )
echo === launch ===
wsl.exe -- bash -lc "cd '%REPO%' && source install/setup.bash && scripts/launch_level6_sim.sh"
pause
