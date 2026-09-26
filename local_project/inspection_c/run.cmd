@echo off
setlocal
cd /d "%~dp0"
if exist inspection_planner_v6.exe goto run
call build.cmd
if errorlevel 1 (
    pause
    exit /b 1
)
:run
inspection_planner_v6.exe
if errorlevel 1 pause
