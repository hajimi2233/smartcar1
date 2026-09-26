@echo off
setlocal
cd /d "%~dp0"
where cl >nul 2>nul
if not errorlevel 1 goto compile
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" goto missing
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSROOT=%%i"
if not defined VSROOT goto missing
call "%VSROOT%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
:compile
if not exist build mkdir build
cl /nologo /std:c11 /utf-8 /W4 /WX /O2 /MT /Fobuild\ /Feinspection_planner_v6.exe main.c planner.c motion.c map_import.c session.c
if errorlevel 1 exit /b 1
cl /nologo /std:c11 /utf-8 /W4 /WX /O2 /MT /Fobuild\ /Febuild\planner_tests.exe tests.c planner.c
if errorlevel 1 exit /b 1
build\planner_tests.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c11 /utf-8 /W4 /WX /O2 /MT /Fobuild\ /Febuild\motion_tests.exe motion_tests.c planner.c motion.c map_import.c session.c
if errorlevel 1 exit /b 1
build\motion_tests.exe
exit /b %errorlevel%
:missing
echo C compiler not found. Install Visual Studio C++ Build Tools or compile with GCC.
exit /b 1
