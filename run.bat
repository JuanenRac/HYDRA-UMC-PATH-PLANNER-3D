@echo off
REM HYDRA-UMC-PATH-PLANNER-3D - run.bat
REM Runs the already-built release binary. Run build.bat first.
REM Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
REM GPL-3.0 - see LICENSE
setlocal
cd /d "%~dp0"

REM build\ is checked first because that is where build.bat copies the
REM binary it just compiled (the "shipped" copy); target\release\ is
REM cargo's own default output directory, kept as a fallback for anyone
REM who ran `cargo build --release` directly without going through
REM build.bat's copy step.
if exist build\hydra-umc-path-planner-3d.exe (
    build\hydra-umc-path-planner-3d.exe %*
) else if exist target\release\hydra-umc-path-planner-3d.exe (
    target\release\hydra-umc-path-planner-3d.exe %*
) else (
    echo No compiled binary found. Run build.bat first.
    pause
    exit /b 1
)
endlocal
pause
