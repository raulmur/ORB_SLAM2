@echo off

setlocal

set "OrbSlamPlatform=x86"
set "OrbSlamToolset=v140"
set "Buildtype=Release"

if NOT "%~1"=="" set "OrbSlamPlatform=%~1"
if NOT "%~2"=="" set "OrbSlamToolset=%~2"
if NOT "%~3"=="" set "Buildtype=%~3"

set "VcPkgTag=0.0.81-5"
set "VcPkgDir=%~d0\Software\vcpkg\vcpkg-%VcPkgTag%"
if defined VCPKG_ROOT_DIR if /i not "%VCPKG_ROOT_DIR%"=="" set "VcPkgDir=%VCPKG_ROOT_DIR%"
set "VcPkgTriplet=%OrbSlamPlatform%-windows"
if defined VCPKG_DEFAULT_TRIPLET if /i not "%VCPKG_DEFAULT_TRIPLET%"=="" set "VcpkgTriplet=%VCPKG_DEFAULT_TRIPLET%"
set "VcPkgPath=%VcPkgDir%\vcpkg.exe"
set "ORB_SLAM_3RD_PARTY_LIB_DIR=%VcPkgDir%\installed\%VcPkgTriplet%"


if NOT EXIST "%VcPkgDir%" echo "vcpkg path is not set correctly", bailing out & exit /b 0

rem ==============================
rem Update and Install packages.
rem ==============================
call "%VcPkgPath%" update
call "%VcPkgPath%" install opencv pangolin --triplet %VcPkgTriplet%

rem build project
call "%~dp0build_windows.bat" %*