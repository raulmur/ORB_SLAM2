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


if NOT EXIST "%VcPkgDir%" echo "vcpkg path is not set correctly", bailing out & exit /b 0

rem ==============================
rem Update and Install packages.
rem ==============================
call "%VcPkgPath%" update
call "%VcPkgPath%" install opencv pangolin --triplet %VcPkgTriplet%

rem build project

set "OrbSlamCMakeGeneratorName=Visual Studio 14 2015"

if "%OrbSlamPlatform%"=="x86" (
    if "%OrbSlamToolset%"=="v140" set "OrbSlamCMakeGeneratorName=Visual Studio 14 2015"
    if "%OrbSlamToolset%"=="v141" set "OrbSlamCMakeGeneratorName=Visual Studio 15 2017"
)

if "%OrbSlamPlatform%"=="x64" (
    if "%OrbSlamToolset%"=="v140" set "OrbSlamCMakeGeneratorName=Visual Studio 14 2015 Win64"
    if "%OrbSlamToolset%"=="v141" set "OrbSlamCMakeGeneratorName=Visual Studio 15 2017 Win64"
)

rem set ORB_3RD_PARTY_LIB_DIR for opencv and pangolin library path
set "ORB_SLAM_3RD_PARTY_LIB_DIR=%VcPkgDir%\installed\%VcPkgTriplet%"

if NOT EXIST "%ORB_SLAM_3RD_PARTY_LIB_DIR%" echo "OrbSlam dependant lib path is not set correctly", bailing out & exit /b 0

set "CMAKE_PREFIX_PATH=%ORB_SLAM_3RD_PARTY_LIB_DIR%;%CMAKE_PREFIX_PATH%"

rem creating and build orb-slam project
set "BuildDir=%~dp0\products\windows-%OrbSlamPlatform%-%OrbSlamToolset%"-%Buildtype%
if not exist "%BuildDir%" mkdir %BuildDir%
cd "%BuildDir%"

call cmake.exe -G "%OrbSlamCMakeGeneratorName%" -DORBSLAM2_STATIC_LIB=ON -DG2O_STATIC_LIB=ON -DDBOW2_STATIC_LIB=ON -DBUILD_EXAMPLES=ON -DBUILD_THIRDPARTY_LIB=ON "%~dp0"
call cmake.exe --build . --config %Buildtype%