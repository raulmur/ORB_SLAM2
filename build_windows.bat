@echo off

setlocal

set "OrbSlamPlatform=x86"
set "OrbSlamToolset=v140"
set "Buildtype=Release"

if NOT "%~1"=="" set "OrbSlamPlatform=%~1"
if NOT "%~2"=="" set "OrbSlamToolset=%~2"
if NOT "%~3"=="" set "Buildtype=%~3"

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
if "%ORB_3RD_PARTY_LIB_DIR%"=="" (
echo "it is not defined"
set "ORB_3RD_PARTY_LIB_DIR=%USERPROFILE%\Software\lib\%OrbSlamPlatform%-windows-%OrbSlamToolset%"
)

set "CMAKE_PREFIX_PATH=%ORB_3RD_PARTY_LIB_DIR%;%CMAKE_PREFIX_PATH%"

rem creating and build orb-slam project
set "BuildDir=%~dp0\products\windows-%OrbSlamPlatform%-%OrbSlamToolset%"-%Buildtype%
if not exist "%BuildDir%" mkdir %BuildDir%
cd "%BuildDir%"

call cmake.exe -G "%OrbSlamCMakeGeneratorName%" -DORBSLAM2_STATIC_LIB=ON -DG2O_STATIC_LIB=ON -DDBOW2_STATIC_LIB=ON -DBUILD_EXAMPLES=ON "%~dp0"
call cmake.exe --build . --config %Buildtype%
endlocal
