@echo off

setlocal

call "%~dp0bootstrap_windows.bat" %*
if errorlevel 1 echo Bootstrapping error, bailing out & exit /b 1

set "OrbSlamCMakeGeneratorName=Visual Studio 14 2015"

if "%OrbSlamPlatform%"=="x86" (
    if "%OrbSlamToolset%"=="v140" set "OrbSlamCMakeGeneratorName=Visual Studio 14 2015"
    if "%OrbSlamToolset%"=="v141" set "OrbSlamCMakeGeneratorName=Visual Studio 15 2017"
)

if "%OrbSlamPlatform%"=="x64" (
    if "%OrbSlamToolset%"=="v140" set "OrbSlamCMakeGeneratorName=Visual Studio 14 2015 Win64"
    if "%OrbSlamToolset%"=="v141" set "OrbSlamCMakeGeneratorName=Visual Studio 15 2017 Win64"
)

set "OrbSlamBuildDir=%~dp0\products\cmake.msbuild.windows.%OrbSlamPlatform%.%OrbSlamToolset%"
if not exist "%OrbSlamBuildDir%" mkdir "%OrbSlamBuildDir%"
cd "%OrbSlamBuildDir%"

echo:& echo Invoking cmake like this as: cmake.exe -G "%OrbSlamCMakeGeneratorName%" -DORBSLAM2_STATIC_LIB=ON -DG2O_STATIC_LIB=ON -DDBOW2_STATIC_LIB=ON -DBUILD_EXAMPLES=ON -DBUILD_THIRDPARTY_LIB=ON "%~dp0" & echo.

call cmake.exe -G "%OrbSlamCMakeGeneratorName%" -DORBSLAM2_STATIC_LIB=ON -DG2O_STATIC_LIB=ON -DDBOW2_STATIC_LIB=ON -DBUILD_EXAMPLES=ON -DBUILD_THIRDPARTY_LIB=ON "%~dp0"
rem call cmake.exe --build . --config %OrbSlamBuildtype%
endlocal
