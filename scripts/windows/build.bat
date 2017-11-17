@echo off

setlocal

call "%~dp0bootstrap.bat" %*
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

set "OrbSlamBuildDir=%~dp0..\..\products\cmake.msbuild.windows.%OrbSlamPlatform%.%OrbSlamToolset%"
if not exist "%OrbSlamBuildDir%" mkdir "%OrbSlamBuildDir%"
cd "%OrbSlamBuildDir%"

call cmake.exe -G "%OrbSlamCMakeGeneratorName%" -DBUILD_EXAMPLES=ON "%~dp0..\.."
call cmake.exe --build . --config %OrbSlamBuildType%

endlocal
