@echo off

setlocal

call "%~dp0bootstrap.bat" %*
if errorlevel 1 echo Bootstrapping error, bailing out & exit /b 1

if NOT "%~1"=="" set "OrbSlamPlatform=%~1"
if NOT "%~2"=="" set "OrbSlamToolset=%~2"
if NOT "%~3"=="" set "OrbSlamBuildType=%~3" 

set "VcPkgDir=%~d0\Software\vcpkg\vcpkg"
set "VcPkgTriplet=%OrbSlamPlatform%-windows-%OrbSlamToolset%"
set "VcPkgTripletDir=%VcPkgDir%\installed\%VcPkgTriplet%"
if not exist "%VcPkgTripletDir%" echo %VcPkgTripletDir% does not exist, bailing out & exit /b 1
set "CMAKE_PREFIX_PATH=%VcPkgTripletDir%;%CMAKE_PREFIX_PATH%"

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

call cmake.exe -G "%OrbSlamCMakeGeneratorName%" -DBUILD_EXAMPLES=ON -DVCPKG_TARGET_TRIPLET=%VcPkgTriplet% -DCMAKE_TOOLCHAIN_FILE="%VcPkgDir%\scripts\buildsystems\vcpkg.cmake" "%~dp0..\.."
rem call cmake.exe --build . --config %OrbSlamBuildType%

endlocal
