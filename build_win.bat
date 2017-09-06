@echo off

setlocal

set "MacsPlatform=x86"
set "MacsToolset=v140"
set "Buildtype=Release"

set "BuildDir=%~dp0\ORB_SLAM2.%MacsPlatform%.%MacsToolset%"
if not exist "%BuildDir%" mkdir %BuildDir%
cd "%BuildDir%"

if NOT "%~1"=="" set "MacsPlatform=%~1"
if NOT "%~2"=="" set "MacsToolset=%~2"
if NOT "%~3"=="" set "Buildtype=%~3"

set "MacsCMakeGeneratorName=Visual Studio 14 2015"

if "%MacsPlatform%"=="x86" (
    if "%MacsToolset%"=="v140" set "MacsCMakeGeneratorName=Visual Studio 14 2015"
    if "%MacsToolset%"=="v141" set "MacsCMakeGeneratorName=Visual Studio 15 2017"
)

if "%MacsPlatform%"=="x64" (
    if "%MacsToolset%"=="v140" set "MacsCMakeGeneratorName=Visual Studio 14 2015 Win64"
    if "%MacsToolset%"=="v141" set "MacsCMakeGeneratorName=Visual Studio 15 2017 Win64"
)

call cmake.exe -G "%MacsCMakeGeneratorName%" -DORBSLAM2_STATIC_LIB=ON -DG2O_STATIC_LIB=ON -DDBOW2_STATIC_LIB=ON -DBUILD_EXAMPLES=ON ..
call cmake.exe --build . --config %Buildtype%
endlocal
