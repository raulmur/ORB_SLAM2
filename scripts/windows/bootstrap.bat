@echo off

set "OrbSlamPlatform=x86"
set "OrbSlamToolset=v141"
set "OrbSlamBuildType=Release"

if NOT "%~1"=="" set "OrbSlamPlatform=%~1"
if NOT "%~2"=="" set "OrbSlamToolset=%~2"
if NOT "%~3"=="" set "OrbSlamBuildType=%~3" 

rem ----------------------------------
rem Locate vcpkg using environment variables
rem ----------------------------------
set "VcPkgDir=%~d0\Software\vcpkg\vcpkg"
set "VcpkgTriplet=%OrbSlamPlatform%-windows-%OrbSlamToolset%"
if defined VCPKG_ROOT_DIR if /i not "%VCPKG_ROOT_DIR%"=="" set "VcPkgDir=%VCPKG_ROOT_DIR%"
if defined VCPKG_DEFAULT_TRIPLET if /i not "%VCPKG_DEFAULT_TRIPLET%"=="" set "VcpkgTriplet=%VCPKG_DEFAULT_TRIPLET%"

rem ----------------------------------
rem Try to look for vcpkg at default locations
rem ----------------------------------
if not exist "%VcPkgDir%" set "VcPkgDir=%~d0\Software\vcpkg\vcpkg"
if not exist "%VcPkgDir%" set "VcPkgDir=%~d0\.vcpkg\vcpkg"
if not exist "%VcPkgDir%" set "VcPkgDir=C:\Software\vcpkg\vcpkg"
if not exist "%VcPkgDir%" set "VcPkgDir=C:\.vcpkg\vcpkg"
if not exist "%VcPkgDir%" set "VcPkgDir=%USERPROFILE%\.vcpkg\vcpkg"
if not exist "%VcPkgDir%" (
	echo vcpkg not found, installing at %VcPkgDir%...
    git clone --recursive https://github.com/paul-michalik/vcpkg.git "%VcPkgDir%"
) else (
	echo vcpkg found at %VcPkgDir%...
    pushd "%VcPkgDir%"
    git pull --all --prune
	popd 
)

if not exist "%VcPkgDir%" echo vcpkg path is not set correctly, bailing out & exit /b 1

call "%VcPkgDir%\bootstrap-vcpkg.bat"

echo. & echo Bootstrapping dependencies for triplet: %VcPkgTriplet% & echo.

set "VcPkgPath=%VcPkgDir%\vcpkg.exe"

if not exist "%VcPkgPath%" echo vcpkg path is not set correctly, bailing out & exit /b 1

rem ==============================
rem Update and Install packages.
rem ==============================
call "%VcPkgPath%" update
call "%VcPkgPath%" remove --outdated --recurse
call "%VcPkgPath%" install boost eigen3 opencv pangolin --triplet %VcPkgTriplet%

set "VcPkgTripletDir=%VcPkgDir%\installed\%VcPkgTriplet%"

if not exist "%VcPkgTripletDir%" echo %VcPkgTripletDir% does not exist, bailing out & exit /b 1

set "CMAKE_PREFIX_PATH=%VcPkgTripletDir%;%CMAKE_PREFIX_PATH%"

echo. & echo Bootstrapping successful for triplet: %VcPkgTriplet% & echo CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH% & echo.
