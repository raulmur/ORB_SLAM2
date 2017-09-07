@echo off

setlocal

set "OrbSlamPlatform=x86"
set "OrbSlamToolset=v140"
set "Buildtype=Release"

if NOT "%~1"=="" set "OrbSlamPlatform=%~1"
if NOT "%~2"=="" set "OrbSlamToolset=%~2"
if NOT "%~3"=="" set "Buildtype=%~3"

set "VcPkgGitUrl=https://github.com/paul-michalik/vcpkg.git"
set "VcPkgTag=0.0.81-5"
set "VcPkgDir=%~d0\Software\vcpkg\vcpkg-%VcPkgTag%"
if defined VCPKG_ROOT_DIR if /i not "%VCPKG_ROOT_DIR%"=="" set "VcPkgDir=%VCPKG_ROOT_DIR%"
set "VcPkgTriplet=%OrbSlamPlatform%-windows"
if defined VCPKG_DEFAULT_TRIPLET if /i not "%VCPKG_DEFAULT_TRIPLET%"=="" set "VcpkgTriplet=%VCPKG_DEFAULT_TRIPLET%"
set "VcPkgPath=%VcPkgDir%\vcpkg.exe"
set "ORB_3RD_PARTY_LIB_DIR=%VcPkgDir%\installed\%VcPkgTriplet%"

echo. & echo Installing/updating vcpkg...
call :BootstrapVcPkg "%VcPkgGitUrl%" "%VcPkgTag%" "%VcPkgDir%"
if %errorlevel% neq 0 echo An error occured in %~n0, bailing out & exit /b %errorlevel%
echo. & echo vcpkg %MacsVcPkgTag% successfully installed...

rem ==============================
rem Update and Install packages.
rem ==============================
call "%VcPkgPath%" update
call "%VcPkgPath%" install opencv pangolin --triplet %VcPkgTriplet%

rem build project
call "%~dp0build_windows.bat" %*
rem ==============================
rem Utilities. Do not add anything beyond this point.
rem ==============================

goto :eof

:BootstrapVcPkg
setlocal
    set "LocErrorState=0"
    set "LocVcPkgGit=%~1"
    set "LocVcPkgTag=%~2"
    set "LocVcPkgDir=%~3"
    set "LocVcPkgTag=tags/%LocVcPkgTag%"

    echo Checking for existence of "%LocVcPkgDir%"...
    if not exist "%LocVcPkgDir%" (
        echo Attempt to: git clone "%LocVcPkgGit%" "%LocVcPkgDir%"...
        git clone "%LocVcPkgGit%" "%LocVcPkgDir%"
    )
    if %errorlevel% neq 0 echo An error occured in %~n0, bailing out & exit /b %errorlevel%

    pushd "%LocVcPkgDir%"
    if %errorlevel% neq 0 echo An error occured in %~n0, bailing out & exit /b %errorlevel%

    echo Attempt to: git fetch --all --tags --prune...
    git fetch --all --tags --prune
    if %errorlevel% neq 0 echo An error occured in %~n0, bailing out & exit /b %errorlevel

    rem echo Attempt to: git checkout %LocVcPkgTag%...
    rem git checkout %LocVcPkgTag%
    rem if %errorlevel% neq 0 echo An error occured in %~n0, bailing out & exit /b %errorlevel

	echo Attempt to: git checkout bug/pangolin...
    git checkout bug/pangolin
    if %errorlevel% neq 0 echo An error occured in %~n0, bailing out & exit /b %errorlevel
	
    rem 
    rem It is not clear to me when to call bootstrapping script, so I do call it always...
    rem 
    if not exist vcpkg.exe call "bootstrap-vcpkg.bat"
    if %errorlevel% neq 0 echo An error occured in %~n0, bailing out & exit /b %errorlevel%

    popd
endlocal & exit /b %errorlevel%

goto :eof
