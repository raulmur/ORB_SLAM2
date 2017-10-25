@echo off

setlocal 
set "VcpkgBinDir=E:\Software\vcpkg\vcpkg\installed\x86-windows-v141\bin"
set "OrbSlam2ProductsDir=%~dp0\products\cmake.msbuild.windows.x86.v141"
set "Path=%Path%;%VcpkgBinDir%"
"%~dp0\products\cmake.msbuild.windows.x86.v141\Release\mono_video.exe" %~1 "%~dp0Vocabulary\ORBvoc.txt" "%~dp0Examples\Monocular\Garching-Test-Drive.yaml"
endlocal

