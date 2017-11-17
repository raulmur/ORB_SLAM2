@echo off

setlocal 
set "VcpkgBinDir=D:\git\vcpkg\installed\x64-windows-v141\bin"
set "ProjectDir=%~dp0..\.."
set "OrbSlam2ProductsDir=%ProjectDir%\products\cmake.msbuild.windows.x64.v141"
set "Path=%Path%;%VcpkgBinDir%"
"%OrbSlam2ProductsDir%\Debug\mono_video.exe" \
    %~1 \
    "%ProjectDir%\Vocabulary\ORBvoc.txt" \
    "%ProjectDir%\Examples\Monocular\Garching-Test-Drive.yaml"
endlocal

