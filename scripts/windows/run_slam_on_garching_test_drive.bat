@echo off

setlocal 
set "VcpkgBinDir=E:\Software\vcpkg\vcpkg\installed\x86-windows-v141\bin"
set "ProjectDir=%~dp0..\.."
set "OrbSlam2ProductsDir=%ProjectDir%\products\cmake.msbuild.windows.x86.v141"
set "Path=%Path%;%VcpkgBinDir%"
"%OrbSlam2ProductsDir%\Release\mono_video.exe" \
    %~1 \
    "%ProjectDir%\Vocabulary\ORBvoc.txt" \
    "%ProjectDir%\Examples\Monocular\Garching-Test-Drive.yaml"
endlocal

