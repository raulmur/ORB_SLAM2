@echo off

setlocal
set "VcpkgBinDir=%~d0\Software\vcpkg\vcpkg\installed\x86-windows-v141\bin"
set "ProjectDir=%~dp0..\.."
set "Path=%Path%;%VcpkgBinDir%;C:\Program Files\7-Zip"

if not exist "%ProjectDir%\Vocabulary\ORBvoc.txt" (
    7z x "%ProjectDir%\Vocabulary\ORBvoc.txt.tar.gz" -o"%ProjectDir%\Vocabulary"
    7z x "%ProjectDir%\Vocabulary\ORBvoc.txt.tar" -o"%ProjectDir%\Vocabulary"
    del /f /q "%ProjectDir%\Vocabulary\ORBvoc.txt.tar"
)

set "OrbSlam2ProductsDir=%ProjectDir%\products\cmake.msbuild.windows.x86.v141"

set MonoVideoApp="%OrbSlam2ProductsDir%\Release\mono_video.exe"
set VideoFile="%~1"
set VocabularyFile="%ProjectDir%\Vocabulary\ORBvoc.txt"
set SettingsFile="%ProjectDir%\Examples\Monocular\Garching-Test-Drive.yaml"

%MonoVideoApp% %VideoFile% %VocabularyFile% %SettingsFile%

endlocal
