@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /nologo /EHsc /std:c++14 /Fe:build\probe.exe build\probe.cpp
if errorlevel 1 exit /b 1
build\probe.exe
