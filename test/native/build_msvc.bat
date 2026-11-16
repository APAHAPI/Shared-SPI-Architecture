@echo off
rem Host tests for the portable core. Run from the repository root:
rem   test\native\build_msvc.bat
setlocal

call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 (
  echo ERROR: could not initialise the MSVC environment.
  exit /b 1
)

if not exist build mkdir build

cl /nologo /EHsc /std:c++14 /W4 /WX- /I src "/Fobuild\\" ^
   test\native\test_main.cpp ^
   src\BNO085Types.cpp ^
   src\BNO085Sensor.cpp ^
   src\diag\Logger.cpp ^
   src\output\BinaryEncoder.cpp ^
   src\sh2\Sh2Client.cpp ^
   src\sh2\Sh2Codes.cpp ^
   src\sh2\Sh2Reports.cpp ^
   src\shtp\ShtpTransport.cpp ^
   src\util\Crc16.cpp ^
   /Fe:build\native_tests.exe
if errorlevel 1 (
  echo BUILD: FAIL
  exit /b 1
)

build\native_tests.exe
if errorlevel 1 (
  echo RESULT: FAIL
  exit /b 1
)
echo RESULT: PASS
