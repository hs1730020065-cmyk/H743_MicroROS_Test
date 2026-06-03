@echo off
set "REAL_GDB=D:\stm32cubeide\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-gdb.exe"

for %%A in (%*) do (
  if "%%~A"=="--version" goto version
)

"%REAL_GDB%" %*
exit /b %ERRORLEVEL%

:version
echo GNU gdb (GNU Tools for STM32 14.3.rel1.20251027-0700) 15.2.90.20241229-git
exit /b 0
