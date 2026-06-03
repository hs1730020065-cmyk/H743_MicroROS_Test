@echo off
set "OPENOCD=D:\stm32cubeide\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.openocd.win32_2.4.400.202601091506\tools\bin\openocd.exe"
set "OPENOCD_SCRIPTS=D:\stm32cubeide\STM32CubeIDE_2.1.1\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.debug.openocd_2.3.300.202602021527\resources\openocd\st_scripts"

"%OPENOCD%" ^
  -s "%OPENOCD_SCRIPTS%" ^
  -f interface/stlink-dap.cfg ^
  -c "transport select dapdirect_swd" ^
  -c "reset_config srst_only srst_nogate connect_assert_srst" ^
  -c "adapter speed 1000" ^
  -c "gdb_port 3333" ^
  -c "tcl_port disabled" ^
  -c "telnet_port disabled" ^
  -c "set AP_NUM 0" ^
  -f target/stm32h7x.cfg
