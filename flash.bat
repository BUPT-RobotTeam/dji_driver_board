openocd -f interface/cmsis-dap.cfg  -f target/stm32f4x.cfg -c "program build/dji_driver_board.elf verify reset exit"
