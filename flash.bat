openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program build/dji_driver_board.elf verify reset exit"
