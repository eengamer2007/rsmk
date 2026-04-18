#!/bin/bash

cargo b
cp target/thumbv8m.main-none-eabihf/debug/rsmk rsmk.elf
openocd -f interface/cmsis-dap.cfg -f rp2350.cfg -c "adapter speed 5000" -c "program rsmk.elf verify reset exit"
