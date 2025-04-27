#!/bin/bash

cargo b --release
picotool uf2 convert target/thumbv8m.main-none-eabihf/release/rsmk -t elf out.uf2 -t uf2
picotool load out.uf2
picotool reboot
