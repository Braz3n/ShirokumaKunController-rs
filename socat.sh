#!/bin/bash

PORT=/dev/ttyACM0
BAUD=115200
ELF=/home/zane/embassy/aircon-rs/target/thumbv8m.main-none-eabihf/release/aircon-rs

socat ${PORT},rawer,b${BAUD} STDOUT | defmt-print -e ${ELF}