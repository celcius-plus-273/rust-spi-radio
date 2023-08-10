#!/bin/bash

file_name="client.hex" 	# edit this to changee output file name

board="TEENSY40"     	# edit this for to change target board
						# TEENSY40 = Teensy 4.0
						# TEENSY41 = Teensy 4.1
						# https://github.com/PaulStoffregen/teensy_loader_cli

# compiles/builds the project and converts the file into an output hex file
cargo objcopy --release -- -O ihex target/thumbv7em-none-eabihf/release/$file_name

echo "========================================================="
echo ""
echo "     Press pushbutton on Teensy to start programming"
echo ""
echo "========================================================="

# loads the output hex file into the target board (i.e. flashes the mcu)
./teensy_loader_cli --mcu=$board -w target/thumbv7em-none-eabihf/release/$file_name

echo "========================================================="
echo ""
echo "     $file_name was succesfully flashed into $board"
echo ""
echo "========================================================="
