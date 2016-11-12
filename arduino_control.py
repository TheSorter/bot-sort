#!/usr/bin/env python3

import argparse
import serial


#fancy command line argument. In command line type: python3 arduino_control.py 
parser = argparse.ArgumentParser(description='This is a script for controlling Arduino over serial')
parser.add_argument("-p", "--port", default="/dev/tty.usbmodem1411", help="Path to serial device")
parser.add_argument("-b", "--baud", type=int, default=9600, help="Baud rate for communication")
parser.add_argument("-c", "--comp", help="Set specific computer. Automatically gets port.")
args = parser.parse_args()

port = args.port
baud = args.baud


#for displaying settings
center_val = 50
title_string = "BOT SORT".center(center_val)
port_string = "Port: {}".format(port).center(center_val)
baud_string = "Baud rate: {}".format(baud).center(center_val)



print()
print(title_string, end="\n\n")
print(port_string)
print(baud_string, end="\n\n")




ser = serial.Serial(port,baud)
ser.write(bytes())
# while True:




ser.close()