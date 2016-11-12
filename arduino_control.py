#!/usr/bin/env python3

import argparse
import serial # pip3 install pyserial
import time
import sys


def get_input_and_act():
    for line in sys.stdin:
        line = line[:len(line)-1]
        if line == 'recycle':
            do_the_thing(1)
        elif line == 'trash':
            do_the_thing(2)
        time.sleep(2)


def read_file_and_act():
    with open('test.txt') as f:
        for line in f.readlines():
            line = line[:len(line) - 1]
            print(line)


def do_the_thing(motion):
    if motion == '1':
        print('Doing Action 1')
        ser.write('1'.encode('utf-8'))
    elif motion == '2':
        print('Doing Action 2')
        ser.write('2'.encode('utf-8'))
    elif motion == '3':
        print('Doing Action 3')
        ser.write('3'.encode('utf-8'))

# fancy command line argument. In command line type: python3 arduino_control.py
parser = argparse.ArgumentParser(description='This is a script for controlling Arduino over serial')
parser.add_argument("-p", "--port", default="/dev/tty.usbmodem1411", help="Path to serial device")
parser.add_argument("-b", "--baud", type=int, default=9600, help="Baud rate for communication")
parser.add_argument("-c", "--comp", help="Set specific computer. Automatically gets port.")
parser.add_argument("-a", "--action", default='0', help="choose which action to do")
args = parser.parse_args()

if args.comp == 'cole':
    port = '/dev/cu.usbmodemFA131'
else:
    port = args.port

baud = args.baud
action = args.action


# for displaying settings
center_val = 50
title_string = "BOT SORT".center(center_val)
port_string = "Port: {}".format(port).center(center_val)
baud_string = "Baud rate: {}".format(baud).center(center_val)

print()
print(title_string, end="\n\n")
print(port_string)
print(baud_string, end="\n\n")

ser = serial.Serial(port, baud)
time.sleep(2)  # wait for serial on the arduino to initialize
if action == '0':
    # get_input_and_act()
    read_file_and_act()
else:
    do_the_thing(action)
time.sleep(2)
ser.close()
