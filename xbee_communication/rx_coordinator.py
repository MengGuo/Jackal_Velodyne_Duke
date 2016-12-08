#!/usr/bin/python

import serial

ser = serial.Serial('/dev/ttyUSB0', 9600)


ack='\x00\x00\x00\x00\x00\x00\x00\x00'

while True:
    incoming = ser.readline().strip()
    print '%s' %incoming
    ser.write('%s' % ack)

ser.close()
