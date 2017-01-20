#!/usr/bin/python

import serial
from matplotlib import pyplot as plt

import re

ser = serial.Serial('/dev/ttyUSB0', 9600)

plt.ion() # set plot to animated

pose = [0, 0, 0]

plt.plot([pose[0]], [pose[1]], 'ro')
plt.ylim([-20,20])
plt.xlim([-20,20])

ack='\x00\x00\x00\x00\x00\x00\x00\x00'

while True:
    incoming = ser.readline().rstrip()
    print '%s' %incoming
    ser.write('%s' % ack)
    m = re.search(r"\[()\]", s)
    data = m.group(1).split(',')
    pose = (float(data[0]), float(data[1]), float(data[2]))
    plt.draw() # update the plot

ser.close()
