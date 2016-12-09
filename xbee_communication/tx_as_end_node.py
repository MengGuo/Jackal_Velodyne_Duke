#! /usr/bin/python

# Import and init an XBee device
from xbee import XBee,ZigBee
import serial

import numpy as np
ser = serial.Serial('/dev/ttyUSB0', 9600)

# Use an XBee 802.15.4 device
# To use with an XBee ZigBee device, replace with:
#xbee = ZigBee(ser)

#xbee = XBee(ser)
xbee = ZigBee(ser)
N = 1000
vector = np.ones((N,N))

xbee.send('tx',
          dest_addr_long = '\x00\x00\x00\x00\x00\x00\x00\x00',
          dest_addr = '\x00\x00',
          data = '%s\r\n' %str(vector))

print(xbee.wait_read_frame())

ser.close()
