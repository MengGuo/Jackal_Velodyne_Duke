#! /usr/bin/python

# Import and init an XBee device
from xbee import XBee,ZigBee
import serial

ser = serial.Serial('/dev/ttyUSB0', 9600)

# Use an XBee 802.15.4 device
# To use with an XBee ZigBee device, replace with:
#xbee = ZigBee(ser)

#xbee = XBee(ser)
xbee = ZigBee(ser)

xbee.send('tx',
          dest_addr_long = '\x00\x00\x00\x00\x00\x00\x00\x00',
          dest_addr = '\x00\x00',
          data = 'Hello World\n')

print(xbee.wait_read_frame())

ser.close()
