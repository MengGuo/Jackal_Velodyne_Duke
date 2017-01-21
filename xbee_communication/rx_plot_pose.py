#!/usr/bin/python

import serial

from math import pi as PI
from math import atan2, sin, cos, sqrt

import matplotlib.patches
from matplotlib import pyplot

import matplotlib
from matplotlib.patches import Polygon

import re

def norm(pose1, pose2):
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)

def transform(car, center, dl):
    new_car = []
    for node in car:
        vec = [node[0]-center[0], node[1]-center[1]]
        new_x = vec[0]*cos(dl) - vec[1]*sin(dl)
        new_y = vec[0]*sin(dl) + vec[1]*cos(dl)
        new_car.append([new_x+center[0], new_y+center[1]])
    return new_car

def visualize_jackal(figure, pose):
    pyplot.cla()
    fig = figure
    ax = fig.add_subplot(111)
    if pose:
        xl = pose[0]
        yl = pose[1]
        dl = pose[2]
        ax.plot(xl, yl, 'ro', markersize=8)
        L1 = 0.4
        L2 = 0.8
        car=[(xl-L1,yl-L1), (xl-L1,yl+ L1), (xl, yl+L2), (xl+L1, yl+L1), (xl+L1,yl-L1)]
        Ecolor = 'Grey'            
        polygon2 = Polygon(transform(car, [xl,yl], dl), fill = True, facecolor=Ecolor, edgecolor='black', lw=4, zorder=2)
        ax.add_patch(polygon2)    
    ax.grid()
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    ax.set_aspect('equal')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    #fig.subplots_adjust(0.003,0.062,0.97,0.94)
    #pyplot.show()
    pyplot.pause(0.01)
    return fig


#==============================
#==============================

ser = serial.Serial('/dev/ttyUSB0', 9600)

figure = pyplot.figure()
pyplot.ion()
pyplot.draw()

pose = [0,0,0]
figure = visualize_jackal(figure, pose)

ack='\x00\x00\x00\x00\x00\x00\x00\x00'

while True:
    incoming = ser.readline().rstrip()
    print r'%s' %incoming
    print list(incoming)
    ser.write('%s' %ack)
    m = re.search(r"\[(.*)\]", incoming)
    data = m.group(1).split(',')
    pose = (float(data[0]), float(data[1]), float(data[2]))
    print 'new pose %s' %str(pose)
    figure = visualize_jackal(figure, pose)

ser.close()
