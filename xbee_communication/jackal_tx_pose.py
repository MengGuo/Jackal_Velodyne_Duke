#! /usr/bin/python

# ROS 
import roslib, rospy
roslib.load_manifest('jackal_velodyne_duke')
import sys
import time
from math import pi as PI
from math import atan2, sin, cos, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

# Xbee
from xbee import XBee,ZigBee
import serial


def PoseCallback(posedata):
    # PoseWithCovarianceStamped data from amcl_pose
    global robot_pose # [time, [x,y,yaw]]
    header = posedata.header
    pose = posedata.pose
    if (not robot_pose[0]) or (header.stamp > robot_pose[0]):
        # more recent pose data received
        robot_pose[0] = header.stamp
        # TODO: maybe add covariance check here?
        print('robot position update!')
        euler = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]) #roll, pitch, yaw
        robot_pose[1] = [pose.pose.position.x, pose.pose.position.y, euler[2]] # in radians
    return robot_pose


#========================================
global robot_pose
rospy.init_node('Euler_pose_to_xbee')

INITIAL = [10.0, 5.0, PI*0]
robot_pose = [None, INITIAL]
ser = serial.Serial('/dev/ttyUSB0', 9600)

# Use an XBee 802.15.4 device
# IMPORTANT! sudo chmod a+rw /dev/ttyUSB0
xbee = ZigBee(ser)


#----------
#subscribe to
#----------
rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, PoseCallback)

#----------
#main loop
#----------

t0 = rospy.Time.now()
while not rospy.is_shutdown():
    try:
        t = rospy.Time.now()-t0
        print '----------Time: %.2f----------' %t.to_sec()
        rospy.sleep(0.1)
        print 'robot pose %s' %str(robot_pose)
        xbee.send('tx',
          dest_addr_long = '\x00\x00\x00\x00\x00\x00\x00\x00',
          dest_addr = '\x00\x00',
          # for now, only send pose, no timestamp
          data = '%s\r\n' %str(robot_pose[1]))
        print 'robot pose %s sent via Xbee!' %str(robot_pose)
        print(xbee.wait_read_frame())
    except rospy.ROSInterruptException:
        pass
    
ser.close()    

