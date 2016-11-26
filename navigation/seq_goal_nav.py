#!/usr/bin/env python
import roslib, rospy
roslib.load_manifest('jackal_velodyne_duke')
import sys

import time

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

from math import pi as PI
from math import atan2, sin, cos, sqrt

from tf.transformations import euler_from_quaternion, quaternion_from_euler


def norm2(pose1, pose2):
    # 2nd norm distance
    return sqrt((pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2)


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


def SendGoal(GoalPublisher, goal, time_stamp):
    # goal: [x, y, yaw]
    GoalMsg = PoseStamped()
    #GoalMsg.header.seq = 0
    GoalMsg.header.stamp = time_stamp
    GoalMsg.header.frame_id = 'map'
    GoalMsg.pose.position.x = goal[0]
    GoalMsg.pose.position.y = goal[1]
    #GoalMsg.pose.position.z = 0.0
    quaternion = quaternion_from_euler(0, 0, goal[2])
    GoalMsg.pose.orientation.x = quaternion[0]
    GoalMsg.pose.orientation.y = quaternion[1]
    GoalMsg.pose.orientation.z = quaternion[2]
    GoalMsg.pose.orientation.w = quaternion[3]
    GoalPublisher.publish(GoalMsg)


def SendInitialPose(InitialPosePublisher, initial_pose, time_stamp):
    # goal: [x, y, yaw]
    InitialPoseMsg = PoseWithCovarianceStamped()
    #InitialPoseMsg.header.seq = 0
    InitialPoseMsg.header.stamp = time_stamp
    InitialPoseMsg.header.frame_id = 'map'
    InitialPoseMsg.pose.pose.position.x = initial_pose[0]
    InitialPoseMsg.pose.pose.position.y = initial_pose[1]
    #InitialPoseMsg.pose.position.z = 0.0
    quaternion = quaternion_from_euler(0, 0, initial_pose[2])
    InitialPoseMsg.pose.pose.orientation.x = quaternion[0]
    InitialPoseMsg.pose.pose.orientation.y = quaternion[1]
    InitialPoseMsg.pose.pose.orientation.z = quaternion[2]
    InitialPoseMsg.pose.pose.orientation.w = quaternion[3]
    InitialPosePublisher.publish(InitialPoseMsg)    


#========================================
global robot_pose
rospy.init_node('navigate_goal_sequence')


INITIAL = [-3.1147847748884683, -1.1183813706038452, 0.7960764716766603]
GOAL = [[6.80652717383558, 8.011186314690564, 0.7987732650786219],[-3.1147847748884683, -1.1183813706038452, 0.7960764716766603]]

robot_pose = [None, INITIAL]
#----------
#publish to
#----------
InitialPosePublisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 100)

for i in xrange(10):
    SendInitialPose(InitialPosePublisher, INITIAL, rospy.Time.now())
    rospy.sleep(0.1)
    
print('Initial pose set to %s.' %INITIAL)

GoalPublisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size = 100)
#----------
#subscribe to
#----------
rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, PoseCallback)

#----------
#main loop
#----------
t0 = rospy.Time.now()
k = 0
reach_xy_bound = 0.5 # m
reach_yaw_bound = 0.1*PI # rad
while not rospy.is_shutdown():
    try:
        t = rospy.Time.now()-t0
        print '----------Time: %.2f----------' %t.to_sec()
        while (k <= len(GOAL) -1) and (not rospy.is_shutdown()):
            current_goal = GOAL[k]
            if ((norm2(robot_pose[1][0:2], current_goal[0:2]) > reach_xy_bound) or (abs(robot_pose[1][2])-current_goal[2]) > reach_yaw_bound):
                SendGoal(GoalPublisher, current_goal, t)
                print('Goal %s sent.' %(str(current_goal)))
                rospy.sleep(10)
            else:
                print('Goal %s reached.' %(str(current_goal)))
                k += 1
        print('Goal sequence finished.')
        break
    except rospy.ROSInterruptException:
        pass
      
