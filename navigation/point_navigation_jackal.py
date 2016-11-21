#!/usr/bin/env python
import roslib, rospy
roslib.load_manifest('Jackal_Velodyne_Duke')
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
    if header.stamp > robot_pose[0]:
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
    GoalMsg.header.frame_id = '1'
    GoalMsg.pose.x = goal[0]
    GoalMsg.pose.y = goal[1]
    #GoalMsg.pose.z = 0.0
    quaternion = quaternion_from_euler(0, 0, goal[2])
    GoalMsg.orientation.x = quaternion[0]
    GoalMsg.orientation.y = quaternion[1]
    GoalMsg.orientation.z = quaternion[2]
    GoalMsg.orientation.w = quaternion[3]
    GoalPublisher.publish(GoalMsg)


def SendInitialPose(InitialPosePublisher, initial_pose, time_stamp):
    # goal: [x, y, yaw]
    InitialPoseMsg = PoseStamped()
    #InitialPoseMsg.header.seq = 0
    InitialPoseMsg.header.stamp = time_stamp
    InitialPoseMsg.header.frame_id = '1'
    InitialPoseMsg.pose.x = initial_pose[0]
    InitialPoseMsg.pose.y = initial_pose[1]
    #InitialPoseMsg.pose.z = 0.0
    quaternion = quaternion_from_euler(0, 0, initial_pose[2])
    InitialPoseMsg.orientation.x = quaternion[0]
    InitialPoseMsg.orientation.y = quaternion[1]
    InitialPoseMsg.orientation.z = quaternion[2]
    InitialPoseMsg.orientation.w = quaternion[3]
    InitialPosePublisher.publish(InitialPoseMsg)    


#========================================
global robot_pose
rospy.init_node('navigate_goal_sequence')


INITIAL = [10.0, 5.0, PI*0]
GOAL = [(1.0, 1.0, PI*0.5), (0.5, 0.6, PI*0.2), (1.5, 0.9, PI*0.8), (1.2, 0.2, -PI*0.5)]

robot_pose = [0, INITIAL]
#----------
#publish to
#----------
InitialPosePublisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 100)

SendInitialPose(InitialPosePublisher, INITIAL, 0)
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
        while (k <= len(GOAL) -1):
            current_goal = GOAL[k]
            if ((norm2(robot_pose[1][0:2], goal[k][0:2]) > reach_xy_bound) or (abs(robot_pose[1][2])-goal[k][2]) > reach_yaw_bound):
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
      
