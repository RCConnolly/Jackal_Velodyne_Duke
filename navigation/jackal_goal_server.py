#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatus
from move_base_client import MoveBaseClient
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionResult
from odom_drive_to_wall import DriveStraight
from nav_module import findNearestObject, Goal2D
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler


def SendInitialPose(InitialPosePublisher, initial_pose):
    # goal: [x, y, yaw]
    InitialPoseMsg = PoseWithCovarianceStamped()
    # InitialPoseMsg.header.seq = 0
    InitialPoseMsg.header.stamp = rospy.Time.now()
    InitialPoseMsg.header.frame_id = 'map'
    InitialPoseMsg.pose.pose.position.x = initial_pose[0]
    InitialPoseMsg.pose.pose.position.y = initial_pose[1]
    # InitialPoseMsg.pose.position.z = 0.0
    quaternion = quaternion_from_euler(0, 0, initial_pose[2])
    InitialPoseMsg.pose.pose.orientation.x = quaternion[0]
    InitialPoseMsg.pose.pose.orientation.y = quaternion[1]
    InitialPoseMsg.pose.pose.orientation.z = quaternion[2]
    InitialPoseMsg.pose.pose.orientation.w = quaternion[3]
    InitialPosePublisher.publish(InitialPoseMsg)


class JackalGoalServer:
    def __init__(self, ns):
        self.ns = ns
        res_topic = ns + '/result'
        self.pub = rospy.Publisher(res_topic, Bool, queue_size=1)
        
    def goal_callback(self, goal):
        '''
        goal is of type MoveBaseActionGoal
        '''
        mb_client = MoveBaseClient()
        
        # Move to a target location near wall
        reached_goal = mb_client.send_goal(goal)
        if reached_goal:
            rospy.loginfo("Sucessfully reached target area.")
        else:
            rospy.logerr("{} unable to reach goal".format(self.ns))
            self.pub.publish(False)
        '''
        driver = DriveStraight()

        # Rotate toward wall
        (obj_distance, obj_angle) = findNearestObject()
        turn_goal = Goal2D(0, 0, obj_angle, 'front_laser')
        turn_res = mb_client.send_goal(turn_goal.to_move_base())
        if turn_res.status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Turned toward nearest sample")

        # Drive to wall
        wall_separation = 0.012
        dist_to_robo_front = 0.21
        laser_range_acc = 0.03
        dist_buff = 0.1
        goal_distance = (obj_distance - wall_separation -
                         dist_to_robo_front - laser_range_acc -
                         dist_buff)
        driver.move(goal_distance)
        rospy.loginfo("Drove toward nearest sample")
        '''

        self.pub.publish(reached_goal)


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        # Initialize node
        num_args = len(sys.argv)
        if(num_args < 2):
            print('Call syntax: jackal_goal_server.py <jackal_name> <x_init> <y_init> <yaw_init>')
            sys.exit()
	print('Args:')
	print(sys.argv)
	
	ns = sys.argv[1]
	if(num_args >= 5):
            x_init = float(sys.argv[2])
            y_init = float(sys.argv[3])
            yaw_init = float(sys.argv[4])
        else:
            x_init = 0
            y_init = 0
            yaw_init = 0

        node_name = ns + '_goal_server'
        rospy.init_node(node_name)
        rospy.loginfo('Created {} node'.format(node_name))

        # Set initial pose for amcl
        initial = [x_init, y_init, yaw_init]
	print(initial)
        rospy.loginfo('Setting initial pose to: {}'.format(initial))
        InitialPosePublisher = rospy.Publisher('initialpose',
                                               PoseWithCovarianceStamped,
                                               queue_size=100)
        for i in range(10):
            SendInitialPose(InitialPosePublisher, initial)
            rospy.sleep(0.1)

        # Create server
        goal_server = JackalGoalServer(ns)

        # Subscribe to goal topic & relay to goal server
        goal_topic = ns + '/goal'
        rospy.Subscriber(goal_topic, MoveBaseGoal,
                         callback=goal_server.goal_callback)
        rospy.loginfo('Subscribed to {}'.format(goal_topic))
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interruption at node {}".format(node_name))
