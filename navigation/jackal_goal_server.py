#! /usr/bin/env python

import rospy
import sys
from nav_module import findNearestObject, Goal2D
from actionlib_msgs.msg import GoalStatus
from move_base_client import MoveBaseClient, MoveBaseActionGoal, MoveBaseActionResult
from odom_drive_to_wall import DriveStraight


class JackalGoalServer:
    def __init__(self, ns):
        self.ns = ns
        res_topic = ns + '/result'
        self.pub = rospy.Publisher(res_topic, MoveBaseActionResult,
                                   queue_size=1)
        
    def goal_callback(self, goal):
        '''
        goal is of type MoveBaseActionGoal
        '''
        mb_client = MoveBaseClient
        driver = DriveStraight
        
        # Move to a target location near wall
        act_res = mb_client.send_goal(goal)
        if act_res.status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Sucessfully reached target area")

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

        self.pub.publish(act_res)

        

        
# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        # Initialize node
        ns = sys.argv[1]
        node_name = ns + '_goal_server'
        rospy.init_node(node_name)

        # Subscribe to goal topic
        goal_server = JackalGoalServer(ns)
        goal_topic = ns + '/goal'
        rospy.Subscriber(goal_topic, MoveBaseActionGoal,
                         callback=goal_server.goal_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
