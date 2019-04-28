#!/usr/bin/env python

""" odom_drive_to_wall.py

    Drives a straight to a specified goal distance measured
    by the /odom frame.
"""

import rospy
import sys
from geometry_msgs.msg import Twist, Point
import tf
from math import sqrt, pow


class DriveStraight:
    def __init__(self):

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'
        
    def move(self, goal_distance, linear_speed=0.10):

        rospy.loginfo('Moving a distance of {} at speed {}'
                      .format(goal_distance, linear_speed))

        # Controls looping rate with rospy.sleep
        rate = 20
        r = rospy.Rate(rate)
        
        # Min/max velocity for jackal listed here:
        # https://github.com/jackal/jackal/blob/5eb9356c891a4168cfa98b4b42a55561b245ba81/jackal_navigation/params/base_local_planner_params.yaml
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed
                    
        # Get the starting position values
        position = Point()
        position = self.get_odom_pos()
        x_start = position.x
        y_start = position.y

        # Keep track of the distance traveled
        distance = 0
            
        # Enter the loop to move along a side
        while (distance < goal_distance) and (not rospy.is_shutdown()):
            # Publish the Twist message and sleep 1 cycle
            self.cmd_vel.publish(move_cmd)
            
            r.sleep()
        
            # Get the current position
            position = self.get_odom_pos()
            
            # Compute the Euclidean distance from the start
            distance = sqrt(pow((position.x - x_start), 2) +
                            pow((position.y - y_start), 2))

        if(distance >= goal_distance):
            rospy.loginfo('Finished moving')
            
        # Stop the robot
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
        
    def get_odom_pos(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return Point(*trans)
 
if __name__ == '__main__':
    try:
        rospy.init_node('drive_straight')
        if(len(sys.argv) < 2):
            print('Call syntax: ./odom_drive_to_wall.py <distance> <velocity>')
            sys.exit()
            
        dist = float(sys.argv[1])
        if(len(sys.argv) > 2):
            vel = float(sys.argv[2])
        else:
            vel = 0.1

        driver = DriveStraight()
        driver.move(dist,vel)
        
    except:
        rospy.loginfo("drive_straight node terminated.")
