#!/usr/bin/env python

""" odom_drive_to_wall.py

    Drives a forward to a specified goal distance determined
    by the distance to a nearest object measured by a laser
    scanner.
"""

import rospy
from geometry_msgs.msg import Twist, Point
import tf
from math import sqrt, pow


class DriveForward:
    def __init__(self):

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        self.odom_frame = '/odom'
        self.base_frame = '/base_link'
        
    def move(self, dist=1.0, speed=0.15):

        # Controls looping rate with rospy.sleep
        rate = 20
        r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0.15 meters per second
        # Min/max velocity for jackal listed here:
        # https://github.com/jackal/jackal/blob/5eb9356c891a4168cfa98b4b42a55561b245ba81/jackal_navigation/params/base_local_planner_params.yaml
        linear_speed = speed
        
        # Set the travel distance in meters
        goal_distance = dist
        
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
        while distance < goal_distance and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(move_cmd)
            
            r.sleep()
        
            # Get the current position
            position = self.get_odom_pos()
            
            # Compute the Euclidean distance from the start
            distance = sqrt(pow((position.x - x_start), 2) +
                            pow((position.y - y_start), 2))

        if(distance >= goal_distance):
            rospy.loginfo('Moved goal distance!')
            
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
        OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")

