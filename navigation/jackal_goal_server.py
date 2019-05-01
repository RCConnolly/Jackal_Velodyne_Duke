#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import Bool, String
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
        self.task = None
        self.turn_goal = None
        
        self.res_topic = ns + '/result'
        self.res_pub = rospy.Publisher(self.res_topic, Bool, queue_size=1)

        self.turn_topic = '/turn_goal'
        self.turn_pub = rospy.Publisher(self.turn_topic, MoveBaseGoal, queue_size=1)

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
            self.task = None
            self.turn_goal = None
            self.res_pub.publish(False)
            return

        # Rotate toward sample
        if(self.task == 'listen'):
            (obj_distance, obj_angle) = findNearestObject()
            turn_goal = Goal2D(0, 0, obj_angle, 'front_laser').to_move_base()
            #TODO - instead of relative angle, convert to a shared frame such as the map
            self.turn_pub.publish(turn_goal)
        elif(self.task == 'speak'):
            if(self.turn_goal is None):
                rospy.loginfo('{} waiting for turning goal'.format(self.ns))
                turn_goal = rospy.wait_for_message(self.turn_topic,
                                                   MoveBaseGoal,
                                                   timeout=10)
            else:
                turn_goal = self.turn_goal
        else:
            rospy.loginfo("Didn't receive task to perform, continuing to next goal.")
            self.task = None
            self.turn_goal = None
            self.res_pub.publish(reached_goal)
            return
            
        turn_res = mb_client.send_goal(turn_goal)
        if(turn_res):
            rospy.loginfo("Turned toward nearest sample")

            # Drive to wall
            if(self.task == 'listen'):
                driver = DriveStraight()
                wall_separation = 0.012
                dist_to_robo_front = 0.21
                laser_range_acc = 0.03
                dist_buff = 0.1
                goal_distance = (obj_distance - dist_to_robo_front)
                driver.move(goal_distance)
                rospy.loginfo("Drove toward nearest sample")

        self.turn_goal = None
        self.res_pub.publish(turn_res)
        return

    def set_task(self, task):
        self.task = task.data
        rospy.loginfo("Setting {} task to {}".format(self.ns, task.data))
    
    def set_turn_goal(self, goal):
        if(self.turn_goal is None):
            self.turn_goal = goal
            rospy.loginfo("Setting turn goal for {}.".format(self.ns))
        else:
            rospy.loginfo("{} turn goal already set.".format(self.ns))

    def performTask(self, do_task):
        if(do_task):
            if(self.task is None):
                rospy.loginfo('No task for {} to perform'.format(self.ns))
            else:
                rospy.loginfo('{} performing {} task'.format(self.ns, self.task))
                if(self.task == 'listen'):
                    # TODO - implement data acquisition instead of sleeping
                    rospy.loginfo('listening...')
                    rospy.sleep(3.0)
                    # Reverse from wall 0.5m
                    driver = DriveStraight()
                    driver.move(0.5, -0.1)
                    
                elif(self.task == 'speak'):
                    # TODO - implement white noise playing
                    rospy.loginfo('speaking')
                    rospy.sleep(3.0)
        else:
            rospy.loginfo('{} not ready to perform task'.format(self.ns))
            
        return


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

        # Subscribe to task topic &  set jackal task on new message
        task_topic = ns + '/task'
        rospy.Subscriber(task_topic, String,
                         callback=goal_server.set_task)
        rospy.loginfo('Subscribed to {}'.format(task_topic))

        # Subscribe to task starting topic & set jackal task on new message
        do_task_topic = ns + '/do_task'
        rospy.Subscriber(do_task_topic, Bool,
                         callback=goal_server.performTask)
        rospy.loginfo('Subscribed to {}'.format(do_task_topic))

        # Subscribe to turn goal topic & set the jackal's turn goal
        rospy.Subscriber(goal_server.turn_topic, MoveBaseGoal,
                         callback=goal_server.set_turn_goal)
        rospy.loginfo('Subscribed to {}'.format(goal_server.turn_topic))

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS interruption at node {}".format(node_name))
