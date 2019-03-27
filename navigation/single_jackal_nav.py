#!/usr/bin/env python

import rospy
import sys
from nav_module import Goal2D
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Bool

PI = 3.14

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        rospy.init_node('single_jackal_navigation')

        j1_name = sys.argv[1]
        j1_goal = j1_name + '/goal'
        j1_res = j1_name + '/result'
        j1_goal_pub = rospy.Publisher(j1_goal, MoveBaseGoal, queue_size=1)

        # Some test goals for simulation
        tst_right = Goal2D(-3.0, -.8, PI/2, 'map')
        tst_left = Goal2D(-6.0, -2.0, -PI/2, 'map')
        tst_up = Goal2D(-8.2, -1.0, 1.6, 'map')
        tst_left2 = Goal2D(-6.0, 1.5, 0.0, 'map')

        # Jackal acoustic IM goals
        Jackal1_goals = [tst_right, tst_left]
        Jackal1_tasks = ['listen', 'speak']
        num_goals = len(Jackal1_goals)

        rospy.loginfo('Pause while goal publisher becomes recognized...')
        rospy.sleep(5)
        rospy.loginfo('...finished pausing')

        for i in range(num_goals):
            # Move to goal
            rospy.loginfo('Sending goal to Jackal')
            j1_goal_pub.publish(Jackal1_goals[i].to_move_base())

            rospy.loginfo('Waiting for results from  Jackal')
            j1_reached_goal = rospy.wait_for_message(j1_res,Bool)

            # Perform task
            if(j1_reached_goal):
                rospy.loginfo('Jackal1 task: {}'.format(Jackal1_tasks[i]))
                rospy.sleep(3)
            else:
                rospy.logerr('Unable to reach a target goal.')
                sys.exit()

            # Reverse from wall?

        rospy.loginfo('Finished goal sequence.')
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interruption in single_jackal_navigation")
