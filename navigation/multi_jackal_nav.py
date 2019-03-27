#!/usr/bin/env python

import rospy
import sys
from nav_module import Goal2D
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        rospy.init_node('multi_jackal_navigation')

        j1_name = sys.argv[1]
        j1_goal = j1_name + '/goal'
        j1_res = j1_name + '/result'
        j1_goal_pub = rospy.Publisher(j1_goal, MoveBaseActionGoal)

        j2_name = sys.argv[2]
        j2_goal = j2_name + '/goal'
        j2_res = j2_name + '/result'
        j2_goal_pub = rospy.Publisher(j2_goal, MoveBaseActionGoal)

        # Some test goals for simulation
        tst_right = Goal2D(-3.0, -.75, 3.14, 'map')
        tst_left = Goal2D(-6.0, -2.0, 3.14, 'map')
        tst_up = Goal2D(-8.2, -1.0, 1.6, 'map')
        tst_left2 = Goal2D(-6.0, 1.5, 0.0, 'map')

        # Jackal acoustic IM goals
        Jackal1_goals = [tst_right, tst_left]
        Jackal2_goals = [tst_up, tst_left2]
        Jackal1_tasks = ['listen', 'speak']
        Jackal2_tasks = ['speak', 'listen']
        assert(len(Jackal1_goals) == len(Jackal2_goals))
        num_goals = len(Jackal1_goals)

        for i in range(num_goals):
            # Move to goal
            rospy.loginfo('Sending goals to each Jackal')
            j1_goal_pub(Jackal1_goals[i].to_move_base())
            j2_goal_pub(Jackal2_goals[i].to_move_base())

            rospy.loginfo('Waiting for results from both Jackals')
            j1_res = rospy.wait_for_message(j1_res, MoveBaseActionResult)
            j2_res = rospy.wait_for_message(j2_res, MoveBaseActionResult)

            # Perform task
            if((j1_res.status == GoalStatus.SUCCEEDED) and
               (j2_res.status == GoalStatus.SUCCEEDED)):
                rospy.loginfo('Performing tasks...')
                rospy.sleep(3)
            else:
                rospy.logerr('Unable to reach a target goal.')
                sys.exit()
                
            # Reverse from wall?

        rospy.loginfo('Finished goal sequence.')
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interruption in multi_jackal_navigation")
