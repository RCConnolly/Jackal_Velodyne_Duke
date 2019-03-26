#!/usr/bin/env python
# license removed for brevity
import rospy

from nav_module import findNearestObject, Goal2D
from move_base_client import MoveBaseClient

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        mb_client = MoveBaseClient()

        # Iterate over goals
        tst_right = Goal2D(-3.0, -.75, 3.14, 'map')
        tst_left = Goal2D(-6.0, -2.0, 3.14, 'map')
        tst_up = Goal2D(-8.2, -1.0, 1.6, 'map')
        tst_left2 = Goal2D(-6.0, 1.5, 0.0, 'map')
        goals = [tst_right, tst_left]
        for goal in goals:
            # Move to a target location near wall
            result = mb_client.send_goal(goal.to_move_base())
            if result:
                rospy.loginfo("Goal execution done!")

            # Rotate toward wall
            (obj_distance, obj_angle) = findNearestObject()
            turn_goal = Goal2D(0, 0, obj_angle, 'front_laser')
            turn_res = mb_client.send_goal(turn_goal.to_move_base())
            if turn_res:
                rospy.loginfo("Successfully turned toward wall!")

            # Drive to wall
            wall_separation = 0.012
            dist_to_robo_front = 0.21
            laser_range_acc = 0.03
            goal_distance = (obj_distance - wall_separation -
                             dist_to_robo_front - laser_range_acc)
            driver.move(goal_distance)

            # Perform task
            rospy.loginfo("TODO - Perform listen/speak task... for now pausing 2s")
            rospy.sleep(2)

            # Reverse 0.5 m from wall
            driver.move(0.5, -0.2)
            
        rospy.loginfo('Finished goal sequence')
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
