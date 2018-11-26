#!/usr/bin/env python
# license removed for brevity
import rospy

from nav_module import findNearestObject
from odom_drive_to_wall import DriveStraight
from move_base_client import MoveBaseClient, Goal2D
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


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        mb_client = MoveBaseClient()
        driver = DriveStraight()

        # Set initial pose for amcl
        initial = [0, 0, 0]
        InitialPosePublisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=100)
        for i in range(10):
            SendInitialPose(InitialPosePublisher, initial)
            rospy.sleep(0.1)

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

            # Reverse 0.5 m from wall
            rospy.loginfo("TODO - Perform listen/speak task... for now pausing 2s")
            rospy.sleep(2)
            driver.move(0.5, -0.2)
            
        rospy.loginfo('Finished goal sequence')
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
