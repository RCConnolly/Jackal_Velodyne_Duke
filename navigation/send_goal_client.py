#!/usr/bin/env python
# license removed for brevity

import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def SendInitialPose(InitialPosePublisher, initial_pose):
    # goal: [x, y, yaw]
    InitialPoseMsg = PoseWithCovarianceStamped()
    #InitialPoseMsg.header.seq = 0
    InitialPoseMsg.header.stamp = rospy.Time.now()
    InitialPoseMsg.header.frame_id = 'map'
    InitialPoseMsg.pose.pose.position.x = initial_pose[0]
    InitialPoseMsg.pose.pose.position.y = initial_pose[1]
    #InitialPoseMsg.pose.position.z = 0.0
    quaternion = quaternion_from_euler(0, 0, initial_pose[2])
    InitialPoseMsg.pose.pose.orientation.x = quaternion[0]
    InitialPoseMsg.pose.pose.orientation.y = quaternion[1]
    InitialPoseMsg.pose.pose.orientation.z = quaternion[2]
    InitialPoseMsg.pose.pose.orientation.w = quaternion[3]
    InitialPosePublisher.publish(InitialPoseMsg)

def movebase_client():

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = -5
    goal.target_pose.pose.position.y = 3.3
   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        initial = [0,0,0]
        InitialPosePublisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 100)
        for i in range(10):
            SendInitialPose(InitialPosePublisher, initial)
            rospy.sleep(0.1)
        # goal = [-5, 3.3, 0]
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
