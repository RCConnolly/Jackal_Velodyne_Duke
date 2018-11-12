#!/usr/bin/env python
# license removed for brevity

import rospy

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from geometry_msgs.msg import Pose, Point, Quaternion

from sensor_msgs.msg import LaserScan

from tf.transformations import quaternion_from_euler

def getNearestObjectAngle():
    # returns the angle in sensor's frame to nearest object
    try:
        msg = rospy.wait_for_message('/front/scan', LaserScan, 10)
    except rospy.ROSException:
        rospy.loginfo("Timeout exceeded waiting for laser scan")

    if len(msg.ranges) < 1:
        return 0

    min_val = msg.ranges[0]
    for index, value in enumerate(msg.ranges):
        if value < min_val:
            min_val = value
            angle = msg.angle_min + (msg.angle_increment*index)

    return angle

def createMoveBaseGoal(x,y,yaw,frame):
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0

    quaternion = quaternion_from_euler(0, 0, yaw)
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    goal.target_pose.header.frame_id = frame


    return goal

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

def active_cb():
    rospy.loginfo("Goal pose is now being processed by the Move_Base Action Server...")

def feedback_cb(feedback):
    #To print current pose at each feedback:
    #rospy.loginfo("Feedback for goal pose: "+str(feedback))
    return

def done_cb(status, result):
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
    if status == GoalStatus.PREEMPTED:
        rospy.loginfo("Goal pose received a cancel request after it started executing, completed execution!")

    if status == GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal pose reached")
        multiple_goals = False
        if multiple_goals:
            # TODO implement for multiple goals
            # should make a call to client's send goal function
            print("TODO -- doing nothing")
        else:
            rospy.loginfo("Final goal pose reached!")
            return
            
    if status == GoalStatus.ABORTED:
        rospy.loginfo("Goal pose  was aborted by the Action Server")
        rospy.signal_shutdown("Goal pose aborted, shutting down!")
        return

    if status == GoalStatus.REJECTED:
        rospy.loginfo("Goal pose has been rejected by the Action Server")
        rospy.signal_shutdown("Goal pose rejected, shutting down!")
        return

    if status == GoalStatus.RECALLED:
        rospy.loginfo("Goal pose received a cancel request before it started executing, successfully cancelled!")


def movebase_client(goal):

    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal.target_pose.header.stamp = rospy.Time.now()

    # Sends the goal to the action server.
    client.send_goal(goal, done_cb, active_cb, feedback_cb)
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
        result = movebase_client(createMoveBaseGoal(-4,-1,3.1, "map"))
        if result:
            rospy.loginfo("Goal execution done!")

        closest_obj_angle = getNearestObjectAngle()
        turn_goal = createMoveBaseGoal(0,0,closest_obj_angle, "front_laser")
        turn_res = movebase_client(turn_goal)
        if turn_res:
            rospy.loginfo("Successfully turned toward wall!")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
