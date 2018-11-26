import rospy

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from tf.transformations import quaternion_from_euler


class Goal2D:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, frame='base_link'):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.frame = frame

    def to_move_base(self):
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.position.z = 0

        quaternion = quaternion_from_euler(0, 0, self.yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        goal.target_pose.header.frame_id = self.frame

        return goal


class MoveBaseClient:
    def __init__(self):
        self.multiple_goals = False

    def active_cb(self):
        rospy.loginfo("Goal pose is now being processed by the Move_Base Action Server...")

    def feedback_cb(self, feedback):
        # To print current pose at each feedback:
        # rospy.loginfo("Feedback for goal pose: "+str(feedback))
        return

    def done_cb(self, status, result):
        # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == GoalStatus.PREEMPTED:
            rospy.loginfo("Goal pose received a cancel request after it started executing, completed execution!")

        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal pose reached")
            if self.multiple_goals:
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

    def send_goal(self, goal):

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal.target_pose.header.stamp = rospy.Time.now()

        client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        wait = client.wait_for_result()

        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return client.get_result()
