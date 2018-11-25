import rospy

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction


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

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        # Creates a new goal with the MoveBaseGoal constructor
        goal.target_pose.header.stamp = rospy.Time.now()

        # Sends the goal to the action server.
        client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        # Waits for the server to finish performing the action.
        wait = client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return client.get_result()
