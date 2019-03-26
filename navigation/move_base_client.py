import rospy
import sys

import actionlib
from actionlib_msgs.msg import GoalStatus
from odom_drive_to_wall import DriveStraight
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult


class MoveBaseClient:
    def __init__(self, ns, pub):
        self.ns = ns
        res_topic = ns + '/result'
        self.pub = rospy.Publisher(res_topic, MoveBaseActionResult, queue_size=1)
        self.driver = DriveStraight()


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
        reached_goal = client.wait_for_result()

        # If the result doesn't arrive, assume the Server is not available
        if not reached_goal:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            res = client.get_result()
            self.pub.publish(res)
            return res

if __name__ == '__main__':
    try:
        ns = sys.argv[1]
        node_name = ns + '_move_base_client'
        rospy.init_node(node_name)

        mb_client = MoveBaseClient(ns)
        goal_name = ns + '/goal'
        rospy.Subscriber(goal_name, MoveBaseActionGoal,
                         callback=mb_client.send_goal)
        rospy.spin()
        
    except rospy.ROSInterruptException:
        print(ns + " MoveBaseClient interrupted", file=sys.stderr)

