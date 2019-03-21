import rospy

from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseGoal
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


def findNearestObject():
    # returns the angle in sensor's frame to nearest object
    '''
    returns a tuple (distance, angle) of the distance and angle
    of the nearest object measured by latest LaserScan message
    on /front/scan topic
    '''
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

    return (min_val, angle)
