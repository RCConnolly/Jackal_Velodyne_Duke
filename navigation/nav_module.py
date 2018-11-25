import rospy

from move_base_msgs.msg import MoveBaseGoal
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler


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


def createMoveBaseGoal(x, y, yaw, frame):
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
