import rospy

from sensor_msgs.msg import LaserScan

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
