#!/usr/bin/env python
# license removed for brevity
import rospy

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


if __name__ == '__main__':
    try:
        rospy.init_node('initialize_pose')

        # Set initial pose for amcl
        initial = [0, 0, 0]
        InitialPosePublisher = rospy.Publisher('initialpose',
                                               PoseWithCovarianceStamped,
                                               queue_size=100)
        for i in range(10):
            SendInitialPose(InitialPosePublisher, initial)
            rospy.sleep(0.1)

        print("Set initial pose")
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupted node: initialize_pose")

