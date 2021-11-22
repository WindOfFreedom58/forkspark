import rospy
import time
import tf2_ros

import tf_conversions

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


def handle_spark_pose(msg, frames):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = frames[0]
    t.child_frame_id = frames[1]

    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation = msg.pose.pose.orientation

    br.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("base_link_tf", anonymous=True)
    global_frame = rospy.get_param("/global_frame")
    base_frame = rospy.get_param("base_frame")

    odom_subscriber = rospy.Subscriber("/odom", Odometry, handle_spark_pose, (global_frame, base_frame))
    rospy.spin()
