import rospy

import sys

import tf_conversions

import tf2_ros
import geometry_msgs.msg

if __name__ == "__main__":
    if (len(sys.argv) < 2):
        rospy.logerr(
            "Invalid number of args\nUsage: ./static_broadcaster.py tf_param_name")
        sys.exit(0)
    else:
        # Get base frame name from parameter server
        base_frame = rospy.get_param("base_frame")

        tf_param_name = sys.argv[1]
        tf_param = rospy.get_param(tf_param_name)

        if tf_param["frame_name"] == base_frame:
            rospy.logerr(
                "Static frame name cannot be the same name with base frame.")
            sys.exit(0)

        rospy.init_node(tf_param_name)
        br = tf2_ros.StaticTransformBroadcaster()

        tf_msg = geometry_msgs.msg.TransformStamped()

        tf_msg.header.stamp = rospy.Time.now()
        tf_msg.header.frame_id = base_frame
        tf_msg.child_frame_id = tf_param["frame_name"]

        tf_msg.transform.translation.x = tf_param["translation"]["x"]
        tf_msg.transform.translation.y = tf_param["translation"]["y"]
        tf_msg.transform.translation.z = tf_param["translation"]["z"]

        quat = tf_conversions.transformations.quaternion_from_euler(
            tf_param["rotation"]["roll"], tf_param["rotation"]["pitch"], tf_param["rotation"]["yaw"])
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        br.sendTransform(tf_msg)
        rospy.spin()
