#!/usr/bin/env python
import rospy
import tf

def publish_base_link():
    rospy.init_node('base_link_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        br.sendTransform(
            (0.0, 0.0, -0.05),  # Translation: x, y, z
            tf.transformations.quaternion_from_euler(0, 0, 0),  # Rotation: roll, pitch, yaw
            rospy.Time.now(),
            "base_link",
            "base_footprint"
        )
        rate.sleep()

if __name__ == "__main__":
    publish_base_link()
