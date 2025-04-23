#!/usr/bin/env python
import rospy
from map_switcher.srv import SwitchMap
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

def switch_and_reset(map_path, pose):
    rospy.wait_for_service('/change_map')
    switch_map = rospy.ServiceProxy('/change_map', SwitchMap)

    try:
        rospy.loginfo(f"Switching to map: {map_path}")
        res = switch_map(map_path)
        if res.success:
            rospy.loginfo("Map switched successfully. Resetting pose...")
            time.sleep(1)  # wait for map_server to load

            pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.pose.pose.position.x = pose[0]
            msg.pose.pose.position.y = pose[1]
            msg.pose.pose.orientation.w = 1.0
            pub.publish(msg)

        else:
            rospy.logerr("Map switch failed.")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('map_changer_test')
    switch_and_reset("/path/to/your/map.yaml", (0.0, 0.0))
