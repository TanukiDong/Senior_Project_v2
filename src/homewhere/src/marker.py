#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped

# ────────────── parameters you might tweak ──────────────
FRAME_ID = "map"
OFFSET = (0.5, -1.0, 1.0)
TEXT_H = 0.25
THICK = 0.01
TOPIC = "robot_xy_marker"
BG_W = 0.25
BG_H = 1.25
# ─────────────────────────────────────────────────────────

def make_cube(ns, id):
    m = Marker()
    m.ns, m.id, m.type, m.action = ns, id, Marker.CUBE, Marker.ADD
    m.color.r = m.color.g = m.color.b = 1.0
    m.color.a = 0.9
    m.pose.orientation.w = 1.0
    return m

def make_text(ns, id):
    m = Marker()
    m.ns, m.id, m.type, m.action = ns, id, Marker.TEXT_VIEW_FACING, Marker.ADD
    m.scale.z = TEXT_H
    m.color.r = m.color.g = m.color.b = 0.0
    m.color.a = 1.0
    m.pose.orientation.w = 1.0
    return m

def cb(msg):
    rx = msg.pose.pose.position.x
    ry = msg.pose.pose.position.y

    ox, oy, oz = OFFSET
    cx, cy, cz = rx + ox, ry + oy, oz

    label = "(%.2f , %.2f)" % (rx, ry)

    hdr = msg.header
    hdr.frame_id = FRAME_ID
    hdr.stamp = rospy.Time.now()

    bg = make_cube("xy_bg", 0)
    bg.header = hdr
    bg.scale.x = BG_W
    bg.scale.y = BG_H
    bg.scale.z = THICK
    bg.pose.position.x = cx
    bg.pose.position.y = cy
    bg.pose.position.z = cz + THICK / 2.0

    fg = make_text("xy_fg", 0)
    fg.header = hdr
    fg.text = label
    fg.pose.position.x = cx
    fg.pose.position.y = cy
    fg.pose.position.z = cz + THICK

    pub.publish(bg)
    pub.publish(fg)

# ───────────── Main ROS Init ─────────────
rospy.init_node("xy_label_node")
pub = rospy.Publisher(TOPIC, Marker, queue_size=2)

# ✅ CORRECT TYPE: PoseWithCovarianceStamped
rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, cb)

rospy.spin()
