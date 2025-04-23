#!/usr/bin/env python
"""multilevel_nav_manager.py  –  slope-driven blind ramp traversal with restart 2025-04-24

Modified 2025‑04‑23
• Add 1‑s forward "finish" drive after exiting ramp
• Republish goal transformed into new‑map frame so robot heads to correct floor
"""

import rospy
import yaml
import os
import sys
import subprocess
import math
import roslib.packages
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from actionlib_msgs.msg import GoalID

# ──── State labels ─────────────────────────────────────────────
STATE_IDLE        = 0
STATE_NORMAL_NAV  = 1
STATE_TO_RAMP     = 2
STATE_ENTER_RAMP  = 3
STATE_UP_RAMP     = 4
STATE_MAP_SWITCH  = 5

DIST_TO_RAMP_TH       = 0.2   # meters
RAMP_SPEED            = 1.00  # m/s
RAMP_RATE             = 20    # Hz
FINISH_DRIVE_DURATION = 1.0   # sec of straight drive after slope ↓ edge

class MultiLevelNavManager:
    def __init__(self):
        rospy.init_node("multilevel_nav_manager")

        # ── params ───────────────────────────────────────────────
        self.start_room  = str(rospy.get_param("~start_room", "1"))
        ramp_cfg         = rospy.get_param("~ramp_config")
        self.map_table   = rospy.get_param("~map_table", {})
        if not self.map_table:
            rospy.logerr("Empty map_table – include bounds in your YAML!")
            sys.exit(1)

        with open(ramp_cfg, "r") as f:
            ramps = yaml.safe_load(f)["ramps"]
        self.ramp_table = {(r["from"], r["to"]): r for r in ramps}

        # ── runtime vars ────────────────────────────────────────
        self.current_room  = self.start_room
        self.state         = STATE_IDLE
        self.goal_original = None
        self.active_ramp   = None
        self.last_amcl_xy  = None
        self.blind_timer   = None
        self.warmup_timer  = None
        self.finish_timer  = None
        self.warmup_done   = False
        self.on_slope      = False

        # ── pubs / subs ─────────────────────────────────────────
        self.cmd_pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.init_pub = rospy.Publisher(
            "/initialpose",
            PoseWithCovarianceStamped,
            queue_size=1,
            latch=True,
        )

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_cb)
        rospy.Subscriber("/on_slope", Bool, self.slope_cb)

        self.state = STATE_NORMAL_NAV
        rospy.loginfo("Started in room %s", self.start_room)
        rospy.spin()

    # ───────────────── Callbacks ───────────────────────────────────
    def goal_cb(self, msg: PoseStamped):
        self.goal_original = msg
        desired = self.determine_room(msg)

        # only treat as normal if not already en route to ramp
        if desired == self.current_room and self.state != STATE_TO_RAMP:
            rospy.loginfo("Normal navigation in room %s", self.current_room)
            self.state = STATE_NORMAL_NAV
            return

        # need ramp
        self.cancel_move_base()
        key = (self.current_room, desired)
        if key not in self.ramp_table:
            rospy.logerr("No ramp defined for %s → %s", *key)
            return

        self.active_ramp = self.ramp_table[key]
        ex, ey, _ = self.active_ramp["entry_pose"]
        rospy.loginfo("STATE_TO_RAMP: heading to (%.2f,%.2f)", ex, ey)
        self.send_intermediate_goal(self.active_ramp["entry_pose"])
        self.state = STATE_TO_RAMP

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.last_amcl_xy = (x, y)

        # once we're close enough, advance to ENTER_RAMP
        if self.state == STATE_TO_RAMP and self.active_ramp:
            ex, ey, _ = self.active_ramp["entry_pose"]
            d = math.hypot(x - ex, y - ey)
            rospy.loginfo("Dist to entry: %.2f (th=%.2f)", d, DIST_TO_RAMP_TH)
            if d < DIST_TO_RAMP_TH:
                rospy.loginfo("Reached entry → STATE_ENTER_RAMP")
                self.state = STATE_ENTER_RAMP
                self.cancel_move_base()

    def slope_cb(self, msg: Bool):
        """Called for every /on_slope message."""
        self.on_slope = msg.data  # remember last value

        # ────────────────────────────────────────────
        # ①  ENTER the ramp: start both timers
        # ────────────────────────────────────────────
        if self.state == STATE_ENTER_RAMP:
            rospy.loginfo("ENTER_RAMP → blind warm‑up")
            subprocess.call(["rosnode", "kill", "/move_base", "/amcl"])

            self.state        = STATE_UP_RAMP
            self.warmup_done  = False

            # (a) start continuous blind drive
            self.blind_timer = rospy.Timer(
                rospy.Duration(1.0 / RAMP_RATE),
                self._publish_blind_twist,
                oneshot=False,
            )

            # (b) start *one‑shot* 1‑second warm‑up timer
            self.warmup_timer = rospy.Timer(
                rospy.Duration(1.0),
                self._end_warmup,
                oneshot=True,
            )
            return  # ignore current /on_slope

        # ────────────────────────────────────────────
        # ②  AFTER warm‑up: keep driving *only* while on_slope==True
        # ────────────────────────────────────────────
        if self.state == STATE_UP_RAMP and self.warmup_done:
            # falling edge – slope finished
            if not self.on_slope and self.finish_timer is None:
                rospy.loginfo("Slope ↓ → finish‑drive for %.1fs", FINISH_DRIVE_DURATION)
                # keep blind_timer running, schedule graceful stop
                self.finish_timer = rospy.Timer(
                    rospy.Duration(FINISH_DRIVE_DURATION),
                    self._end_finish_drive,
                    oneshot=True,
                )

    # ─────────────────── Helpers ─────────────────────────────────
    def determine_room(self, pose_msg: PoseStamped):
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        for room_id, info in self.map_table.items():
            b = info.get("bounds")  # [xmin,xmax,ymin,ymax]
            if b and len(b) == 4:
                if b[0] <= x <= b[1] and b[2] <= y <= b[3]:
                    rospy.loginfo("Goal (%.2f,%.2f) → room %s", x, y, room_id)
                    return room_id
        rospy.logwarn(
            "Goal (%.2f,%.2f) not in any bounds; staying in %s", x, y, self.current_room
        )
        return self.current_room

    def send_intermediate_goal(self, xyzr):
        goal = PoseStamped()
        goal.header.stamp    = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = xyzr[0]
        goal.pose.position.y = xyzr[1]
        q = quaternion_from_euler(0, 0, math.radians(xyzr[2]))
        (
            goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w,
        ) = q

        pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
        rospy.sleep(0.05)
        pub.publish(goal)

    def cancel_move_base(self):
        pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        rospy.sleep(0.05)
        pub.publish(GoalID())
        rospy.loginfo("move_base cancel sent")

    # ───────── timers ──────────────────────────────
    def _end_warmup(self, _evt):
        self.warmup_done = True
        rospy.loginfo("Warm‑up complete")

        # Edge‑case: sensor never went HIGH
        if not self.on_slope and self.state == STATE_UP_RAMP and self.finish_timer is None:
            self._end_finish_drive(None)

    def _publish_blind_twist(self, _event):
        twist = Twist()
        twist.linear.x = RAMP_SPEED
        self.cmd_pub.publish(twist)

    def _end_finish_drive(self, _evt):
        # Called once when FINISH_DRIVE_DURATION elapses
        if self.finish_timer:
            self.finish_timer.shutdown()
            self.finish_timer = None
        self._stop_blind_drive()

    def _stop_blind_drive(self):
        # Cancel ongoing timers
        if self.blind_timer:
            self.blind_timer.shutdown()
            self.blind_timer = None
        if self.warmup_timer:
            self.warmup_timer.shutdown()
            self.warmup_timer = None

        self.cmd_pub.publish(Twist())  # zero velocity
        self.state = STATE_NORMAL_NAV
        self.switch_map()

    # ───────── map switching & goal reprojection ─────────────────
    def _reproject_goal_to_new_map(self, goal_ps):
        """Return goal_pose expressed in new‑map frame using entry→exit offset."""
        if not self.active_ramp:
            return goal_ps  # fallback

        # Extract planar delta relative to entry_pose in old map
        gx, gy = goal_ps.pose.position.x, goal_ps.pose.position.y
        ex_in, ey_in, _ = self.active_ramp["entry_pose"]
        dx, dy = gx - ex_in, gy - ey_in

        ex_out, ey_out, _ = self.active_ramp["exit_pose"]
        new_goal            = PoseStamped()
        new_goal.header.stamp    = rospy.Time.now()
        new_goal.header.frame_id = "map"
        new_goal.pose.position.x = ex_out + dx
        new_goal.pose.position.y = ey_out + dy
        new_goal.pose.position.z = 0.0

        # Copy orientation (yaw only) from original
        q_in = goal_ps.pose.orientation
        _, _, yaw = euler_from_quaternion([q_in.x, q_in.y, q_in.z, q_in.w])
        q_out = quaternion_from_euler(0, 0, yaw)
        (
            new_goal.pose.orientation.x,
            new_goal.pose.orientation.y,
            new_goal.pose.orientation.z,
            new_goal.pose.orientation.w,
        ) = q_out
        return new_goal

    def switch_map(self):
        self.state = STATE_MAP_SWITCH
        nr = self.active_ramp["to"]
        map_file = os.path.join(
            roslib.packages.get_pkg_dir("homewhere"), self.map_table[nr]["file"]
        )
        rospy.loginfo("Loading map: %s", map_file)

        # kill old map_server (load_map.launch will restart amcl & map_server)
        subprocess.call(["rosnode", "kill", "/map_server"])
        subprocess.Popen(["roslaunch", "homewhere", "load_map.launch", f"map_file:={map_file}"])
        subprocess.Popen(["roslaunch", "homewhere", "move_base.launch"])

        # initialpose at exit
        xp, yp, yaw = self.active_ramp["exit_pose"]
        ip = PoseWithCovarianceStamped()
        ip.header.stamp    = rospy.Time.now()
        ip.header.frame_id = "map"
        ip.pose.pose.position.x = xp
        ip.pose.pose.position.y = yp
        q = quaternion_from_euler(0, 0, math.radians(yaw))
        (
            ip.pose.pose.orientation.x,
            ip.pose.pose.orientation.y,
            ip.pose.pose.orientation.z,
            ip.pose.pose.orientation.w,
        ) = q
        self.init_pub.publish(ip)

        # Compute transformed goal *before* clearing active_ramp
        new_goal_ps = None
        if self.goal_original:
            new_goal_ps = self._reproject_goal_to_new_map(self.goal_original)

        self.current_room = nr
        self.active_ramp  = None
        rospy.sleep(1.0)

        if new_goal_ps:
            rospy.loginfo("Re‑publishing adjusted original goal")
            pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
            rospy.sleep(0.05)
            pub.publish(new_goal_ps)
            self.goal_original = new_goal_ps  # update stored copy

        self.state = STATE_NORMAL_NAV

if __name__ == "__main__":
    try:
        MultiLevelNavManager()
    except rospy.ROSInterruptException:
        pass
