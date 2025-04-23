#!/usr/bin/env python
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

DIST_TO_RAMP_TH       = 0.2
RAMP_SPEED            = 1.00
RAMP_RATE             = 10

class MultiLevelNavManager:
    def __init__(self):
        rospy.init_node("multilevel_nav_manager")

        # ── params ───────────────────────────────────────────────
        self.start_room  = str(rospy.get_param("~start_room", "1"))
        ramp_cfg         = rospy.get_param("~ramp_config")
        self.map_table   = rospy.get_param("~map_table", {})
        if not self.map_table:
            rospy.logfatal("Empty map_table – provide room definitions in YAML!")
            sys.exit(1)

        with open(ramp_cfg, "r") as f:
            ramps = yaml.safe_load(f)["ramps"]
        self.ramp_table = {(r["from"], r["to"]): r for r in ramps}

        # ── vars ────────────────────────────────────────────────
        self.current_room  = self.start_room
        self.state         = STATE_IDLE
        self.goal_original = None
        self.active_ramp   = None
        self.blind_timer   = None
        self.warmup_timer  = None
        self.finish_timer  = None
        self.warmup_done   = False
        self.on_slope      = False

        # ── pubs/subs ───────────────────────────────────────────
        self.cmd_pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.init_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_cb)
        rospy.Subscriber("/on_slope", Bool, self.slope_cb)

        rospy.loginfo("\033[92m MultilevelNavManager ready in room %s \033[0m", self.start_room)
        rospy.spin()

    # ───────────────── Callbacks ────────────────────────────────
    def goal_cb(self, msg: PoseStamped):
        if self.state == STATE_IDLE or self.state == STATE_MAP_SWITCH:
            if not self.goal_original:
                self.goal_original = msg
                
            desired_room = self.determine_room(self.goal_original)

            if desired_room == self.current_room and self.state != STATE_TO_RAMP:
                self.state = STATE_NORMAL_NAV
                return
            
            # Goal on different floor
            self.cancel_move_base()
            key = (self.current_room, desired_room)
            if key not in self.ramp_table:
                rospy.logerr("No ramp defined for %s → %s", *key)
                return
            self.active_ramp = self.ramp_table[key]
            self.state       = STATE_TO_RAMP
            self.send_goal(self.active_ramp["entry_pose"])

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        if self.state == STATE_TO_RAMP:
            ex, ey, _ = self.active_ramp["entry_pose"]
            d = math.hypot(msg.pose.pose.position.x - ex, msg.pose.pose.position.y - ey)
            if d < DIST_TO_RAMP_TH:
                self.state = STATE_ENTER_RAMP
                self.cancel_move_base()

    def slope_cb(self, msg: Bool):
        self.on_slope = msg.data

        if self.state == STATE_ENTER_RAMP:
            rospy.loginfo("\033[92m ENTER_RAMP → blind warm‑up \033[0m")
            subprocess.call(["rosnode", "kill", "/move_base", "/amcl","/map_server"])
            self.state       = STATE_UP_RAMP
            self.warmup_done = False
            
            self.blind_timer = rospy.Timer(rospy.Duration(1.0 / RAMP_RATE), self.blind_move,oneshot=False)
            self.warmup_timer= rospy.Timer(rospy.Duration(1.0), self.start_blind_move, oneshot=True)
            return

        if self.state == STATE_UP_RAMP and self.warmup_done and not self.on_slope and self.finish_timer is None:
            rospy.loginfo("\033[92m Slope ↓ → finish‑drive \033[0m")
            self.finish_timer = rospy.Timer(rospy.Duration(1.0), self.finish_blind_move, oneshot=True)
            self.state == STATE_MAP_SWITCH
            self.switch_map()

    # ─────────────────── Helpers ────────────────────────────────
    def determine_room(self, ps: PoseStamped):
        x, y = ps.pose.position.x, ps.pose.position.y
        for room_id, info in self.map_table.items():
            xmin, xmax, ymin, ymax = info["bounds"]
            if xmin <= x <= xmax and ymin <= y <= ymax:
                rospy.loginfo("\033[92m Goal (%.2f,%.2f) → room %s \033[0m", x, y, room_id)
                return room_id
        rospy.logwarn("Goal (%.2f,%.2f) not in any bounds; staying in %s", x, y, self.current_room)
        return "0"

    def send_goal(self, xyzr):
        goal = PoseStamped()
        goal.header.frame_id = "map"; goal.header.stamp = rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y = xyzr[0], xyzr[1]
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = quaternion_from_euler(0,0, math.radians(xyzr[2]))
        rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True).publish(goal)

    def cancel_move_base(self):
        rospy.Publisher("/move_base/cancel", GoalID, queue_size=1).publish(GoalID())
        rospy.loginfo("\033[91m move_base cancel sent \033[0m")

    def blind_move(self, _):
        t = Twist(); t.linear.x = RAMP_SPEED; self.cmd_pub.publish(t)

    def start_blind_move(self, _):
        self.warmup_done = True

    def finish_blind_move(self, _):
        if self.blind_timer: self.blind_timer.shutdown(); self.blind_timer=None
        if self.warmup_timer: self.warmup_timer.shutdown(); self.warmup_timer=None
        if self.finish_timer: self.finish_timer.shutdown(); self.finish_timer=None

    # ───────── Map switching ─────────
    def switch_map(self):
        t = Twist()
        t.linear.x = 0
        self.cmd_pub.publish(t)
            
        new_room = self.active_ramp["to"]
        map_file = os.path.join(roslib.packages.get_pkg_dir("homewhere"), self.map_table[new_room]["file"]) # Not hard code!
        rospy.loginfo("\033[92m Loading new map of room %s \033[0m", new_room)

        subprocess.Popen(["roslaunch", "homewhere", "load_map.launch", f"map_file:={map_file}"])

        self.relocalize()
        self.current_room = new_room
        
        # Wait for /amcl_pose to appear (AMCL ready)
        try:
            rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=10.0)
        except rospy.ROSException:
            rospy.logwarn("\033[91m Timed‑out waiting for /amcl_pose – continuing anyway \033[0m")
            
        rospy.sleep(2.0)
        
        # Reproject goal & send
        if self.goal_original:
            new_goal = self.adjust_goal(self.goal_original, self.current_room)
            self.send_goal(new_goal)
            rospy.loginfo("\033[92m Reprojected goal → (%.2f, %.2f) \033[0m", new_goal[0], new_goal[1])
            
        self.active_ramp  = None
        self.state        = STATE_NORMAL_NAV

    def relocalize(self):
        xp, yp, yaw = self.active_ramp["exit_pose"]

        ip = PoseWithCovarianceStamped()
        ip.header.frame_id = "map"
        ip.header.stamp = rospy.Time.now()
        
        # Set position
        ip.pose.pose.position.x = xp
        ip.pose.pose.position.y = yp
        ip.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        quat = quaternion_from_euler(0, 0, math.radians(yaw))
        ip.pose.pose.orientation.x = quat[0]
        ip.pose.pose.orientation.y = quat[1]
        ip.pose.pose.orientation.z = quat[2]
        ip.pose.pose.orientation.w = quat[3]

        # Set covariance — matching what you used in `rostopic pub`
        ip.pose.covariance = [
            0.25, 0,    0, 0, 0, 0,
            0,    0.25, 0, 0, 0, 0,
            0,    0,    0.0, 0, 0, 0,
            0,    0,    0, 0.0, 0, 0,
            0,    0,    0, 0, 0.0, 0,
            0,    0,    0, 0, 0, 0.0685
        ]

        self.init_pub.publish(ip)

    # ───────── Goal reprojection ─────────
    def adjust_goal(self, goal_ps, room_to):
        cx, cy = self.map_table[room_to]["center"]
        dx, dy  = goal_ps.pose.position.x - cx, goal_ps.pose.position.y - cy
        return [dx,dy,0]

if __name__ == "__main__":
    try:
        MultiLevelNavManager()
    except rospy.ROSInterruptException:
        pass
