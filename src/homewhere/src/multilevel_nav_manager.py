#!/usr/bin/env python
import rospy, yaml, os, sys, subprocess, math, roslib.packages
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionResult
from algo.ramp_path import path_find
from algo.room_determination import determine_room

# ──── State labels ─────────────────────────────────────────────
STATE_IDLE        = 0
STATE_NORMAL_NAV  = 1
STATE_TO_RAMP     = 2
STATE_ENTER_RAMP  = 3
STATE_UP_RAMP     = 4
STATE_MAP_SWITCH  = 5

DIST_TO_RAMP_TH       = 0.3
RAMP_SPEED            = 1.00
RAMP_RATE             = 10

class MultiLevelNavManager:
    def __init__(self):
        rospy.init_node("multilevel_nav_manager")

        # ── params ───────────────────────────────────────────────
        self.start_room   = str(rospy.get_param("~start_room", "1"))
        ramp_cfg          = rospy.get_param("~ramp_config")
        self.map_table    = rospy.get_param("~map_table", {})
        if not self.map_table:
            rospy.logfatal("Empty map_table – provide room definitions in YAML!")
            sys.exit(1)

        with open(ramp_cfg, "r") as f:
            ramps = yaml.safe_load(f)["ramps"]
        self.ramp_table = {(r["from"], r["to"]): r for r in ramps}

        # ── vars ────────────────────────────────────────────────
        self.current_room  = self.start_room
        self.goal_room     = None
        self.state         = STATE_IDLE
        self.goal_original = None
        self.active_ramp   = None
        # path through rooms: e.g. ["1","3","4"]
        self.path          = []
        self.path_index    = 0

        self.blind_timer   = None
        self.warmup_timer  = None
        self.finish_timer  = None
        self.warmup_done   = False
        self.on_slope      = False
        self.ramp_done     = False

        self.orientation = 0.0
        self.yaw = 0.0
        self.start_yaw = 0.0

        # ── pubs/subs ───────────────────────────────────────────
        self.cmd_pub  = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.init_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_cb)
        rospy.Subscriber("/on_slope", Bool, self.slope_cb)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.result_cb)
        rospy.Subscriber("/yaw", Float64, self.yaw_cb)

        rospy.loginfo("\033[92m MultilevelNavManager started in room %s \033[0m", self.start_room)
        rospy.spin()

    # ───────────────── Callbacks ────────────────────────────────
    def yaw_cb(self, msg):
        self.yaw = msg.data


    def goal_cb(self, msg: PoseStamped):
        if self.state != STATE_IDLE:
            return

        self.cancel_move_base()
        rospy.sleep(2.0)

        goal_room = self.determine_room(msg)

        # Store original goal and target room
        self.goal_original = msg
        self.goal_room     = goal_room
        rospy.loginfo("\033[92m Global Goal Received at (%.2f, %.2f) located in room %s \033[0m",
                      msg.pose.position.x, msg.pose.position.y, goal_room)

        # Same-room: direct send
        if goal_room == self.current_room:
            rospy.loginfo("\033[92m Goal is in current room \033[0m")
            self.state = STATE_NORMAL_NAV
            local_goal = self.adjust_goal(self.goal_original, self.current_room)
            self.send_goal(local_goal)
            return

        # Different-room: compute path through multiple rooms
        steps = path_find(self.current_room,
                          goal_room,
                          ramps_yaml   = rospy.get_param("~ramp_config"),
                          map_table_yaml = rospy.get_param("~map_table_config"))
        if len(steps) < 2:
            rospy.logerr("No path detected from %s → %s", self.current_room, goal_room)
            return

        self.path       = steps
        self.path_index = 0
        rospy.loginfo("\033[92m Path to follow %s \033[0m", " → ".join(self.path))

        # Kick off first ramp segment
        self.next_ramp_segment()

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        if self.state == STATE_TO_RAMP:
            ex, ey, _ = self.active_ramp["entry_pose"]
            d = math.hypot(msg.pose.pose.position.x - ex,
                           msg.pose.pose.position.y - ey)
            rospy.loginfo("\033[93m Distance toward ramp: %.2f >= %.2f \033[0m", d, DIST_TO_RAMP_TH)
            if d <= DIST_TO_RAMP_TH:
                self.state = STATE_ENTER_RAMP
                rospy.loginfo("\033[92m Entering ramp..... \033[0m")
                self.cancel_move_base()

    def slope_cb(self, msg: Bool):
        self.on_slope = msg.data

        if self.state == STATE_ENTER_RAMP:
            rospy.loginfo("\033[92m Moving up ramp..... \033[0m")
            subprocess.call(["rosnode", "kill", "/move_base", "/amcl", "/map_server"])

            self.state       = STATE_UP_RAMP
            self.warmup_timer= rospy.Timer(rospy.Duration(3.0), self.start_blind_move, oneshot=True)
            self.blind_timer = rospy.Timer(rospy.Duration(1.0), self.blind_move,    oneshot=False)
            return

        if (self.state == STATE_UP_RAMP and self.warmup_done
                and not self.on_slope and not self.ramp_done):
            rospy.loginfo("\033[92m Finish Drive \033[0m")
            self.finish_timer = rospy.Timer(rospy.Duration(1.0), self.finish_blind_move, oneshot=True)
            self.ramp_done     = True
            rospy.loginfo("\033[92m Switching map....... \033[0m")
            self.switch_map()

    def result_cb(self, msg: MoveBaseActionResult):
        SUCCEEDED = 3
        if msg.status.status == SUCCEEDED and self.current_room == self.goal_room:
            rospy.loginfo("\033[92m Original goal reached – switching to IDLE \033[0m")
            self.reset()

    # ─────────────────── Helpers ────────────────────────────────
    def next_ramp_segment(self):
        fr = self.path[self.path_index]
        to = self.path[self.path_index + 1]
        key = (fr, to)
        if key not in self.ramp_table:
            rospy.logerr("No ramp defined for %s → %s", fr, to)
            self.reset()
            return

        ramp = self.ramp_table[key]
        self.active_ramp = ramp
        self.state       = STATE_TO_RAMP
        rospy.loginfo("➔ Heading from room %s to %s via ramp %s", fr, to, ramp["id"])
        rospy.loginfo("Moving toward ramp at %s .....", ramp["entry_pose"])
        self.warmup_done = False
        self.ramp_done   = False
        self.send_goal(ramp["entry_pose"])
        self.orientation = ramp["orientation"]

    def switch_map(self):
        new_room = self.active_ramp["to"]
        map_file = os.path.join(
            roslib.packages.get_pkg_dir("homewhere"),
            self.map_table[new_room]["file"]
        )
        rospy.loginfo("\033[92m Loading new map of room %s \033[0m", new_room)
        subprocess.Popen([
            "roslaunch", "homewhere", "load_map.launch",
            f"map_file:={map_file}"
        ])

        self.relocalize()
        self.current_room = new_room

        # wait for amcl to start
        try:
            rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped, timeout=10.0)
        except rospy.ROSException:
            rospy.logwarn("\033[91m Timed‑out waiting for /amcl_pose – continuing anyway \033[0m")
        rospy.sleep(5.0)

        # advance path index and decide next action
        self.path_index += 1
        if self.path_index < len(self.path) - 1:
            rospy.loginfo("→ Completed ramp, moving next segment")
            self.next_ramp_segment()
        else:
            rospy.loginfo("→ All ramps done; now sending final goal")
            local_goal = self.adjust_goal(self.goal_original, self.current_room)
            self.send_goal(local_goal)
            self.state = STATE_NORMAL_NAV

    def local_to_global(self, x_local, y_local, room):
        cx, cy = self.map_table[room]["center"]
        return x_local + cx, y_local + cy

    def global_to_local(self, x_global, y_global, room):
        cx, cy = self.map_table[room]["center"]
        return x_global - cx, y_global - cy

    def determine_room(self, ps: PoseStamped):
        x, y = ps.pose.position.x, ps.pose.position.y
        # for room_id, info in self.map_table.items():
        #     xmin, xmax, ymin, ymax = info["bounds"]
        #     if xmin <= x <= xmax and ymin <= y <= ymax:
        #         rospy.loginfo("\033[92m Goal (%.2f,%.2f) is in room %s \033[0m", x, y, room_id)
        #         return room_id
        # rospy.logwarn("\033[91m Goal (%.2f,%.2f) is not in any bounds \033[0m", x, y)
        # return "0"
        return determine_room(yaml_path=rospy.get_param("~map_table_config"), x_global=x, y_global=y)

    def send_goal(self, xyzr):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y = xyzr[0], xyzr[1]
        quat = quaternion_from_euler(0, 0, math.radians(xyzr[2]))
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = quat
        rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True).publish(goal)
        rospy.loginfo("\033[92m Goal sent → (%.2f, %.2f) \033[0m", xyzr[0], xyzr[1])

    def cancel_move_base(self):
        rospy.Publisher("/move_base/cancel", GoalID, queue_size=1).publish(GoalID())
        rospy.loginfo("\033[91m move_base cancel sent \033[0m")

    def blind_move(self, event=None):
        angle = math.radians(self.orientation) - self.start_yaw
        t = Twist()
        t.linear.x = RAMP_SPEED * math.cos(angle)
        t.linear.y = RAMP_SPEED * math.sin(angle)
        print(f"Angle: {self.orientation}, Yaw: {math.degrees(self.start_yaw)}, True Angle: {math.degrees(angle)}")
        self.cmd_pub.publish(t)

    def start_blind_move(self, event=None):
        self.blind_move()
        rospy.sleep(1.0)
        self.warmup_done = True
        self.start_yaw = self.yaw
        rospy.loginfo("\033[92m Blind move start \033[0m")

    def finish_blind_move(self, event=None):
        if self.blind_timer:   self.blind_timer.shutdown()
        if self.warmup_timer:  self.warmup_timer.shutdown()
        if self.finish_timer:  self.finish_timer.shutdown()
        t = Twist(); t.linear.x = 0.0; self.cmd_pub.publish(t)

    def relocalize(self):
        xp, yp, yaw = self.active_ramp["exit_pose"]
        ip = PoseWithCovarianceStamped()
        ip.header.frame_id = "map"
        ip.header.stamp = rospy.Time.now()
        ip.pose.pose.position.x = xp
        ip.pose.pose.position.y = yp
        ip.pose.pose.position.z = 0.0
        quat = quaternion_from_euler(0, 0, math.radians(yaw))
        ip.pose.pose.orientation.x = quat[0]
        ip.pose.pose.orientation.y = quat[1]
        ip.pose.pose.orientation.z = quat[2]
        ip.pose.pose.orientation.w = quat[3]
        ip.pose.covariance = [
            0.25, 0,    0, 0, 0, 0,
            0,    0.25, 0, 0, 0, 0,
            0,    0,    0.0, 0, 0, 0,
            0,    0,    0, 0.0, 0, 0,
            0,    0,    0, 0, 0.0, 0,
            0,    0,    0, 0, 0, 0.0685
        ]
        self.init_pub.publish(ip)

    def adjust_goal(self, goal_ps, room_to):
        gx, gy = goal_ps.pose.position.x, goal_ps.pose.position.y
        lx, ly = self.global_to_local(gx, gy, room_to)
        return [lx, ly, 0]

    def reset(self):
        self.goal_original = None
        self.goal_room     = None
        self.state         = STATE_IDLE
        self.active_ramp   = None
        self.blind_timer   = None
        self.warmup_timer  = None
        self.finish_timer  = None
        self.warmup_done   = False
        self.on_slope      = False
        self.ramp_done     = False
        self.path          = []
        self.path_index    = 0
        rospy.loginfo("\033[92m ROBOT RESET \033[0m")

if __name__ == "__main__":
    try:
        MultiLevelNavManager()
    except rospy.ROSInterruptException:
        pass