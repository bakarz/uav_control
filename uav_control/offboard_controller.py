import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)

# ── Parameters ───────────────────────────────────────────────────────────
TAKEOFF_ALTITUDE    = -5.0   # NED metres (negative = up)
TAKEOFF_TOLERANCE   = 0.4    # m — altitude band to consider takeoff done
WAYPOINT_TOLERANCE  = 0.4    # m — XYZ arrival radius
WAYPOINT_DWELL_S    = 3.0    # s — pause at each waypoint for delivery
LAND_SPEED_Z        = 0.3    # m/s — descent/ascent rate during landing/takeoff
LAND_DETECT_Z       = 0.15   # m — NED z threshold for touchdown detection
WARMUP_TICKS        = 10     # ticks at 10 Hz before arming (~1 s)
CTRL_HZ             = 10     # control loop rate (Hz)

# ── Waypoints: (x, y, z_ned, yaw_deg | None) ─────────────────────────────────
# NED: z negative = above ground.  yaw None = auto-face the waypoint.
WAYPOINTS = [
    ( 5.0,   0.0, -5.0, 90.0),   # WP1 — 10 m North
    ( 5.0,  5.0, -5.0, 90.0),   # WP2 — 10 m East
    (  0.0,  5.0, -5.0, 90.0),   # WP3 — 10 m South
    (  0.0,   0.0, -5.0,  0.0),   # WP4 — return home, face North
]


# ──────────────────────────────────────────────────────────────────────────────
# State identifiers
# ──────────────────────────────────────────────────────────────────────────────
class State:
    IDLE         = "IDLE"
    WARMUP       = "WARMUP"
    ARM          = "ARM"
    TAKEOFF      = "TAKEOFF"
    MISSION      = "MISSION"
    LAND         = "LAND"
    DWELL        = "DWELL"
    DONE         = "DONE"


# ──────────────────────────────────────────────────────────────────────────────
# Node
# ──────────────────────────────────────────────────────────────────────────────
class OffboardController(Node):

    def __init__(self):
        super().__init__("offboard_controller")

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry",
            self._odom_cb, px4_qos)
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status",
            self._status_cb, px4_qos)

        # ── Internal state ────────────────────────────────────────────────────
        self.state        = State.IDLE
        self.tick         = 0
        self.pos          = [0.0, 0.0, 0.0]  # NED metres
        self.yaw          = 0.0              # radians
        self.arming_state = 0
        self.nav_state    = 0

        # Waypoint navigator
        self.wp_index     = 0
        self.dwell_ticks  = 0
        self.dwell_target = int(WAYPOINT_DWELL_S * CTRL_HZ)
        self.land_z       = TAKEOFF_ALTITUDE

        # ── Timer ─────────────────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / CTRL_HZ, self._loop)

        self.get_logger().info(
            f"OffboardController ready — {len(WAYPOINTS)} waypoints")
        self._log_mission()

    # ──────────────────────────────────────────────────────────────────────────
    # ROS 2 callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: VehicleOdometry):
        self.pos = list(msg.position)
        q = msg.q                            # [w, x, y, z]
        self.yaw = math.atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2),
        )

    def _status_cb(self, msg: VehicleStatus):
        self.arming_state = msg.arming_state
        self.nav_state    = msg.nav_state

    # ──────────────────────────────────────────────────────────────────────────
    # Main 10 Hz loop
    # ──────────────────────────────────────────────────────────────────────────

    def _loop(self):
        self.tick += 1
        self._pub_offboard_mode()          # heartbeat — always first

        if self.state == State.IDLE:
            self._go(State.WARMUP)

        elif self.state == State.WARMUP:
            self._pub_setpoint(0.0, 0.0, TAKEOFF_ALTITUDE, 0.0)
            if self.tick >= WARMUP_TICKS:
                self._go(State.ARM)

        elif self.state == State.ARM:
            self._pub_setpoint(0.0, 0.0, TAKEOFF_ALTITUDE, 0.0)
            self._engage_offboard()
            self._arm()
            self._go(State.TAKEOFF)

        elif self.state == State.TAKEOFF:
            self._pub_setpoint(0.0, 0.0, TAKEOFF_ALTITUDE, 0.0)
            if abs(self.pos[2] - TAKEOFF_ALTITUDE) < TAKEOFF_TOLERANCE:
                self.get_logger().info(
                    f"Takeoff complete ({abs(TAKEOFF_ALTITUDE):.0f} m) "
                    f"— starting waypoint mission")
                self._go(State.MISSION)

        elif self.state == State.MISSION:
            self._step_mission()

        elif self.state == State.LAND:
            self._step_land()

        elif self.state == State.DWELL:
            self._step_dwell()

        elif self.state == State.DONE:
            if self.tick % 50 == 0:
                self.get_logger().info("All done. Press Ctrl+C to exit.")

    # ──────────────────────────────────────────────────────────────────────────
    # Pose-to-pose waypoint navigation
    # ──────────────────────────────────────────────────────────────────────────

    def _step_mission(self):
        """Fly to current waypoint; switch to DWELL on arrival."""
        if self.wp_index >= len(WAYPOINTS):
            self.get_logger().info("Waypoint mission complete")
            self._go(State.DONE)
            return

        wx, wy, wz, wyaw = WAYPOINTS[self.wp_index]
        yaw_rad = (math.radians(wyaw) if wyaw is not None
                   else self._bearing_to(wx, wy))
        self._pub_setpoint(wx, wy, wz, yaw_rad)

        if self.tick % CTRL_HZ == 0:
            dist = self._dist3(wx, wy, wz)
            self.get_logger().info(
                f"  WP{self.wp_index + 1}/{len(WAYPOINTS)}"
                f"  → ({wx:.1f}, {wy:.1f}, {abs(wz):.1f} m)"
                f"  dist={dist:.2f} m"
                f"  pos=({self.pos[0]:.1f}, {self.pos[1]:.1f},"
                f" {abs(self.pos[2]):.1f} m)"
            )

        if self._dist3(wx, wy, wz) < WAYPOINT_TOLERANCE:
            self.get_logger().info(
                f"Arrived WP{self.wp_index + 1} — landing for delivery")
            self.land_z = self.pos[2]
            self._go(State.LAND)

    def _step_dwell(self):
        """Hold on the ground for delivery at the current waypoint."""
        wx, wy, wz, wyaw = WAYPOINTS[self.wp_index]
        yaw_rad = (math.radians(wyaw) if wyaw is not None
                   else self._bearing_to(wx, wy))
        self._pub_setpoint(wx, wy, 0.0, yaw_rad)

        self.dwell_ticks += 1
        if self.dwell_ticks >= self.dwell_target:
            if self.wp_index + 1 >= len(WAYPOINTS):
                self.get_logger().info("Delivery mission complete")
                self._go(State.DONE)
            else:
                self.wp_index += 1
                self._go(State.TAKEOFF)

    # ──────────────────────────────────────────────────────────────────────────
    # Landing
    # ──────────────────────────────────────────────────────────────────────────

    def _step_land(self):
        """Descend to the ground at the current waypoint for delivery."""
        wx, wy, wz, wyaw = WAYPOINTS[self.wp_index]
        yaw_rad = (math.radians(wyaw) if wyaw is not None
                   else self._bearing_to(wx, wy))

        self.land_z += LAND_SPEED_Z / CTRL_HZ
        target_z = min(self.land_z, 0.0)
        self._pub_setpoint(wx, wy, target_z, yaw_rad)

        if self.tick % CTRL_HZ == 0:
            self.get_logger().info(
                f"Landing at WP{self.wp_index + 1}  alt={abs(self.pos[2]):.2f} m")

        if self.pos[2] > -LAND_DETECT_Z:
            self.get_logger().info(
                f"Touchdown at WP{self.wp_index + 1} — delivering medicaments")
            self.dwell_ticks = 0
            self._go(State.DWELL)

    # ──────────────────────────────────────────────────────────────────────────
    # Publisher helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _pub_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = self._ts()
        self.pub_offboard.publish(msg)

    def _pub_setpoint(self, x: float, y: float, z: float, yaw: float):
        """Position-only setpoint."""
        msg = TrajectorySetpoint()
        msg.position     = [float(x), float(y), float(z)]
        msg.velocity     = [float("nan")] * 3   # ignored by PX4 when NaN
        msg.yaw          = float(yaw)
        msg.timestamp    = self._ts()
        self.pub_setpoint.publish(msg)

    def _arm(self):
        self.get_logger().info("ARM")
        self._vcmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def _engage_offboard(self):
        self.get_logger().info("→ OFFBOARD mode")
        self._vcmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                   param1=1.0, param2=6.0)

    def _vcmd(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = self._ts()
        self.pub_cmd.publish(msg)

    # ──────────────────────────────────────────────────────────────────────────
    # Geometry / utility
    # ──────────────────────────────────────────────────────────────────────────

    def _ts(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _dist3(self, x: float, y: float, z: float) -> float:
        return math.sqrt(
            (self.pos[0] - x) ** 2 +
            (self.pos[1] - y) ** 2 +
            (self.pos[2] - z) ** 2
        )

    def _bearing_to(self, x: float, y: float) -> float:
        return math.atan2(y - self.pos[1], x - self.pos[0])

    def _go(self, new_state: str):
        self.get_logger().info(f"  [{self.state}] → [{new_state}]")
        self.state = new_state

    def _log_mission(self):
        self.get_logger().info("── Waypoints ───────────────────────────")
        for i, (x, y, z, yaw) in enumerate(WAYPOINTS):
            self.get_logger().info(
                f"  WP{i+1}  x={x:6.1f}  y={y:6.1f}  "
                f"alt={abs(z):.1f} m  "
                f"yaw={'auto' if yaw is None else str(yaw)+'°'}")
        self.get_logger().info("────────────────────────────────────────")


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")


if __name__ == "__main__":
    main()