#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from fly_catcher_interfaces.msg import DroneCmd


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x*x + y*y + z*z)


def clamp_norm3(x: float, y: float, z: float, max_n: float):
    n = norm3(x, y, z)
    if n < 1e-9 or n <= max_n:
        return x, y, z
    s = max_n / n
    return x*s, y*s, z*s


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    # Standard ZYX
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr*cp*cy + sr*sp*sy
    q.x = sr*cp*cy - cr*sp*sy
    q.y = cr*sp*cy + sr*cp*sy
    q.z = cr*cp*sy - sr*sp*cy
    return q


class DroneSimNode(Node):
    """
    Sub: /drone/cmd (fly_catcher_interfaces/msg/DroneCmd)
    Pub: /drone/state (nav_msgs/Odometry) @ 10Hz

    Dynamics:
      - Convert cmd -> desired velocity vector
      - Apply acceleration limit (2g)
      - Apply speed clamp (0..3 m/s)
      - Integrate position
    """

    def __init__(self):
        super().__init__("drone_sim")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("cmd_topic", "/drone/cmd")
        self.declare_parameter("state_topic", "/drone/state")
        self.declare_parameter("rate_hz", 10.0)

        # Limits
        self.declare_parameter("speed_max", 3.0)   # [m/s]
        self.declare_parameter("accel_max_g", 2.0) # [g], magnitude limit

        # Thrust mapping: thrust [0..100] -> speed_cmd [0..speed_max]
        self.declare_parameter("thrust_min", 0.0)
        self.declare_parameter("thrust_max", 100.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.rate = float(self.get_parameter("rate_hz").value)

        self.sub = self.create_subscription(DroneCmd, self.cmd_topic, self.on_cmd, 10)
        self.pub = self.create_publisher(Odometry, self.state_topic, 10)

        # State
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.last_cmd = DroneCmd()
        self.last_cmd.thrust = 0.0
        self.last_cmd.pitch = 0.0
        self.last_cmd.yaw = 0.0

        self.last_t = self.get_clock().now()
        self.timer = self.create_timer(1.0 / max(1e-3, self.rate), self.step)

        self.get_logger().info(
            f"Drone sim: sub {self.cmd_topic}, pub {self.state_topic}, speed_max=3m/s, accel_max=2g"
        )

    def on_cmd(self, msg: DroneCmd):
        self.last_cmd = msg

    def cmd_to_vdes(self) -> tuple[float, float, float]:
        speed_max = float(self.get_parameter("speed_max").value)
        tmin = float(self.get_parameter("thrust_min").value)
        tmax = float(self.get_parameter("thrust_max").value)

        thrust = clamp(float(self.last_cmd.thrust), tmin, tmax)
        yaw = float(self.last_cmd.yaw)
        pitch = float(self.last_cmd.pitch)

        # Map thrust -> desired speed [0..speed_max]
        u = (thrust - tmin) / max(1e-6, (tmax - tmin))
        speed_cmd = clamp(u * speed_max, 0.0, speed_max)

        # Direction unit vector from yaw/pitch (ENU)
        # yaw: heading in XY; pitch: elevation (+ up)
        cx = math.cos(pitch) * math.cos(yaw)
        cy = math.cos(pitch) * math.sin(yaw)
        cz = math.sin(pitch)

        vx_des = speed_cmd * cx
        vy_des = speed_cmd * cy
        vz_des = speed_cmd * cz
        return vx_des, vy_des, vz_des

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_t = now

        vx_des, vy_des, vz_des = self.cmd_to_vdes()

        # Acceleration limit: a = (v_des - v)/dt
        ax = (vx_des - self.vx) / dt
        ay = (vy_des - self.vy) / dt
        az = (vz_des - self.vz) / dt

        a_max = float(self.get_parameter("accel_max_g").value) * 9.81
        ax, ay, az = clamp_norm3(ax, ay, az, a_max)

        # Integrate velocity
        self.vx += ax * dt
        self.vy += ay * dt
        self.vz += az * dt

        # Clamp speed 0..speed_max
        speed_max = float(self.get_parameter("speed_max").value)
        sp = norm3(self.vx, self.vy, self.vz)
        if sp > speed_max:
            self.vx, self.vy, self.vz = clamp_norm3(self.vx, self.vy, self.vz, speed_max)

        # Integrate position
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = float(self.z)

        # For visualization, point the drone yaw toward commanded yaw (optional)
        odom.pose.pose.orientation = rpy_to_quat(0.0, 0.0, float(self.last_cmd.yaw))

        odom.twist.twist.linear.x = float(self.vx)
        odom.twist.twist.linear.y = float(self.vy)
        odom.twist.twist.linear.z = float(self.vz)

        self.pub.publish(odom)


def main():
    rclpy.init()
    node = DroneSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
