#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


def clamp_norm(vx: float, vy: float, vz: float, max_norm: float):
    n = math.sqrt(vx*vx + vy*vy + vz*vz)
    if n < 1e-9 or n <= max_norm:
        return vx, vy, vz
    s = max_norm / n
    return vx*s, vy*s, vz*s


class FlyNode(Node):
    """
    Publishes /fly/position (PointStamped) @ 10Hz.
    Generates a closed-loop 3D path and enforces speed limit 1 m/s.
    World frame: ENU (x=east, y=north, z=up), default frame_id="map".
    """

    def __init__(self):
        super().__init__("fly_sim")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "/fly/position")
        self.declare_parameter("rate_hz", 10.0)

        # Initial fly position relative to drone at t0: 10m north, 3m above
        self.declare_parameter("x0", 0.0)
        self.declare_parameter("y0", 10.0)
        self.declare_parameter("z0", 3.0)

        # Closed-loop path parameters
        self.declare_parameter("radius_m", 4.0)         # horizontal circle radius around (x0,y0)
        self.declare_parameter("vertical_amp_m", 1.0)   # vertical sine amplitude around z0
        self.declare_parameter("path_period_s", 20.0)   # seconds per lap

        # Speed limit
        self.declare_parameter("speed_max", 1.0)        # [m/s]

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.topic = str(self.get_parameter("topic").value)
        self.rate = float(self.get_parameter("rate_hz").value)

        self.pub = self.create_publisher(PointStamped, self.topic, 10)

        self.t0 = self.get_clock().now()
        self.last_t = self.get_clock().now()

        self.x = float(self.get_parameter("x0").value)
        self.y = float(self.get_parameter("y0").value)
        self.z = float(self.get_parameter("z0").value)

        self.timer = self.create_timer(1.0 / max(1e-3, self.rate), self.step)

        self.get_logger().info(
            f"Fly sim publishing {self.topic} @ {self.rate}Hz, speed_max=1m/s, init=({self.x},{self.y},{self.z})"
        )

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_t = now

        # Desired position on parametric loop
        t = (now - self.t0).nanoseconds * 1e-9
        R = float(self.get_parameter("radius_m").value)
        A = float(self.get_parameter("vertical_amp_m").value)
        T = max(1e-3, float(self.get_parameter("path_period_s").value))
        w = 2.0 * math.pi / T

        x0 = float(self.get_parameter("x0").value)
        y0 = float(self.get_parameter("y0").value)
        z0 = float(self.get_parameter("z0").value)

        x_des = x0 + R * math.cos(w * t)
        y_des = y0 + R * math.sin(w * t)
        z_des = z0 + A * math.sin(2.0 * w * t)

        # Simple proportional velocity toward the desired point
        # (Then hard clamp to speed_max.)
        kp = 1.5
        vx = kp * (x_des - self.x)
        vy = kp * (y_des - self.y)
        vz = kp * (z_des - self.z)

        v_max = float(self.get_parameter("speed_max").value)
        vx, vy, vz = clamp_norm(vx, vy, vz, v_max)

        # Integrate
        self.x += vx * dt
        self.y += vy * dt
        self.z += vz * dt

        # Publish
        msg = PointStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.point.x = float(self.x)
        msg.point.y = float(self.y)
        msg.point.z = float(self.z)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FlyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
