import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry
from fly_catcher_interfaces.msg import DroneCmd
from visualization_msgs.msg import Marker

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pursuit_node')

        # Core Parameters
        self.declare_parameter('catch_threshold', 0.2)
        
        # Kalman Filter (Position only, since we aren't predicting velocity)
        self.dt = 0.1 
        self.x = np.zeros(3)  # State: [x, y, z]
        self.P = np.eye(3) * 1.0
        self.Q = np.eye(3) * 0.01 
        self.R = np.eye(3) * 0.1
        
        self.initialized_kf = False
        self.first_catch_recorded = False
        self.drone_pos = None
        self.start_time = None

        # Subscriptions
        self.fly_sub = self.create_subscription(PointStamped, '/fly/position', self.fly_cb, 10)
        self.drone_sub = self.create_subscription(Odometry, '/drone/state', self.drone_cb, 10)
        
        # Publications
        self.cmd_pub = self.create_publisher(DroneCmd, '/drone/cmd', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.click_pub = self.create_publisher(PointStamped, '/clicked_point', 10)

        # Visual Trajectories
        self.drone_line = self.init_line_marker(1, (0.0, 1.0, 1.0)) # Cyan
        self.fly_line = self.init_line_marker(2, (1.0, 1.0, 0.0))   # Yellow

        self.timer = self.create_timer(0.1, self.control_loop)

    def init_line_marker(self, marker_id, color):
        m = Marker()
        m.header.frame_id = "map"
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.id = marker_id
        m.scale.x = 0.05
        m.color.a = 0.7
        m.color.r, m.color.g, m.color.b = color
        return m

    def fly_cb(self, msg):
        z = np.array([msg.point.x, msg.point.y, msg.point.z])
        
        if not self.initialized_kf:
            self.x = z
            self.initialized_kf = True
            self.start_time = self.get_clock().now()
            self.get_logger().info("Fly Detected. Pure Pursuit Active.")
            return

        # Simple Kalman Update for smooth position tracking
        # (Innovation)
        y = z - self.x
        S = self.P + self.R
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K) @ self.P + self.Q

        # Update Fly Path
        self.fly_line.points.append(msg.point)
        self.marker_pub.publish(self.fly_line)

    def drone_cb(self, msg):
        self.drone_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        # Update Drone Path
        p = Point()
        p.x, p.y, p.z = self.drone_pos
        self.drone_line.points.append(p)
        self.marker_pub.publish(self.drone_line)

    def control_loop(self):
        if not self.initialized_kf or self.drone_pos is None:
            return
        
        # In Pure Pursuit, the target IS the fly's current filtered position
        p_target = self.x
        diff = p_target - self.drone_pos
        dist = np.linalg.norm(diff)

        cmd = DroneCmd()
        
        if dist < self.get_parameter('catch_threshold').value:
            if not self.first_catch_recorded:
                total_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                self.publish_first_catch(total_time)
                self.first_catch_recorded = True
            cmd.thrust = 0.0
        else:
            # High-performance pursuit: Aim directly at target with max thrust
            cmd.yaw = np.arctan2(diff[1], diff[0])
            cmd.pitch = np.arctan2(diff[2], np.linalg.norm(diff[:2]))
            cmd.thrust = 100.0 # Simulator will cap speed at 3m/s

        self.cmd_pub.publish(cmd)
        self.publish_target_marker(p_target)

    def publish_first_catch(self, elapsed):
        msg = PointStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = self.drone_pos.tolist()
        self.click_pub.publish(msg)
        self.get_logger().info(f"SUCCESS: Fly caught in {elapsed:.2f}s at {self.drone_pos.tolist()}")

    def publish_target_marker(self, pos):
        m = Marker()
        m.header.frame_id = "map"
        m.id = 0
        m.type = Marker.SPHERE
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos.tolist()
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.a = 0.9; m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0 # Red Target
        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PurePursuitController())
    rclpy.shutdown()