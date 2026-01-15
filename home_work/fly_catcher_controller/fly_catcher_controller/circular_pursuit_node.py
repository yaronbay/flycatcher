import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry
from fly_catcher_interfaces.msg import DroneCmd
from visualization_msgs.msg import Marker

class CircularPursuitController(Node):
    def __init__(self):
        super().__init__('circular_pursuit_node')

        # --- CTRV State: [x, y, z, v, yaw, yaw_rate] ---
        self.x = np.zeros(6) 
        self.P = np.eye(6) * 1.0
        self.dt = 0.1 #
        
        # Noise settings
        self.Q = np.diag([0.05, 0.05, 0.05, 0.1, 0.1, 0.05])
        self.R = np.eye(3) * 0.01
        
        self.initialized = False
        self.first_catch_recorded = False
        self.drone_pos = None
        self.start_time = None

        # ROS Comm
        self.fly_sub = self.create_subscription(PointStamped, '/fly/position', self.fly_cb, 10)
        self.drone_sub = self.create_subscription(Odometry, '/drone/state', self.drone_cb, 10)
        self.cmd_pub = self.create_publisher(DroneCmd, '/drone/cmd', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.click_pub = self.create_publisher(PointStamped, '/clicked_point', 10)

        # Visualization Markers
        self.drone_line = self.init_line_marker(20, (0.0, 1.0, 1.0)) # Cyan
        self.fly_line = self.init_line_marker(21, (1.0, 1.0, 0.0))   # Yellow

        self.create_timer(self.dt, self.control_loop)

    def init_line_marker(self, marker_id, color):
        m = Marker()
        m.header.frame_id = "map"
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.id = marker_id
        m.scale.x = 0.05
        m.color.a = 0.8
        m.color.r, m.color.g, m.color.b = color
        return m

    def fly_cb(self, msg):
        z = np.array([msg.point.x, msg.point.y, msg.point.z])
        if not self.initialized:
            self.x[0:3] = z
            self.initialized = True
            self.start_time = self.get_clock().now()
            return

        # --- EKF Prediction Step ---
        v, yaw, omega = self.x[3], self.x[4], self.x[5]
        if abs(omega) > 0.001:
            self.x[0] += (v/omega) * (np.sin(yaw + omega*self.dt) - np.sin(yaw))
            self.x[1] += (v/omega) * (-np.cos(yaw + omega*self.dt) + np.cos(yaw))
        else:
            self.x[0] += v * np.cos(yaw) * self.dt
            self.x[1] += v * np.sin(yaw) * self.dt
        self.x[4] += omega * self.dt

        # Jacobian F for CTRV
        F = np.eye(6)
        F[0, 3], F[0, 4] = np.cos(yaw)*self.dt, -v*np.sin(yaw)*self.dt
        F[1, 3], F[1, 4] = np.sin(yaw)*self.dt,  v*np.cos(yaw)*self.dt
        self.P = F @ self.P @ F.T + self.Q

        # --- EKF Update Step ---
        H = np.zeros((3, 6)); H[0:3, 0:3] = np.eye(3)
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

        # Update Trail
        self.fly_line.points.append(msg.point)
        self.marker_pub.publish(self.fly_line)

    def drone_cb(self, msg):
        self.drone_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        p = Point(x=self.drone_pos[0], y=self.drone_pos[1], z=self.drone_pos[2])
        self.drone_line.points.append(p)
        self.marker_pub.publish(self.drone_line)

    def control_loop(self):
        if not self.initialized or self.drone_pos is None: return

        dist = np.linalg.norm(self.x[0:3] - self.drone_pos)
        v, yaw, omega = self.x[3], self.x[4], self.x[5]

        # Intercept math: aim ahead along the curve
        #look_ahead = np.clip(dist * 0.25, 0.1, 0.8)
        #look_ahead = np.clip(dist , 0.1, 0.8)
        look_ahead = 0.1
        target = np.array([
            self.x[0] + (v * np.cos(yaw + omega*look_ahead) * look_ahead),
            self.x[1] + (v * np.sin(yaw + omega*look_ahead) * look_ahead),
            self.x[2]
        ])

        cmd = DroneCmd()
        if dist < 0.2:
            cmd.thrust = 0.0
            if not self.first_catch_recorded:
                self.first_catch_recorded = True
                self.publish_success()
        else:
            # P + Feed-Forward
            cmd.thrust = float(np.clip((v * 32.0) + (dist * 45.0), 15.0, 100.0))
            diff = target - self.drone_pos
            cmd.yaw = np.arctan2(diff[1], diff[0])
            cmd.pitch = np.arctan2(diff[2], np.linalg.norm(diff[:2]))

        self.cmd_pub.publish(cmd)
        self.pub_target_marker(target)

    def pub_target_marker(self, pos):
        m = Marker()
        m.header.frame_id, m.header.stamp = "map", self.get_clock().now().to_msg()
        m.id, m.type = 88, Marker.SPHERE
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos.tolist()
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.a, m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0, 1.0 # Purple
        self.marker_pub.publish(m)

    def publish_success(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.get_logger().info(f"CIRCULAR CATCH SUCCESS: {elapsed:.2f}s")
        msg = PointStamped()
        msg.header.frame_id, msg.header.stamp = "map", self.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = self.drone_pos.tolist()
        self.click_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CircularPursuitController())
    rclpy.shutdown()