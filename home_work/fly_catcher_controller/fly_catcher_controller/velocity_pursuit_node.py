import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import Odometry
from fly_catcher_interfaces.msg import DroneCmd
from visualization_msgs.msg import Marker

class VelocityPursuitController(Node):
    def __init__(self):
        super().__init__('velocity_pursuit_node')

        # Parameters
        self.declare_parameter('catch_threshold', 0.2)
        self.dt = 0.1  # 10Hz loop
        
        # --- Kalman Filter State (6D) ---
        # State x = [px, py, pz, vx, vy, vz]
        self.x = np.zeros(6)
        self.P = np.eye(6) * 1.0
        
        # Transition Matrix (F) - Constant Velocity Model
        self.F = np.eye(6)
        self.F[0, 3] = self.F[1, 4] = self.F[2, 5] = self.dt
        
        # Measurement Matrix (H) - We only measure position [x, y, z]
        self.H = np.zeros((3, 6))
        self.H[0:3, 0:3] = np.eye(3)
        
        self.Q = np.eye(6) * 0.05  # Process noise
        self.R = np.eye(3) * 0.1   # Measurement noise
        
        self.initialized_kf = False
        self.first_catch_recorded = False
        self.drone_pos = None
        self.start_time = None

        # Publishers & Subscribers
        self.fly_sub = self.create_subscription(PointStamped, '/fly/position', self.fly_cb, 10)
        self.drone_sub = self.create_subscription(Odometry, '/drone/state', self.drone_cb, 10)
        self.cmd_pub = self.create_publisher(DroneCmd, '/drone/cmd', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.click_pub = self.create_publisher(PointStamped, '/clicked_point', 10)

        # Unique Visuals for Velocity Node (Magenta/Green)
        self.drone_line = self.init_line_marker(10, (1.0, 0.0, 1.0)) # Magenta
        self.fly_line = self.init_line_marker(11, (0.0, 1.0, 0.0))   # Green

        self.timer = self.create_timer(self.dt, self.control_loop)

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
        
        if not self.initialized_kf:
            self.x[0:3] = z
            self.initialized_kf = True
            self.start_time = self.get_clock().now()
            return

        # KF Prediction
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # KF Update (Correction)
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

        # Update Fly Path Marker
        self.fly_line.header.stamp = self.get_clock().now().to_msg()
        self.fly_line.points.append(msg.point)
        self.marker_pub.publish(self.fly_line)

    def drone_cb(self, msg):
        self.drone_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        # Update Drone Path Marker
        p = Point(x=self.drone_pos[0], y=self.drone_pos[1], z=self.drone_pos[2])
        self.drone_line.header.stamp = self.get_clock().now().to_msg()
        self.drone_line.points.append(p)
        self.marker_pub.publish(self.drone_line)

    def control_loop(self):
        if not self.initialized_kf or self.drone_pos is None:
            return
        
        # 1. Calculate the current real distance to the fly
        real_dist_to_fly = np.linalg.norm(self.x[0:3] - self.drone_pos)
        
        # 2. ADAPTIVE LOOK-AHEAD CALCULATION
        # We want 0.6s at 3 meters out, and 0.1s when we are 0.5 meters out
        # Slope formula: look_ahead = distance * 0.2
        #raw_look_ahead = real_dist_to_fly * 0.2
        look_ahead = float(np.clip(real_dist_to_fly, 0.1, 0.6))
        
        # 3. Prediction based on adaptive time
        p_predicted = self.x[0:3] + (self.x[3:6] * look_ahead)
        diff_to_predicted = p_predicted - self.drone_pos
        
        # 4. Extract Fly Velocity for Feed-Forward
        fly_vel_mag = np.linalg.norm(self.x[3:6])
        
        cmd = DroneCmd()
        
        # --- CATCH CHECK ---
        if real_dist_to_fly < self.get_parameter('catch_threshold').value:
            cmd.thrust = 0.0
            if not self.first_catch_recorded:
                self.first_catch_recorded = True
                duration = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                self.publish_success(duration)
        else:
            # --- SMOOTH THRUST (Proportional + Feed-Forward) ---
            # Using our dynamic thrust equation to ensure we overtake the fly
            feed_forward_gain = 30.0 
            kp = 50.0 
            thrust_needed = (fly_vel_mag * feed_forward_gain) + (kp * real_dist_to_fly)
            cmd.thrust = float(np.clip(thrust_needed, 15.0, 100.0))

            # --- STEERING ---
            cmd.yaw = np.arctan2(diff_to_predicted[1], diff_to_predicted[0])
            cmd.pitch = np.arctan2(diff_to_predicted[2], np.linalg.norm(diff_to_predicted[:2]))

        self.cmd_pub.publish(cmd)
        self.publish_prediction_marker(p_predicted)

    def publish_success(self, elapsed):
        self.get_logger().info(f"VELOCITY CATCH SUCCESS: {elapsed:.2f}s")
        msg = PointStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = self.drone_pos.tolist()
        self.click_pub.publish(msg)

    def publish_prediction_marker(self, pos):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.id = 99
        m.type = Marker.SPHERE
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos.tolist()
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.a = 1.0; m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0 # Yellow Prediction
        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPursuitController()
    rclpy.spin(node)
    rclpy.shutdown()