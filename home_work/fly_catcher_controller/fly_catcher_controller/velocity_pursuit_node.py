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
        
        # --- 6D Kalman Filter State ---
        # State x = [px, py, pz, vx, vy, vz]
        self.x = np.zeros(6)
        #self.P = np.eye(6) #* 0.1
        self.P = P = np.diag([0.01, 0.01, 0.01, 1.0, 1.0, 1.0])
        
        # Transition Matrix (F)
        self.F = np.eye(6)
        self.F[0, 3] = self.F[1, 4] = self.F[2, 5] = self.dt
        
        # measurement matrix measures 6 states [pos + vel]
        # velocity is not observed but being calculated from the location
        self.H = np.eye(6)
        
        # Noise Matrices (Tuned for Simulator "Ground Truth")
        self.Q = np.eye(6) * 0.01  # Process noise (Fly intent)
        self.R = np.eye(6) * 0.001 # Measurement noise (Sensor trust)
        
        # Logic Variables
        self.initialized_kf = False
        self.first_catch_recorded = False
        self.drone_pos = None
        self.last_fly_pos = None
        self.last_fly_time = None
        self.start_time = None

        # Publishers & Subscribers
        self.fly_sub = self.create_subscription(PointStamped, '/fly/position', self.fly_cb, 10)
        self.drone_sub = self.create_subscription(Odometry, '/drone/state', self.drone_cb, 10)
        self.cmd_pub = self.create_publisher(DroneCmd, '/drone/cmd', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.click_pub = self.create_publisher(PointStamped, '/clicked_point', 10)

        # Visuals
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
        now = self.get_clock().now()
        current_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        
        if not self.initialized_kf:
            self.x[0:3] = current_pos
            self.last_fly_pos = current_pos
            self.last_fly_time = now
            self.initialized_kf = True
            self.start_time = now
            return

        # caculate velocity
        dt = (now - self.last_fly_time).nanoseconds / 1e9
        if dt <= 0: return
        
        measured_vel = (current_pos - self.last_fly_pos) / dt
        
        # mesurment
        z = np.concatenate([current_pos, measured_vel])

        # prediction
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # update
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

        # push positions
        self.last_fly_pos = current_pos
        self.last_fly_time = now

        # update marker array
        self.fly_line.header.stamp = now.to_msg()
        self.fly_line.points.append(msg.point)
        self.marker_pub.publish(self.fly_line)

    def drone_cb(self, msg):
        self.drone_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        p = Point(x=self.drone_pos[0], y=self.drone_pos[1], z=self.drone_pos[2])
        self.drone_line.header.stamp = self.get_clock().now().to_msg()
        self.drone_line.points.append(p)
        self.marker_pub.publish(self.drone_line)

    def control_loop(self):
        if not self.initialized_kf or self.drone_pos is None:
            return
        
        # Real distance for logic
        dist_vec = self.x[0:3] - self.drone_pos
        real_dist = np.linalg.norm(dist_vec)
        
        # Adaptive Look-Ahead (L grows with distance)
        look_ahead = np.clip(real_dist * 0.25, 0.1, 0.7)
        
        # Future Prediction: P_future = P_now + (V_now * L)
        p_predicted = self.x[0:3] + (self.x[3:6] * look_ahead)
        diff_to_predicted = p_predicted - self.drone_pos
        
        fly_vel_mag = np.linalg.norm(self.x[3:6])
        cmd = DroneCmd()
        
        if real_dist < self.get_parameter('catch_threshold').value:
            cmd.thrust = 0.0
            if not self.first_catch_recorded:
                self.first_catch_recorded = True
                duration = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                self.publish_success(duration)
        else:
            # P + Feed-Forward (Ensure drone is always faster than fly)
            kp = 45.0
            k_ff = 32.0 
            thrust_needed = (fly_vel_mag * k_ff) + (kp * real_dist)
            cmd.thrust = float(np.clip(thrust_needed, 15.0, 100.0))

            # Steering toward the future point
            cmd.yaw = np.arctan2(diff_to_predicted[1], diff_to_predicted[0])
            cmd.pitch = np.arctan2(diff_to_predicted[2], np.linalg.norm(diff_to_predicted[:2]))

        self.cmd_pub.publish(cmd)
        self.publish_prediction_marker(p_predicted)

    def publish_success(self, elapsed):
        self.get_logger().info(f"Catch Success: {elapsed:.2f}s")
       
        msg = PointStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = self.drone_pos.tolist()
        self.click_pub.publish(msg)
        self.get_logger().info(f"location coodiante is:[{ msg.point.x:.2f}, { msg.point.y:.2f}, { msg.point.z:.2f}] ")

    def publish_prediction_marker(self, pos):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.id = 99
        m.type = Marker.SPHERE
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos.tolist()
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.a = 1.0; m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0 # Yellow
        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPursuitController()
    rclpy.spin(node)
    rclpy.shutdown()