# this is my main flay catcher logic
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

        # parameters declerations
        self.declare_parameter('catch_threshold')   # distance to trigger "catch"
        self.declare_parameter('kp_dist')          # proportional gain for distance
        self.declare_parameter('k_ff_vel')         # ff gain for speed matching
        self.declare_parameter('look_ahead_ratio') # how much to look ahead
        self.declare_parameter('min_look_ahead')    # min seconds to look ahead
        self.declare_parameter('max_look_ahead')    # max seconds to look ahead
        self.declare_parameter('min_thrust')       # min motor power
        self.declare_parameter('max_thrust')      # max motor power
        self.declare_parameter('q_noise')          #  Process noise
        self.declare_parameter('r_noise')         #  Measurement noise
        self.declare_parameter('dt')
        self.declare_parameter('hold_after_catch', True)

        self.dt = self.get_parameter('dt').value  #  (10Hz)
        
        # 6D Kalman Filter initialize
        self.x = np.zeros(6)
        self.P = np.eye(6) * 0.1
        self.F = np.eye(6)
        self.F[0, 3] = self.F[1, 4] = self.F[2, 5] = self.dt
        self.H = np.eye(6) 
        
        self.initialized_kf = False
        self.first_catch_recorded = False
        self.drone_pos = None
        self.last_fly_pos = None
        self.last_fly_time = None
        self.start_time = None

        # initial publisher and subscribers
        self.fly_sub = self.create_subscription(PointStamped, '/fly/position', self.fly_cb, 10)
        self.drone_sub = self.create_subscription(Odometry, '/drone/state', self.drone_cb, 10)
        self.cmd_pub = self.create_publisher(DroneCmd, '/drone/cmd', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.click_pub = self.create_publisher(PointStamped, '/clicked_point', 10)

        # rviz visuals
        self.drone_line = self.init_line_marker(10, (1.0, 0.0, 1.0)) # magenta
        self.fly_line = self.init_line_marker(11, (0.0, 1.0, 0.0))   # green

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
            self.last_fly_pos, self.last_fly_time = current_pos, now
            self.initialized_kf, self.start_time = True, now
            return

        # 1. Calculate Derivative Velocity - to be more precise (not abolute t=0.1s)
        dt = (now - self.last_fly_time).nanoseconds / 1e9
        if dt <= 0: return
        measured_vel = (current_pos - self.last_fly_pos) / dt
        z = np.concatenate([current_pos, measured_vel])

        # 2. get dynamic noise params
        q_val = self.get_parameter('q_noise').value
        r_val = self.get_parameter('r_noise').value
        Q = np.eye(6) * q_val
        R = np.eye(6) * r_val

        # 3. Kalman Filter (Prediction + Update)
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + Q
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

        # add point to fly line
        self.last_fly_pos, self.last_fly_time = current_pos, now
        self.fly_line.points.append(msg.point)
        self.marker_pub.publish(self.fly_line)

    def drone_cb(self, msg):
        self.drone_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        p = Point(x=self.drone_pos[0], y=self.drone_pos[1], z=self.drone_pos[2])
        self.drone_line.points.append(p)
        self.marker_pub.publish(self.drone_line)

    def control_loop(self):
        if not self.initialized_kf or self.drone_pos is None:
            return
        
        # Check if we are already in "Hold" mode
        if self.first_catch_recorded and self.get_parameter('hold_after_catch').value:
            cmd = DroneCmd()
            cmd.thrust = 0.0 # Stay powered down
            self.cmd_pub.publish(cmd)
            return

        # Get Parameters
        ratio = self.get_parameter('look_ahead_ratio').value
        l_min = self.get_parameter('min_look_ahead').value
        l_max = self.get_parameter('max_look_ahead').value
        kp = self.get_parameter('kp_dist').value
        k_ff = self.get_parameter('k_ff_vel').value
        t_min = self.get_parameter('min_thrust').value
        t_max = self.get_parameter('max_thrust').value

        # det distance
        dist_vec = self.x[0:3] - self.drone_pos
        real_dist = np.linalg.norm(dist_vec)
        
        # predictive lead
        look_ahead = np.clip(real_dist * ratio, l_min, l_max)
        p_predicted = self.x[0:3] + (self.x[3:6] * look_ahead)
        diff_to_predicted = p_predicted - self.drone_pos
        
        fly_vel_mag = np.linalg.norm(self.x[3:6])
        cmd = DroneCmd()
        
        if real_dist < self.get_parameter('catch_threshold').value:
            cmd.thrust = 0.0
            if not self.first_catch_recorded:
                self.first_catch_recorded = True
                self.publish_success()
        else:
            # P + Feed-Forward coomand
            thrust_calc = (fly_vel_mag * k_ff) + (kp * real_dist)
            cmd.thrust = float(np.clip(thrust_calc, t_min, t_max))
            cmd.yaw = np.arctan2(diff_to_predicted[1], diff_to_predicted[0])
            cmd.pitch = np.arctan2(diff_to_predicted[2], np.linalg.norm(diff_to_predicted[:2]))

        self.cmd_pub.publish(cmd)
        self.publish_prediction_marker(p_predicted)

    def publish_success(self):
        duration = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.get_logger().info(f"Catch accomplished, time: {duration:.2f}s")
        msg = PointStamped()
        msg.header.frame_id, msg.header.stamp = "map", self.get_clock().now().to_msg()
        msg.point.x, msg.point.y, msg.point.z = self.drone_pos.tolist()
        self.click_pub.publish(msg)
        self.get_logger().info(f"Catch Coodinate: [{msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}] ")

    def publish_prediction_marker(self, pos):
        m = Marker()
        m.header.frame_id, m.header.stamp = "map", self.get_clock().now().to_msg()
        m.id, m.type, m.scale.x = 99, Marker.SPHERE, 0.2
        m.scale.y = m.scale.z = 0.2
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos.tolist()
        m.color.a, m.color.r, m.color.g, m.color.b = 1.0, 1.0, 1.0, 0.0 # Yellow
        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VelocityPursuitController())
    rclpy.shutdown()