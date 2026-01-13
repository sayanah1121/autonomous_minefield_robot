import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import serial
import math
import tf2_ros

class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')
        
        # --- Robot Constants (MEASURE YOUR ROBOT!) ---
        self.wheel_base = 0.20  # Distance between wheels (meters)
        self.wheel_radius = 0.03 # Radius of wheel (meters)
        self.ticks_per_rev = 340 # Encoder ticks per revolution
        
        # --- Serial Connection ---
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Connected to Arduino on /dev/ttyUSB0")
        except:
            self.get_logger().error("Could not connect to Arduino!")

        # --- Publishers ---
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.metal_pub = self.create_publisher(Int32, 'metal_detector_raw', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Subscribers ---
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        # --- Odometry Variables ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_l_ticks = 0
        self.prev_r_ticks = 0
        
        # --- Timer ---
        self.create_timer(0.05, self.update_loop) # 20Hz

    def cmd_callback(self, msg):
        # Simple Mapping: Twist -> PWM (Very basic open loop)
        # In a real robot, you would send velocity targets (m/s) and use PID on Arduino
        pwm_lin = int(msg.linear.x * 200) 
        pwm_ang = int(msg.angular.z * 100)
        command = f"{pwm_lin},{pwm_ang}\n"
        if hasattr(self, 'ser'):
            self.ser.write(command.encode('utf-8'))

    def update_loop(self):
        if not hasattr(self, 'ser') or self.ser.in_waiting == 0: return

        try:
            line = self.ser.readline().decode('utf-8').strip()
            parts = line.split(',')
            if len(parts) == 3:
                l_ticks = int(parts[0])
                r_ticks = int(parts[1])
                metal_val = int(parts[2])

                # Publish Sensor Data
                metal_msg = Int32()
                metal_msg.data = metal_val
                self.metal_pub.publish(metal_msg)

                # Calculate Odometry
                self.compute_odometry(l_ticks, r_ticks)
                
        except ValueError:
            pass

    def compute_odometry(self, l_ticks, r_ticks):
        # Delta Ticks
        dN_l = l_ticks - self.prev_l_ticks
        dN_r = r_ticks - self.prev_r_ticks
        self.prev_l_ticks = l_ticks
        self.prev_r_ticks = r_ticks

        # Distance per tick
        dist_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_l = dN_l * dist_per_tick
        d_r = dN_r * dist_per_tick

        # Robot movement
        d_center = (d_l + d_r) / 2.0
        d_theta = (d_r - d_l) / self.wheel_base

        # Update Pose
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        self.theta += d_theta

        # Publish Odom Message
        q = self.euler_to_quaternion(0, 0, self.theta)
        
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        self.odom_pub.publish(odom)

        # Publish TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        from geometry_msgs.msg import Quaternion
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
