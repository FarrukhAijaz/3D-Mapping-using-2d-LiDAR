# Description : A Node in ROS2 named "servo_angle" that publilshes the current servo angle and the x, y, z coordinates using the servo angle %
# Author: Farrukh Aijaz#
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import math

class LaserDataProcessor(Node):
    def __init__(self):
        super().__init__('laser_data_processor')
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        self.subscription_servo = self.create_subscription(
            Float64,
            'servo_angle',
            self.servo_callback,
            10)
        self.ver_angle = None  # Initialize servo angle
        self.current_scan = None  # Initialize current laser scan data
        self.processing_timer = self.create_timer(0.1, self.process_data)  # Adjust the interval as needed

        # Flag to control processing
        self.servo_data_received = False

    def laser_callback(self, msg):
        # Store the latest laser scan data
        self.current_scan = msg

    def servo_callback(self, msg):
        # Update servo angle
        self.ver_angle = msg.data
        # Set flag to indicate servo data received
        self.servo_data_received = True

    def process_data(self):
        # Process laser data only when servo data is received
        if self.servo_data_received:
            # Reset flag
            self.servo_data_received = False
            # Check if both laser scan data and servo angle are available
            if self.current_scan is not None and self.ver_angle is not None:
                # Process laser scan data
                ranges = self.current_scan.ranges
                angles = [self.current_scan.angle_min + i * self.current_scan.angle_increment for i in range(len(ranges))]
                coordinates = np.zeros((len(ranges), 3)) 
                # Perform calculations
                for i in range(len(ranges)):
                    distance = ranges[i]
                    angle = angles[i]
                    # Adjust calculations based on servo angle
                    x = distance * math.cos(math.radians(self.ver_angle)) * -1 * (math.cos(angle))
                    y = distance * math.cos(math.radians(self.ver_angle)) * math.sin(angle)
                    z = distance * math.sin(math.radians(self.ver_angle))
                    # Print distance, angle, and coordinates
                    self.get_logger().info(f'Point {i+1}: Distance: {distance:.2f} meters, Angle: {math.degrees(angle):.4f} degrees, Servo Angle={self.ver_angle} degrees X: {x:.4f}, Y: {y:.4f}, Z: {z:.4f}')

def main(args=None):
    rclpy.init(args=args)
    laser_processor = LaserDataProcessor()
    rclpy.spin(laser_processor)
    laser_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
