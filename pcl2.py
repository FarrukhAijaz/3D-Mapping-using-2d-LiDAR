# Description : A Node in ROS2 named "pointcloud" that publilshes the pointcloud2 data points based on the current servo angle and the x, y, z coordinates calculated %
# Author: Farrukh Aijaz #
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Float64
from geometry_msgs.msg import Point32
import math
import struct

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
        self.publisher_pointcloud = self.create_publisher(
            PointCloud2,
            'pointcloud',
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
                angles1 = self.current_scan.angles
                # Initialize point cloud data
                pointcloud = PointCloud2()
                pointcloud.header = self.current_scan.header
                pointcloud.height = 1
                pointcloud.width = len(ranges)
                pointcloud.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
                pointcloud.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
                pointcloud.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))
                pointcloud.point_step = 12
                pointcloud.row_step = pointcloud.point_step * len(ranges)
                pointcloud.is_bigendian = False
                pointcloud.is_dense = True
                pointcloud.data = bytearray(pointcloud.point_step * len(ranges))

                # Populate point cloud with calculated XYZ coordinates
                for i in range(len(ranges)):
                    distance = ranges[i]
                    angle = angles1[i]
                    x = distance * math.cos(math.radians(self.ver_angle)) * math.cos(angle)
                    y = distance * math.cos(math.radians(self.ver_angle)) * math.sin(angle)
                    z = distance * math.sin(math.radians(self.ver_angle))
                    if (y < 0):
                        z = - z
                    point = Point32()
                    point.x = x
                    point.y = y
                    point.z = z
                    struct.pack_into('fff', pointcloud.data, i * pointcloud.point_step, x, y, z)

                # Publish the PointCloud2 message
                self.publisher_pointcloud.publish(pointcloud)
                self.get_logger().info("PointCloud2 message published.")

def main(args=None):
    rclpy.init(args=args)
    laser_processor = LaserDataProcessor()
    rclpy.spin(laser_processor)
    laser_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
