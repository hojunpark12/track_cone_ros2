#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, QuaternionStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from pyproj import Proj, transform, CRS, Transformer
from rclpy.clock import Clock
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math
import numpy as np

from deltri.refer_def import*

class odomPublisher(Node):
	def __init__(self):
		super().__init__('odom_pub')

		self.gpose = Odometry()
		self.gpose.header.stamp = self.get_clock().now().to_msg()
		self.gpose.header.frame_id = 'odom'

		qos_profile = QoSProfile(depth=10)
		self.gps_sub = self.create_subscription(NavSatFix, '/ublox_gps/fix', self.gps_callback, qos_profile)
		self.gps_vel_sub = self.create_subscription(TwistWithCovarianceStamped, '/ublox_gps/fix_velocity', self.gps_vel_callback, qos_profile)
		self.imu_sub = self.create_subscription(QuaternionStamped, '/filter/quaternion', self.imu_callback, qos_profile)
		#self.imu_vel_sub = self.create_subscription(Vector3Stamped,'/filter/free_acceleration', self.imu_vel_callback, qos_profile )

		#self.vel_pub = self.create_publisher(Float32, '/imu_vel', qos_profile)
		self.odom_pub = self.create_publisher(Odometry, '/odom1', qos_profile)
		self.timer = self.create_timer(0.1, self.odom_publish)

		self.velocity_msg = Float32()
		self.x_accel = 0
		self.y_accel= 0
		self.delta_t = 1/20
  
		self.last_velocity = 0.0
		self.current_time = None

	def gps_callback(self, gps):
		transformer = Transformer.from_crs('EPSG:4326', 'EPSG:5179')
		a, b = transformer.transform(gps.latitude, gps.longitude)
		# p1 = Proj(init='epsg:4326')
		# p2 = Proj(init='epsg:5179')
		# a, b = transform(p1, p2, gps.longitude, gps.latitude)

		self.gpose.pose.pose.position.x=b-962765.386350862
		self.gpose.pose.pose.position.y=a-1958988.02084955

	def gps_vel_callback(self, gps_vel):
		self.gpose.twist.twist.linear.x = gps_vel.twist.twist.linear.x
		self.gpose.twist.twist.linear.y = gps_vel.twist.twist.linear.y
		self.gpose.twist.twist.linear.z = gps_vel.twist.twist.linear.z


	def imu_callback(self, imu):
		imu_yaw = euler_from_quaternion(imu.quaternion.x, imu.quaternion.y, imu.quaternion.z, imu.quaternion.w)
		imu_yaw +=  0.27 #np.deg2rad(0) # 오차 보정
		#print(np.rad2deg(imu_yaw))
		imu_qx, imu_qy, imu_qz, imu_qw = get_quaternion_from_euler(0,0,imu_yaw)

		self.gpose.pose.pose.orientation.x= imu_qx
		self.gpose.pose.pose.orientation.y= imu_qy
		self.gpose.pose.pose.orientation.z= imu_qz
		self.gpose.pose.pose.orientation.w= imu_qw

	#def imu_vel_callback(self,imu):
	#	
	#	self.x_accel += (imu.vector.x * self.delta_t) # 보통 1초에 45-50개 정도 들어옴 
	#	self.y_accel += (imu.vector.y * self.delta_t)
#
	#	x_vel = self.last_velocity + self.x_accel * self.delta_t
	#	y_vel = self.last_velocity + self.y_accel * self.delta_t
	#	velocity_msg = Float32()
	#	velocity_msg.data = ((x_vel ** 2) + (y_vel ** 2)) ** 0.5 # 피타고라스 정리
	#	self.vel_pub.publish(velocity_msg)

	def odom_publish(self):
		print('publish')
		self.odom_pub.publish(self.gpose)
		#self.vel_pub.publish(self.velocity_msg)

	
def main(args=None):
  rclpy.init(args=args)
  node = odomPublisher()
 
  # create odom frame
  transform = TransformStamped()
  transform.header.frame_id = 'map'
  transform.child_frame_id = 'odom'

  # Broadcast the transform as a static transform
  static_broadcaster = StaticTransformBroadcaster(node)
  static_broadcaster.sendTransform(transform)

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt')
  finally :
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
	main()