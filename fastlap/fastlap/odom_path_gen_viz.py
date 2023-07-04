#! /usr/bin/env python3
#-*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import math

from deltri.refer_def import*

class odom_viz(Node):
  def __init__(self):
    super().__init__('odom_path_gen_viz')
    self.marker_pub=self.create_publisher(Marker, "/rviz/odom_marker_hojun", 1)
    self.trajectory_pub = self.create_publisher(Path, '/rviz/odom_trajectory', 10)
    self.path_pub = self.create_publisher(Path, '/fastlap_path', 10)
    self.path_rviz_pub = self.create_publisher(Path, '/rviz/fastlap_path', 10)
    self.odom_sub=self.create_subscription(Odometry, "/odom1", self.odometry_callback, 10)

    self.x, self.y, self.v, self.yaw, self.z, self.w = 0.0, 0.0, 0, 0, 0, 0
    self.tx, self.ty, self.tw = [], [], []
    self.start_flag =False
    
    self.x_start, self.y_start = 0, 0
    self.x_odom, self.y_odom, self.y_path = [], [], []
    self.i=0 # i = 0 시작전, 1 : 도는 중 ,2 : 끝남  
    self.qx_odom, self.qy_odom, self.qz_odom, self.qw_odom = [], [], [], []

    self.ma = None
    self.points = 0
    self.path = Path()

  def check_far_from_start(self,x,y,xs,ys):
      
      dist = np.sqrt((x-xs)**2 + (y-ys)**2)
      return dist

  def odometry_callback(self, data):
    self.odom_callback(data)
    self.msg_pub()
    self.trajectory()
		# sensor_msgs/Imu.msg

    self.x = data.pose.pose.position.x
    self.y = data.pose.pose.position.y
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    self.v = np.sqrt(vx**2+vy**2)
    self.z = data.pose.pose.orientation.z
    self.w = data.pose.pose.orientation.w
    orientation_list = [
      data.pose.pose.orientation.x,
      data.pose.pose.orientation.y,
      data.pose.pose.orientation.z,
      data.pose.pose.orientation.w
      ]

    roll, pitch, self.yaw = euler_from_quaternion(
      orientation_list[0],
      orientation_list[1],
      orientation_list[2],
      orientation_list[3]
      )

  def odom_callback(self,msg):
      #global i, x_start, y_start, start_flag
      if self.i==0:
        self.x_start = msg.pose.pose.position.x
        self.y_start = msg.pose.pose.position.y
        self.x_odom.append(self.x_start)
        self.y_odom.append(self.y_start)
        self.i=self.i+1
        print('start',self.i,self.start_flag)

      if self.check_far_from_start(msg.pose.pose.position.x,msg.pose.pose.position.y,self.x_start,self.y_start) > 0.5:
        self.start_flag = True

      if self.check_far_from_start(msg.pose.pose.position.x,msg.pose.pose.position.y,self.x_odom[-1],self.y_odom[-1]) > 0.2:  
        self.x_odom.append(msg.pose.pose.position.x)
        self.y_odom.append(msg.pose.pose.position.y)
          
      if self.start_flag == True and self.check_far_from_start(msg.pose.pose.position.x, msg.pose.pose.position.y,self.x_start,self.y_start) <0.5 and self.i==1:
        
        self.path.header.frame_id = 'odom'
        for j in range(len(self.x_odom)-1):
          pose = PoseStamped()
          pose.pose.position.x = self.x_odom[j]
          pose.pose.position.y = self.y_odom[j]
          pose.pose.position.z = float(0)
          yaw = np.arctan2(self.y_odom[j+1]-self.y_odom[j],self.x_odom[j+1]-self.x_odom[j] )
          qx_,qy_,qz_,qw_ = get_quaternion_from_euler(0,0,yaw)
          pose.pose.orientation.x = qx_
          pose.pose.orientation.y = qy_
          pose.pose.orientation.z = qz_
          pose.pose.orientation.w = qw_
          self.path.poses.append(pose)

        print('end')
        self.i=self.i+1
        self.path_pub.publish(self.path)
      
      if self.start_flag:
        self.path_rviz_pub.publish(self.path)

  def msg_pub(self):
    quat = quaternion_from_euler(0, 0, self.yaw)

    m = Marker()
    m.header.frame_id = "odom"
    m.header.stamp = self.get_clock().now().to_msg()
    m.id = 1
    m.type = m.CUBE
	  #m.pose.position.x = x + 1.3 * math.cos(yaw)
	  #m.pose.position.y = y + 1.3 * math.sin(yaw)
    m.pose.position.x = self.x
    m.pose.position.y = self.y
    m.pose.position.z = 0.3

    m.pose.orientation.x = quat[0]
    m.pose.orientation.y = quat[1]
    m.pose.orientation.z = quat[2]
    m.pose.orientation.w = quat[3]

    m.scale.x = 1.600
    m.scale.y = 1.160
    m.scale.z = 1.645
    m.color.r = 17 / 255.0
    m.color.g = 17 / 255.0
    m.color.b = 252 / 255.0
    m.color.a = 0.97
    self.marker_pub.publish(m)
  
  def trajectory(self):
    if self.start_flag:
        path = Path()
        path.header.frame_id = 'odom'
        self.tx.append(self.x)
        self.ty.append(self.y)
        theta = 2.0 * np.arctan2(self.z, self.w)

        if theta < 0.0:theta += 2.0 * np.pi
        self.tw.append(theta)

        if len(self.tx) > 2 :
            # Path 메시지 구성
            path = Path()
            path.header.frame_id = "odom"
            path.header.stamp = self.get_clock().now().to_msg()
            path_length = min(len(self.tx), len(self.ty), len(self.tw))
            m = path_length - 2000 if len(self.tx) >= 2000 else 0
            for n in range(m, path_length):
                # Appending to Visualization Path
                vpose = PoseStamped()
                vpose.header.frame_id = "odom"
                vpose.header.stamp = self.get_clock().now().to_msg()
                vpose.pose.position.x = self.tx[n]
                vpose.pose.position.y = self.ty[n]
                vpose.pose.position.z = 0.0
                vpose.pose.orientation = yaw_to_quaternion(np.pi * 0.5 - self.tw[n])
                path.poses.append(vpose)
            self.trajectory_pub.publish(path)


def main(args=None):
  rclpy.init(args=args)
  node = odom_viz()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt')
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__" :
  main()