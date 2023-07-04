#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import math
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Path, Odometry 
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion, PoseStamped, TwistWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped

from deltri.refer_def import*

# paramters
dt = 0.1
# k =0.5
stanley_k = 1.0 # control gain  
lateral_k = 0.5 # 횡방향
longitudinal_k = 6 # 종방향

# ERP42 PARAMETERS
LENGTH = 1.600
WIDTH = 1.160

max_steering = np.radians(22) # max_angle = 30 degree

class Fast_stanley_control(Node) :

  def __init__(self) :
    super().__init__('stanley_fast_control')

    self.local_path_sub = self.create_subscription(Path,'/Local_path', self.local_path_callback, 10)
    self.global_path_sub = self.create_subscription(Path,'/global_path', self.global_path_callback, 10)
    self.odom_sub = self.create_subscription(Odometry,'/odom1', self.odometry_callback, 10)

    self.acker_pub = self.create_publisher(AckermannDriveStamped, '/Ackerman', 10)
    self.timer = self.create_timer(0.1, self.timer_callback)
    
    self.global_flag = False
    self.local_map_x, self.local_map_y, self.local_map_yaw = [], [], []
    self.speed, self.steer, self.brake = 0.0, 0.0, 0.0
    self.global_map_x, self.global_map_y, self.global_map_yaw = [], [], []
    self.target_speed = 0.0
    
  def timer_callback(self) : # path, odom 값 받아와서 차량 trajectory, steer값 보냄
    # path 정해주는 코드       
    if self.global_flag :
      steer, theta2 = self.stanley_control(self.x, self.y, self.yaw, self.present_speed, self.global_map_x, self.global_map_y, self.global_map_yaw, 1.06) # 횡방향 제어
    else :
      steer, theta2 = self.stanley_control(0, 0, 0, self.present_speed, self.local_map_x, self.local_map_y, self.local_map_yaw, -0.17) # 횡방향 제어
      
    target_speed, brake = self.longitudinal_control(theta2) # 속도, 가속도 제어 
    self.pub_Ackerman(target_speed, steer, brake)

  def longitudinal_control(self, theta, present_speed):
    self.target_speed = 5-3.5*theta/90 # 목표 속도 
    
    if self.target_speed < 1.5 :
      target_speed = 1.5
    
    elif abs(present_speed - self.target_speed) < 0.3 :
      target_speed = self.target_speed      
    
    elif present_speed < self.target_speed :
      target_speed = 5
    
    elif present_speed >= self.target_speed :
      target_speed = 0
      brake = 0.1
      
    # 현재속도, 목표 속도, path의 길이 세트2의 크기 따라서 거기 속도 제어 ?? 
    return target_speed, brake

  def stanley_control(self, x, y, yaw, v, map_xs, map_ys, map_yaws, L) : # 1.04 0.713
    global stanley_k, lateral_k, longitudinal_k

    min_dist = 1e9 # 10에 9 승
    min_dist_lateral_k = 1e9
    min_dist_longitudinal_k = 1e9  
    min_index = 0
    n_points = len(map_xs) # path의 점 갯수
    front_x = x + L * np.cos(yaw) # 앞바퀴를 기준으로 계산함
    front_y = y + L * np.sin(yaw)

    # find the nearest waypoint index
    for i in range(n_points): # 가장 가까운 점의 i를 찾아냄
      dx = front_x - map_xs[i]      
      dy = front_y - map_ys[i]      
      dist = np.sqrt(dx * dx + dy * dy)
      
      if dist > lateral_k*v and dist < min_dist_lateral_k :
        min_dist_lateral_k = dist
        k_velocity_index = i
      
      if dist > longitudinal_k*v and dist < min_dist_longitudinal_k :
        look_ahead_distance_index = i
        min_index = i
      
      if dist < min_dist:
          min_dist = dist
          min_index = i
    
    # compute cte at front axle
    map_x = map_xs[min_index]
    map_y = map_ys[min_index]
    map_yaw = map_yaws[min_index]
    dx = map_x - front_x
    dy = map_y - front_y    
    
    perp_vec = [np.cos(map_yaw + np.pi/2), np.sin(map_yaw + np.pi/2)]
       
    cte = np.dot([dx, dy], perp_vec) # Cross track error   

    # yaw_term = normalize_angle(map_yaw - yaw) * np.sin(np.pi/2 / (1+v/5))   
    theta2 = normalize_angle(map_yaws[look_ahead_distance_index] - yaw)
    yaw_term = normalize_angle(map_yaws[k_velocity_index] - yaw)
    cte_term = np.arctan2(stanley_k*cte, 2.0) if v!=0 else 0 # cross track error v = m/s (y, x)
    
    w_yaw = 1
    w_cte = 1
    
    # steering
    self.steer = w_yaw * yaw_term + w_cte * cte_term
    self.steer = np.clip(self.steer, -max_steering, max_steering) 
    #print('yaw', round(-yaw_term*180/np.pi,1),round(map_yaw*180/np.pi,1),round(yaw*180/np.pi,1),'cte', round(-cte_term*180/np.pi,1),'steer', round(-steer*180/np.pi,1))
    return self.steer, theta2 
  
  def local_path_callback(self, msg):
    #os.system("rosnode kill "+"/darknet_ros")
    #os.system("rosnode kill "+"/lidar_camera_node")
    print('get_local_path_callback')
    for i in range(len(msg.poses)-1):
      self.local_map_x.append(msg.poses[i].pose.position.x)
      self.local_map_y.append(msg.poses[i].pose.position.y)
      self.local_map_yaw.append(euler_from_quaternion(msg.poses[i].pose.orientation.x,msg.poses[i].pose.orientation.y,msg.poses[i].pose.orientation.z,msg.poses[i].pose.orientation.w))

  def global_path_callback(self, msg):
    self.global_flag = True
    print('get_global_path_callback')
    for i in range(len(msg.poses)-1) : 
      self.global_map_x.append(msg.poses[i].pose.position.x)
      self.global_map_y.append(msg.poses[i].pose.position.y)
      self.global_map_yaw.append(euler_from_quaternion(msg.poses[i].pose.orientation.x,msg.poses[i].pose.orientation.y,msg.poses[i].pose.orientation.z,msg.poses[i].pose.orientation.w))

  def odometry_callback(self, data): 
    self.x = data.pose.pose.position.x
    self.y = data.pose.pose.position.y
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    self.present_speed = np.sqrt(vx**2+vy**2)
    self.z = data.pose.pose.orientation.z
    self.w = data.pose.pose.orientation.w
    self.yaw = euler_from_quaternion( data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w )

  def pub_Ackerman(self,target_speed, steer, brake):
    acker = AckermannDriveStamped()
    acker.header.stamp = self.get_clock().now().to_msg()
    acker.drive.speed = target_speed
    acker.drive.steering_angle =steer
    acker.drive.jerk = brake    
    self.acker_pub.publish(acker)

def main(args=None):
  rclpy.init(args=args)
  fast_stanley_publisher = Fast_stanley_control()
  rclpy.spin(fast_stanley_publisher)
  fast_stanley_publisher.destroy_node()
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()