# -*- coding: utf-8 -*-
#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point,PoseStamped
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path, Odometry

from deltri.refer_to_delaunay_tri import*
from deltri.refer_def import*
from ackermann_msgs.msg import AckermannDriveStamped

import matplotlib.tri as mtri
import scipy.interpolate as si
import numpy as np
import math
import copy
  
class make_delaunay(Node) :
  def __init__(self):
    super().__init__('make_delaunay_and_path')

    qos_profile = QoSProfile(depth=10)
    self.rubber_point_sub = self.create_subscription(Path, '/sensor_fusion_output', self.sensor_fusion_output_callback, qos_profile)
    self.ackermann_subscriber = self.create_subscription(AckermannDriveStamped, '/steer_fast', self.acker_callback, 10)
    self.odom_sub = self.create_subscription( Odometry, '/odom1', self.odometry_callback, 10)

    self.cone_pub = self.create_publisher(MarkerArray,'/rubber_cone_point',qos_profile) # 
    self.Local_path_pub = self.create_publisher(Path,'/Local_path', qos_profile) # 첫바퀴 때 쓰는 코드 

    self.timer = self.create_timer(0.1, self.timer_callback) 

    self.sensor_fusion_output = Path()  
    self.steering_angle = {'present_angle' : 0.0, 'target_angle' : 0.0 }   # 0.1 초당 4.4도 움직임 
    self.rubber_location = {'blue' : np.array([0,-1,0]), 'yellow' : np.array([0,1,1]), 'unknown_yellow' : np.array([0,1,1]),'unknown_blue' : np.array([0,-1,0]), 'all' : np.array([[0,1,1],[0,-1,0],[-1,1,1],[-1,-1,0],[-2,1,1],[-2,-1,0]], dtype=np.float64 ) }
    
    self.speed = 1.0

  def timer_callback(self):
    
    points, unknown = [], []
    midpoints_xy = [[-3,0],[-2,0],[-1,0],[0,0],[1,0],[2,0],[3,0]]
    
    yellow_bool, blue_bool = False, False
    self.marker_array = MarkerArray()

    if self.sensor_fusion_output.poses : 
      
      for pose in self.sensor_fusion_output.poses: # 받아올때 blue, yellow unknown class로 받아와줌 
        
        point = [ pose.pose.position.x , pose.pose.position.y , pose.pose.position.z ]          
        points.append(point) 
        
        if pose.pose.position.z == 0 : blue_bool = True
        elif pose.pose.position.z == 1: yellow_bool = True
        elif pose.pose.position.z == -1 :unknown.append(point)
        
        if pose.pose.position.x < 0.5 and abs(pose.pose.position.y) < 2.5 : # 라바콘일 가능성인거 일단 다 넣음 
          self.closest_rubber_location([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])  

      self.store_rubber_location(yellow_bool, blue_bool)

      for pose in self.rubber_location['all'] : # 여기 point에 추가 
        points.append(pose)

      if len(self.sensor_fusion_output.poses) > 2 and yellow_bool and blue_bool :
        
        deltri = DelaunayTriPath(np.array(points))
        midpoints_xy = deltri.get_mid()
        blue_xy = deltri.get_blue()
        yellow_xy = deltri.get_yellow()
        
        if midpoints_xy[0][0] > -0.5 : midpoints_xy = np.concatenate(( np.array([[-1.0, 0]]), midpoints_xy), axis=0)
        elif midpoints_xy[0][0] > -1.0 : midpoints_xy = np.concatenate(( np.array([[-1.5, 0]]), midpoints_xy), axis=0)
        if midpoints_xy[-1][0] < 0.5 : midpoints_xy = np.append(midpoints_xy, np.array([[0.5, 0]]) , axis=0)
        
        self.create_marker(blue_xy, "blue")
        self.create_marker(yellow_xy, "yellow")      
        
      if not yellow_bool and blue_bool : midpoints_xy = [[-2,0],[-1,0],[0,1],[1, 1.5],[0.5, 3],[0,4]]
      if not blue_bool and yellow_bool : midpoints_xy = [[-2,0],[-1,0],[0,-1],[1,-1.5],[0.5,-3],[0,-4]]
      
      self.create_marker(midpoints_xy, "mid_point")
      self.create_marker(points, "all")
      self.create_marker([[-30.0, 0.0]] * 20, "mid_point")
      self.make_path_bspline(midpoints_xy) # midpoint가 3개 이상 한쪽 라바콘색이 안들어오면 path를 일부로 끝으로 보내버림 
      self.cone_pub.publish(self.marker_array)
  
  def store_rubber_location(self, yellow_bool, blue_bool) : # storage and delete 한번 인식한 것에서 절대 좌표로 어떻게 활용할 것인가? 

    if len(self.rubber_location['all']) > 1 : 
      self.rubber_location['all'][:, 0] -= 0.1 * self.speed * np.cos(self.steering_angle['present_angle']) # delete rubber location that the car moved
      self.rubber_location['all'][:, 1] -= 0.1 * self.speed * np.sin(self.steering_angle['present_angle'])
      self.rubber_location['all'] = self.rubber_location['all'][self.rubber_location['all'][:, 0] >= -2.5] # if rubber cone x is smaller than -2, Delete
    
    for keys in ['blue', 'yellow', 'unknown_yellow', 'unknown_blue'] : # 색깔 별로 따로 관리해줘야함. 
      
      self.rubber_location[keys][0] -= 0.1*self.speed*np.cos(self.steering_angle['present_angle'])  # x
      self.rubber_location[keys][1] -= 0.1*self.speed*np.sin(self.steering_angle['present_angle'])  # y
      
      if (self.rubber_location[keys][0] < 0) :  # x 값이 0보다 작을 때   
        check_bool = True

        for i in range( len(self.rubber_location['all']) ) :
          if np.linalg.norm(self.rubber_location['all'][i, :2] - self.rubber_location[keys][:2] ) <= 0.5 :
            check_bool = False
            break
            
        if check_bool : 

          if keys == 'blue' :
            if self.rubber_location[keys][1] < 0 :
              self.rubber_location['all'] = np.vstack([self.rubber_location['all'], [self.rubber_location['blue']]])
          
          if keys == 'yellow' :
            if self.rubber_location[keys][1] > 0 :
              self.rubber_location['all'] = np.vstack([self.rubber_location['all'], [self.rubber_location['yellow']]])
          
          elif keys == 'unknown_left' and not yellow_bool :
            print('unknwon_left_append')
            if self.rubber_location[keys][1] > 0 :
              self.rubber_location['all'] = np.vstack([self.rubber_location['all'], [self.rubber_location[keys][0], self.rubber_location[keys][1], 1]])

          elif keys == 'unknown_right' and not blue_bool :
            print('unknown_right_append')
            if self.rubber_location[keys][1] < 0 : 
              self.rubber_location['all'] = np.vstack([self.rubber_location['all'], [self.rubber_location[keys][0], self.rubber_location[keys][1], 0]])
          
          self.rubber_location[keys] = [100,  self.rubber_location[keys][1], self.rubber_location[keys][2] ]

  def closest_rubber_location(self, points) : # 색깔별 y 값이 차에서 가장 가까운 것 하나만 저장해줌 
    
    if points[2] == 0 : cone_string = 'blue'  
    elif points[2] == 1 : cone_string = 'yellow'
    elif points[2] == -1 :
      if points[1] < 0 : cone_string = 'unknown_blue'
      elif points[1] > 0 : cone_string = 'unknown_yellow'
      
    self.rubber_location[cone_string] = points
    
  def create_marker(self, xy, cone_color) :
    for i in range(len(xy)):
      marker = Marker()
      marker.header.frame_id = 'odom'
      marker.id = len(self.marker_array.markers)
      marker.header.stamp = self.get_clock().now().to_msg()
      marker.type = Marker.SPHERE
      marker.action = Marker.ADD

      marker.pose.position.x = float(xy[i][0])
      marker.pose.position.y = float(xy[i][1])
      marker.pose.position.z = 0.0
      marker.pose.orientation.w = 1.0
      
      if cone_color == 'all' :
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
      
      else :
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        
      if cone_color == 'blue' or ( cone_color == 'all' and xy[i][2] == 0 ) : 
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

      if cone_color == 'yellow' or ( cone_color == 'all' and xy[i][2] == 1) : 
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

      if cone_color == 'all'  and xy[i][2] == -1 :
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
      
      if cone_color == 'mid_point' :
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0 
      
      self.marker_array.markers.append(marker)

  def sensor_fusion_output_callback(self, msg):
    self.sensor_fusion_output = msg
            
  def make_path_bspline(self, midpoints_xy_array):
    #midpoints_xy_array = np.column_stack((midpoints_x, midpoints_y))
        
    cv = midpoints_xy_array
    p = self.bspline(cv,n=50,degree=5)
    path_x, path_y = p.T
    
    # Publish Path
    path_msg = Path()
    path_msg.header.frame_id = 'odom'
    
    for i in range(len(path_x)):
      pose_stamped = PoseStamped()
      pose_stamped.pose.position.x = path_x[i]
      pose_stamped.pose.position.y = path_y[i]
      pose_stamped.pose.position.z = float(0)
          
      idx_next = i + 1
      if(idx_next == len(path_x)):
        idx_next = 0
      yaw = np.arctan2(path_y[idx_next]-path_y[i],path_x[idx_next]-path_x[i])
      qx_,qy_,qz_,qw_ = get_quaternion_from_euler(0,0,yaw)
          
      pose_stamped.pose.orientation.x = qx_
      pose_stamped.pose.orientation.y = qy_
      pose_stamped.pose.orientation.z = qz_
      pose_stamped.pose.orientation.w = qw_
      path_msg.poses.append(pose_stamped)
        
    self.Local_path_pub.publish(path_msg)
        
  def bspline(self, cv, n, degree=3):

        #cv :      Array ov control vertices
        #n  :      Number of samples to return
        #degree:   Curve degree
    cv = np.asarray(cv)
    count = cv.shape[0]
    # Prevent degree from exceeding count-1, otherwise splev will crash
    degree = np.clip(degree,1,count-1)
    # Calculate knot vector
    kv = np.array([0]*degree + list(range(count-degree+1)) + [count-degree]*degree,dtype='int')
    # Calculate query range
    u = np.linspace(0,(count-degree),n)
    # Calculate result
    return np.array(si.splev(u, (kv,cv.T,degree))).T    
    
  def acker_callback(self, msg) : # get speed, steer_angle
    #self.speed = msg.drive.speed
    self.steering_angle['target_angle'] = (msg.drive.steering_angle)

    if abs(self.steering_angle['target_angle'] - self.steering_angle['present_angle']) <= 0.07679: # 0.1초  차량 바퀴회전 속도 
      self.steering_angle['present_angle'] = self.steering_angle['target_angle'] 

    if self.steering_angle['target_angle'] > self.steering_angle['present_angle']:
        self.steering_angle['present_angle'] += 0.07679
    
    elif self.steering_angle['target_angle'] < self.steering_angle['present_angle']:
        self.steering_angle['present_angle'] -= 0.07679
        
    print(self.steering_angle)
    
  def odometry_callback(self, data) : 
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    self.speed = np.sqrt(vx**2+vy**2)
    #print(self.speed)
    
def main(args=None):
  rclpy.init(args=args)
  node = make_delaunay()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt')
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ =="__main__" :
    main()