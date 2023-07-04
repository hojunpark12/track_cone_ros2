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
import matplotlib.tri as mtri
import numpy as np
import math
import copy

  
class make_delaunay(Node):
  def __init__(self):
    super().__init__('make_delaunay_and_path')
    
    qos_profile = QoSProfile(depth=10)
    self.rubber_point_sub = self.create_subscription(Path, '/sensor_fusion_output', self.sensor_fusion_output_callback, qos_profile)
    self.odom_sub = self.create_subscription(Odometry, '/odom1', self.odom_callback, qos_profile)
    
    self.cone_pub = self.create_publisher(MarkerArray,'/rubber_cone_point',qos_profile) # 한바퀴 돌기전 까지만 계속 publish
    self.Local_path_pub = self.create_publisher(Path,'/Local_path', qos_profile) # 첫바퀴 때 쓰는 코드고 

    self.timer = self.create_timer(0.1, self.timer_callback)
    self.odom_xy = [] 
    self.second_path_pub = False
    self.sensor_fusion_output = Path()
    self.marker_array = MarkerArray()

    self.rubber_location = {'blue' : {'x': [], 'y': [], 'state': 'not_start', 'start_point' : [0,0]},     
                            'yellow' : {'x': [], 'y': [], 'state': 'not_start', 'start_point' : [0,0]},   
                            'mid_point' : {'x': [], 'y': [], 'state': 'not_start', 'start_point' : [0,0]}}  
    
    
  def timer_callback(self):
        
    points = []
    yellow_bool = False
    blue_bool = False
    # 다른색 라바콘 안들어왔을 때 
    if self.sensor_fusion_output.poses and len(self.odom_xy) >= 1:

      for pose in self.sensor_fusion_output.poses:
        point = [
          pose.pose.position.x + self.odom_xy[0],
          pose.pose.position.y + self.odom_xy[1],
          pose.pose.position.z
        ]
        points.append(point) 
            
      # print(points, len(points))
      # 다른쪽 라바콘 색깔안들어왔을 때 들어오게 
      
      #if len(points) == 2 :  # 2개만 들어왔을 때 blue, yellow 라바콘 색 판별 이부분은 실험으로 가져가야한다. 
      #  # blue_x = , blue_y = 
      #  # yewllow_x = , yellow_y = 
      #  
      #  midpoints_x = np.empty((0,1))
      #  midpoints_x = np.hstack((midpoints_x, (points[0][0] + points[1][0]) / 2  ))
      #  midpoints_y = np.empty((0,1))
      #  midpoints_y = np.hstack((midpoints_y, (points[0][1] + points[1][1]) / 2  ))

      if len(points) > 2  : 
        # 다른색이든, 같은색이든 2개 만들어올때는 따로 처리해야한다.
        try :
          deltri = DelaunayTriPath(np.array(points), self.odom_xy)
        
          midpoints_x , midpoints_y = deltri.get_mid()
          blue_x, blue_y = deltri.get_blue()
          yellow_x, yellow_y = deltri.get_yellow()
        # 차량에서 가장 가까운거는 0, 멀리있는 건 -1 midpoint만 저장하는걸로

      
          self.pub_rubber_cone_point("blue", round(blue_x[0],2), round(blue_y[0],2))
          self.pub_rubber_cone_point("yellow", round(yellow_x[0],2), round(yellow_y[0],2) )
          self.pub_rubber_cone_point("mid_point", midpoints_x[0], midpoints_y[0])
        except :
          pass
        
        
      # 첫 바퀴일 때  path 또한 저장된  midpoint만 사용하는걸로 
      #if self.rubber_location['blue']['state'] == 'first_lap' or self.rubber_location['yellow']['state'] == 'first_lap' :
      #    midpoints_x = np.hstack((midpoints_x, self.rubber_location['mid_point']['x']))
      #    midpoints_y = np.hstack((midpoints_y, self.rubber_location['mid_point']['y']))
      #    self.make_path_bspline(midpoints_x, midpoints_y)
      #    
      ## 두 번째 바퀴일 때 
      #if self.rubber_location['blue']['state'] == 'second_lap' and self.rubber_location['yellow']['state'] == 'second_lap' :
      #    #self.make_path_bspline(self.rubber_location['mid_point']['x'], self.rubber_location['mid_point']['y'])
      #    self.second_path_pub == True

  def pub_rubber_cone_point(self, cone_color, midpoint_x, midpoint_y): # error값이 제거된 라바콘만 들어감 한개 씩 저장해주는 걸로 
    # Publish cones               
    if self.rubber_location[cone_color]['state'] == 'not_start' :
        print(cone_color, self.rubber_location[cone_color]['state'])
        self.rubber_location[cone_color]['start_point'][0] = midpoint_x # path를 만든 라바콘 중에서 맨처음꺼만 저장해줌 시도해보고 2개? 로 변경
        self.rubber_location[cone_color]['start_point'][1] = midpoint_y
        self.rubber_location[cone_color]['x'].append(midpoint_x)
        self.rubber_location[cone_color]['y'].append(midpoint_y)
        self.rubber_location[cone_color]['state'] = 'first_lap'
        
        print(cone_color, self.rubber_location[cone_color]['state'],self.rubber_location[cone_color]['start_point'][0], self.rubber_location[cone_color]['start_point'][1] )
        self.create_marker(midpoint_x, midpoint_y, 0.0, cone_color)
    if self.rubber_location[cone_color]['state'] == 'first_lap' :
      #print(cone_color, self.rubber_location[cone_color]['x'],self.rubber_location[cone_color]['x'])
      if self.check_rubber_duplication(midpoint_x, midpoint_y, cone_color ) == False :

        self.rubber_location[cone_color]['x'].append(midpoint_x)
        self.rubber_location[cone_color]['y'].append(midpoint_y) 
        print(cone_color, self.rubber_location[cone_color]['x'], self.rubber_location[cone_color]['y'])
        self.create_marker(midpoint_x, midpoint_y, 0.0, cone_color) 
      
       # 한 바퀴 다 돌기전까지는 계속 publish 해줌.    
         
    if check_far_from_start(midpoint_x, midpoint_y, self.rubber_location[cone_color]['start_point'][0], self.rubber_location[cone_color]['start_point'][1]) < 0.1 and \
                                self.rubber_location[cone_color]['state'] == 'first_lap' and \
                                len(self.rubber_location[cone_color]['x']) > 5 :
                                    
      self.rubber_location[cone_color]['state'] == 'second_lap' #  여기 밑에서 전체 라바콘 넣어서 global_path를 publish할수도있음.
      print(cone_color, self.rubber_location[cone_color]['state'])
        
  def sensor_fusion_output_callback(self, msg):
    self.sensor_fusion_output = msg
        
  def odom_callback(self,msg):
    self.odom_xy = []
    self.odom_xy.append(msg.pose.pose.position.x)
    self.odom_xy.append(msg.pose.pose.position.y)
    
  def make_path_bspline(self, midpoints_x, midpoints_y):
    if self.second_path_pub == False :
        midpoints_xy_array = np.column_stack((midpoints_x, midpoints_y))
            
        cv = midpoints_xy_array
        p = self.bspline(cv,n=300,degree=5)
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
        
        return path_x, path_y
        
  def bspline(self, cv, n=100, degree=3):
    import scipy.interpolate as si
    """ Calculate n samples on a bspline

        cv :      Array ov control vertices
        n  :      Number of samples to return
        degree:   Curve degree
    """
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
  
  def check_rubber_duplication(self, midpoint_x, midpoint_y, cone_color):
    for i in range(len(self.rubber_location[cone_color]['x'])):
        if ((self.rubber_location[cone_color]['x'][i] - midpoint_x) ** 2 + (self.rubber_location[cone_color]['y'][i] - midpoint_y) ** 2) < 0.3:
            return True
    return False
  
  def create_marker(self, x, y, z, cone_color):
      marker = Marker()
      marker.header.frame_id = 'odom'
      marker.type = Marker.SPHERE
      marker.action = Marker.ADD
      marker.pose.position.x = x
      marker.pose.position.y = y
      marker.pose.position.z = z
      marker.pose.orientation.w = 1.0
      marker.scale.x = 0.2
      marker.scale.y = 0.2
      marker.scale.z = 0.2
      
      if cone_color == 'blue':
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
      if cone_color == 'yellow' : 
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
      if cone_color == 'mid_point' :
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0 
      
      marker.id = len(self.marker_array.markers)
      self.marker_array.markers.append(marker)
      self.cone_pub.publish(self.marker_array)
    
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