#!/usr/bin/env python3
#=====================================================================================================================
# Year: 2025
# Organization: Technical University of Eindhoven
# Author: D. Bashkaran Latha
#
# Description: This script creates a safe unicycle controller that uses the Artificial Potential Field (APF) method for navigation.
# Created on: February 11, 2025
#=====================================================================================================================

import time
import numpy as np
import threading 
from collections import deque

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf2_ros
from tf_transformations import euler_from_quaternion

from dharshan_safe_navigator import APF_tools
from dharshan_safe_navigator import unicycle_control_tools

class SafeUnicycleControl(Node):
    def __init__(self):
        """
        Initialization
        """
        super().__init__(
            node_name="safe_unicycle_control",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
            )
        
        # Default parameters
        try:
            self.declare_parameter('queue_size', 10)
            self.declare_parameter('timeout', 0.04)
            self.declare_parameter('rate', 10.0)
            self.declare_parameter('safe_distance', 0.3)
            self.declare_parameter('scan_frame', 'front_scan')
            self.declare_parameter('max_linear_speed', 0.5)
            self.declare_parameter('max_angular_speed', 1.5)
            self.declare_parameter('goal_threshold', 0.5)
        except Exception as e:
            pass

        # Get parameters 
        self.queue_size = self.get_parameter('queue_size').value
        self.timeout = self.get_parameter('timeout').value
        self.rate = self.get_parameter('rate').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.scan_frame = self.get_parameter('scan_frame').value         
        self.goal_threshold = self.get_parameter('goal_threshold').value           

        # gains
        self.lin_gain = 0.5
        self.ang_gain = 1.0
        self.start_time = None
        self.look_around_speed = 1.0        
        self.look_around_cycle_time = 5.0

        # robot, goal and obstacle positions
        self.target_frame = None
        self.robot_pose_x = 0.0
        self.robot_pose_y = 0.0
        self.robot_pose_a = 0.0
        self.goal_pose_x = None
        self.goal_pose_y = None
        self.obstacle_x = None
        self.obstacle_y = None
        
        self.laser_queue = deque(maxlen=10)
        self.window_length = 5

        self.queue_lock = threading.Lock()

        # Callback groups
        self.scan_callback_group = MutuallyExclusiveCallbackGroup()
        self.pose_callback_group = MutuallyExclusiveCallbackGroup() 
        self.odom_callback_group = MutuallyExclusiveCallbackGroup()

        # QoS Profile
        self.qos = QoSProfile(depth=1)

        self.create_subscription(
            msg_type=LaserScan,
            topic='scan',
            callback=self.__scan_callback,
            callback_group=self.scan_callback_group,
            qos_profile=self.qos
        )

        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic='odom',
            callback=self.__odom_callback,
            callback_group=self.odom_callback_group,
            qos_profile=self.qos
        )

        self.create_subscription(
            msg_type=PoseStamped,
            topic='goal_pose',
            callback=self.__goal_pose_callback,
            callback_group=self.pose_callback_group,
            qos_profile=self.qos
        )

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.T_scan_to_target_frame = None

        # Create publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', self.qos)
        self.laser_scan_pub = self.create_publisher(LaserScan, 'avg_laser_scan', self.qos)

        # # msg to be published
        self.cmd_vel = Twist()
        
        # Compile scan 
        self.create_timer(1.0/self.rate, self.obstacle_extract, callback_group=MutuallyExclusiveCallbackGroup())
        # # Create timer for synchronization
        self.create_timer(1/self.rate, self.safe_control, callback_group=MutuallyExclusiveCallbackGroup())

    def laser_scan_averaging(self, laser_data):
        while len(self.laser_queue) < self.window_length:
            time.sleep(0.001)

        laser_data_set = list(self.laser_queue)[-self.window_length:]

        range_data_set = np.vstack([scan_data.ranges for scan_data, scan_time in laser_data_set])

        
        average_range = np.median(range_data_set, axis=0)

        laser_data = LaserScan(
            ranges=average_range, 
            angle_min=laser_data.angle_min,
            angle_max=laser_data.angle_max,
            angle_increment=laser_data.angle_increment,
            time_increment=laser_data.time_increment,
            scan_time=laser_data.scan_time,
            range_min=laser_data.range_min,
            range_max=laser_data.range_max,
            header=laser_data.header
        )

        return laser_data
    
    def obstacle_extract(self):
        
        if self.laser_queue:
            with self.queue_lock:
                self.obstacle_x = None
                self.obstacle_y = None
                laser_data,_ = self.laser_queue[0]
                self.laser_queue.popleft()
                
                avg_laser_data = self.laser_scan_averaging(laser_data)

                # self.laser_scan_pub.publish(avg_laser_data)

                if self.T_scan_to_target_frame is not None:                
                    ranges = np.array(avg_laser_data.ranges)
                    print("min range: ", np.min(ranges), "and min index: ", np.argmin(ranges))
                    min_range = np.inf# self.target_frame = 'base_link'
                    min_index = 0
                    for i in range(len(ranges)):
                        r = ranges[i]

                        if r > avg_laser_data.range_min and r < avg_laser_data.range_max:
                            angle = avg_laser_data.angle_min + i * avg_laser_data.angle_increment
                            x = r * np.cos(angle)
                            y = r * np.sin(angle)
                            theta = euler_from_quaternion([
                                self.T_scan_to_target_frame.transform.rotation.x,
                                self.T_scan_to_target_frame.transform.rotation.y,
                                self.T_scan_to_target_frame.transform.rotation.z,
                                self.T_scan_to_target_frame.transform.rotation.w
                            ])[2]
                            
                            trans_x = self.T_scan_to_target_frame.transform.translation.x + x * np.cos(theta) - y * np.sin(theta)
                            trans_y = self.T_scan_to_target_frame.transform.translation.y + x * np.sin(theta) + y * np.cos(theta)

                            new_range = np.sqrt((trans_x-self.robot_pose_x)**2 + (trans_y-self.robot_pose_y)**2)

                            if new_range < min_range:
                                min_range = new_range
                                min_index = i
                    
                    print(f"Min range: {min_range} at index: {min_index}")
                    if not np.isinf(min_range):
                        min_angle = avg_laser_data.angle_min + min_index * avg_laser_data.angle_increment
                        self.obstacle_x = min_range * np.cos(min_angle)
                        self.obstacle_y = min_range * np.sin(min_angle)
                        print(f"Obstacle: {self.obstacle_x}, {self.obstacle_y}")
            

    def __scan_callback(self, msg: LaserScan): 
        if self.target_frame is not None:
            try:
                self.T_scan_to_target_frame = self.tf_buffer.lookup_transform(
                    target_frame=self.target_frame,
                    source_frame=self.scan_frame,
                    time=rclpy.time.Time(seconds=0)
                )
            except Exception as e:
                # Print all transform available
                self.get_logger().info(f"Waiting for transform: {e}")
        
        self.laser_queue.append((msg, time.time()))
               
        

    def __odom_callback(self, msg : Odometry):
        if self.target_frame is not None:
            if msg.header.frame_id == self.target_frame:
                self.robot_pose_x = msg.pose.pose.position.x
                self.robot_pose_y = msg.pose.pose.position.y
                orientation_q = msg.pose.pose.orientation
                orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                self.robot_pose_a = yaw
            
            else:
                print("resetting robot pose and killing subscriber")
                self.robot_pose_x = 0.0
                self.robot_pose_y = 0.0
                self.robot_pose_a = 0.0

                self.destroy_subscription(self.odom_sub)

    def __goal_pose_callback(self, msg : PoseStamped):
        if msg is not None:
            self.goal_pose_x = msg.pose.position.x
            self.goal_pose_y = msg.pose.position.y
            self.target_frame = msg.header.frame_id
            print(f"Goal pose: {self.goal_pose_x}, {self.goal_pose_y}, {self.target_frame}")
                    
            
    def safe_control(self):
        self.cmd_vel = Twist()  
        if ((self.goal_pose_x is not None and self.goal_pose_y is not None)):
            
            distance_to_goal = np.sqrt((self.goal_pose_x - self.robot_pose_x)**2 + (self.goal_pose_y - self.robot_pose_y)**2)
            print(f"~~~~~Distance to goal: {distance_to_goal}")
            print(f"~~~~~Obstacle: {self.obstacle_x}, {self.obstacle_y}")
            if distance_to_goal > self.goal_threshold:
                self.start_time = None
                self.queue_lock.acquire()
                if self.obstacle_x is not None and self.obstacle_y is not None:           
                    gradient = -APF_tools.gradient_navigation_potential(
                        position=[self.robot_pose_x, self.robot_pose_y],
                        goal=[self.goal_pose_x, self.goal_pose_y],
                        obstacle=[self.obstacle_x, self.obstacle_y],
                        attractive_strength=1.0,
                        repulsive_tolerance=0.0,
                        repulsive_threshold_decay=1.0
                    )
                    print("REPULSIVE Gradient: ", gradient)
                else:
                    gradient = -APF_tools.gradient_navigation_potential_attractive(
                        position=[self.robot_pose_x, self.robot_pose_y],
                        goal=[self.goal_pose_x, self.goal_pose_y],
                        strength=1.0
                    )
                    print("ATTRACTIVE Gradient: ", gradient)
                
                self.queue_lock.release()
                # Headway control setup 
                unit_delta_dist =  np.array([
                    self.goal_pose_x - self.robot_pose_x,
                    self.goal_pose_y - self.robot_pose_y
                    ])
                unit_delta_dist = unit_delta_dist / np.linalg.norm(unit_delta_dist)

                lin_vel_gain = np.dot(np.array([np.cos(self.robot_pose_a), np.sin(self.robot_pose_a)]), unit_delta_dist)
                kappa_gain = 0.05
                headway_distance = kappa_gain * distance_to_goal
                headway_pose = np.array([
                    self.robot_pose_x + headway_distance * np.cos(self.robot_pose_a),
                    self.robot_pose_y + headway_distance * np.sin(self.robot_pose_a)
                ])

                if lin_vel_gain > kappa_gain:
                    print(".......... fwd headway control ............")
                    lin_vel, ang_vel = unicycle_control_tools.fwd_headway_unicycle_twist(
                        robot_pose=[self.robot_pose_x, self.robot_pose_y, self.robot_pose_a],
                        # goal_pose=[self.goal_pose_x, self.goal_pose_y, 0.0],
                        goal_pose = [gradient[0], gradient[1], 0.0],
                        headway_pose=headway_pose,
                        head_gain=0.5
                    )
                
                else:
                    print("________ backward headway control _________")
                    lin_vel, ang_vel = unicycle_control_tools.basic_unicycle_twist(
                        robot_pose=[self.robot_pose_x, self.robot_pose_y, self.robot_pose_a],
                        goal_pose=[self.goal_pose_x, self.goal_pose_y, 0.0],
                        k_lin=1.0,
                        k_ang=1.0
                    )
                    lin_vel = 0.0

                    
                print("------BEFORE:")
                print("lin_vel: ", lin_vel)
                print("ang_vel: ", ang_vel)
                if not np.isnan(lin_vel) and not np.isnan(ang_vel):
                    lin_vel_x, ang_vel_z = unicycle_control_tools.scale_velocities(
                        lin_vel=lin_vel,
                        ang_vel=ang_vel,
                        max_lin_vel=self.max_linear_speed,
                        max_ang_vel=self.max_angular_speed
                    )
                    self.cmd_vel.linear.x = lin_vel_x
                    self.cmd_vel.angular.z = ang_vel_z
        
        # elif self.goal_pose_x is None or self.goal_pose_y is None or (self.goal_pose_x == 0.0 and self.goal_pose_y == 0.0):
        #     print("Looking around")
        #     self.cmd_vel.linear.x = 0.0
        #     if self.start_time is None:
        #         self.start_time = time.time()

        #     omega = 2* np.pi / self.look_around_cycle_time
        #     t = time.time() - self.start_time
        #     self.cmd_vel.angular.z = self.look_around_speed * np.sin(omega * t + np.pi/2)
        #     print(omega, t, omega * t, self.cmd_vel.angular.z)
            
        self.cmd_vel_pub.publish(self.cmd_vel)
        

def main(args=None):
    rclpy.init(args=args)

    node = SafeUnicycleControl()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
