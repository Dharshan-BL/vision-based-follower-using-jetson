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

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
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
            self.declare_parameter('base_link_frame', 'base_link')
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
        self.base_link_frame = self.get_parameter('base_link_frame').value
        self.scan_frame = self.get_parameter('scan_frame').value         
        self.goal_threshold = self.get_parameter('goal_threshold').value           

        # gains
        self.lin_gain = 0.5
        self.ang_gain = 1.0
        self.start_time = None
        self.wait_time = 0.5
        self.look_around_speed = 1.0        
        self.look_around_cycle_time = 5.0

        # goal and obstacle positions
        self.goal_pose_x = None
        self.goal_pose_y = None
        self.obstacle_x = None
        self.obstacle_y = None

        # Callback groups
        self.scan_callback_group = MutuallyExclusiveCallbackGroup()
        self.pose_callback_group = MutuallyExclusiveCallbackGroup() 

        # QoS Profile
        self.qos = QoSProfile(depth=1)

        self.create_subscription(
            msg_type=LaserScan,
            topic='scan',
            callback=self.__scan_callback,
            callback_group=self.scan_callback_group,
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
        self.T_scan_to_base_link = None

        # Create publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', self.qos)

        # # msg to be published
        self.cmd_vel = Twist()
        
        # # Create timer for synchronization
        self.create_timer(1/self.rate, self.safe_control, callback_group=MutuallyExclusiveCallbackGroup())
                
    def __scan_callback(self, msg: LaserScan):

        if self.T_scan_to_base_link is None:
            try:
                self.T_scan_to_base_link = self.tf_buffer.lookup_transform(
                    target_frame=self.base_link_frame,
                    source_frame=self.scan_frame,
                    time=rclpy.time.Time(seconds=0)
                )
            except Exception as e:
                # Print all transform available
                self.get_logger().info(f"Waiting for transform: {e}")
        
        self.obstacle_x = None
        self.obstacle_y = None

        if self.T_scan_to_base_link is not None:
            ranges = np.array(msg.ranges)
            min_range = np.inf
            min_index = 0
            for i in range(len(ranges)):
                r = ranges[i]
                angle = msg.angle_min + i * msg.angle_increment

                x = r * np.cos(angle)
                y = r * np.sin(angle)
                theta = euler_from_quaternion([
                    self.T_scan_to_base_link.transform.rotation.x,
                    self.T_scan_to_base_link.transform.rotation.y,
                    self.T_scan_to_base_link.transform.rotation.z,
                    self.T_scan_to_base_link.transform.rotation.w
                ])[2]
                
                trans_x = self.T_scan_to_base_link.transform.translation.x + x * np.cos(theta) - y * np.sin(theta)
                trans_y = self.T_scan_to_base_link.transform.translation.y + x * np.sin(theta) + y * np.cos(theta)

                new_range = np.sqrt(trans_x**2 + trans_y**2)

                if new_range < min_range:
                    min_range = new_range
                    min_index = i
            
            if not np.isinf(min_range):
                min_angle = msg.angle_min + min_index * msg.angle_increment

                self.obstacle_x = min_range * np.cos(min_angle)
                self.obstacle_y = min_range * np.sin(min_angle)
        
    def __goal_pose_callback(self, msg : PoseStamped):
        if msg is not None:
            self.goal_pose_x = msg.pose.position.x
            self.goal_pose_y = msg.pose.position.y

            
    def safe_control(self):
        self.cmd_vel = Twist()  
        self.look_around = True
        if ((self.goal_pose_x is not None and self.goal_pose_y is not None) and
            self.goal_pose_x != 0.0 or self.goal_pose_y != 0.0):

            distance_to_goal = np.sqrt(self.goal_pose_x**2 + self.goal_pose_y**2)

            if distance_to_goal > self.goal_threshold:
                self.look_around = False
                self.start_time = None
                print(f"Detected you at: {self.goal_pose_x}, {self.goal_pose_y}")

                # if self.obstacle_x is not None and self.obstacle_y is not None:           
                #     gradient = -APF_tools.gradient_navigation_potential(
                #         position=[0.0, 0.0],
                #         goal=[self.goal_pose_x, self.goal_pose_y],
                #         obstacle=[self.obstacle_x, self.obstacle_y],
                #         attractive_strength=1.0,
                #         repulsive_tolerance=self.safe_distance,
                #         repulsive_threshold_decay=1.0
                #     )
                # else:
                gradient = -APF_tools.gradient_navigation_potential_attractive(
                    position=[0.0, 0.0],
                    goal=[self.goal_pose_x, self.goal_pose_y],
                    strength=1.0
                )
                
                print(gradient)

                lin_vel, ang_vel = unicycle_control_tools.unicycle_gradient_ctrl_2D(
                    gradient=gradient,
                    yaw=0.0,
                    lin_gain=self.lin_gain,
                    ang_gain=self.ang_gain
                )
                
                if not np.isnan(lin_vel[0]) and not np.isnan(ang_vel[0]):
                    lin_vel_x, ang_vel_z = unicycle_control_tools.scale_velocities(
                        lin_vel=lin_vel[0],
                        ang_vel=ang_vel[0],
                        max_lin=self.max_linear_speed,
                        max_ang=self.max_angular_speed
                    )
                    self.cmd_vel.linear.x = lin_vel_x
                    self.cmd_vel.angular.z = ang_vel_z

            else:
                print("Goal reached")

        
        if self.look_around:
            print("Looking around")
            self.cmd_vel.linear.x = 0.0
            if self.start_time is None:
                self.start_time = time.time()

            omega = 2* np.pi / self.look_around_cycle_time
            t = time.time() - self.start_time
            self.cmd_vel.angular.z = self.look_around_speed * np.sin(omega * t + np.pi/2)
            print(omega, t, omega * t, self.cmd_vel.angular.z)
            
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
    
