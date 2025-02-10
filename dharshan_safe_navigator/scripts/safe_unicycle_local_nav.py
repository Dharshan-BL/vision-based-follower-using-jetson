#!/usr/bin/env python3
#=====================================================================================================================
# Project: MSc. Thesis
# Title: Keyframe-Based Robot Navigation: Integrated Mapping, Planning and Control using Graph of Star-Convex Regions
# Year: 2024
# Organization: Technical University of Eindhoven
# Author: D. Bashkaran Latha
# Advisor: Dr. Ömür Arslan
#
# Description: This script creates a keyframe from the pose and laser scan messages and adds it to the keyframe graph for Matplotlib visualization.
# Created on: November 7, 2024
# Author Notes: I wish I didn't skip my coffee before writing this script.
#=====================================================================================================================

from threading import Lock
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as pltPolygon
import pygeos as pg
import time, copy
from queue import Queue
from collections import deque

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import LaserScan

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
            self.declare_parameter('robot_radius', 0.3)
            self.declare_parameter('goal_tolerance', 0.2)
            self.declare_parameter('max_linear_speed', 1.0)
            self.declare_parameter('max_angular_speed', 1.5)
        except Exception as e:
            pass

        # Get parameters 
        self.queue_size = self.get_parameter('queue_size').value
        self.timeout = self.get_parameter('timeout').value
        self.rate = self.get_parameter('rate').value
        self.safe_distance = self.get_parameter('safe_distance').value
        
        # Thread locker to not add to queue while processing
        self.queue_lock = Lock()                            

        # pose and laser messages
        self.pose_msg = None
        self.laser_msg = None

        # Keyframe related attributes
        self.keyframe_length = 0

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
            msg_type=Pose,
            topic='pose',
            callback=self.__goal_pose_callback,
            callback_group=self.pose_callback_group,
            qos_profile=self.qos
        )
        
        # Create publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', self.qos)

        # # msg to be published
        self.cmd_vel = Twist()
        
        # # Create timer for synchronization
        self.create_timer(1/self.rate, self.safe_control, callback_group=MutuallyExclusiveCallbackGroup())
                
    def __scan_callback(self, msg):
        if msg is not None:
                self.laser_msg = msg
            
    
    def __goal_pose_callback(self, msg):
        if msg is not None:
            self.goal_pose = msg

    def safe_control(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    topic_handler = SafeUnicycleControl()
    executor = MultiThreadedExecutor()
    executor.add_node(topic_handler)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        topic_handler.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
