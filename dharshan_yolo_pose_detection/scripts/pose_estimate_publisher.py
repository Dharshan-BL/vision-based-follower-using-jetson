#!/usr/bin/env python3
#=====================================================================================================================
# Year: 2025
# Organization: Technical University of Eindhoven
# Author: D. Bashkaran Latha
#
# Description: This script creates a safe unicycle controller that uses the Artificial Potential Field (APF) method for navigation.
# Created on: February 11, 2025
#=====================================================================================================================

from threading import Lock
import numpy as np

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Twist
import tf2_ros
from yolo_msgs.msg import DetectionArray


class PoseEstimatePublisher(Node):
    def __init__(self):
        """
        Initialization
        """
        super().__init__(
            node_name="pose_estimate_publisher",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
            )
        
        # Default parameters
        try:
            self.declare_parameter('queue_size', 10)
            self.declare_parameter('timeout', 0.04)
            self.declare_parameter('rate', 10.0)
            self.declare_parameter('score_threshold', 0.85)
        except Exception as e:
            pass

        # Get parameters 
        self.queue_size = self.get_parameter('queue_size').value
        self.timeout = self.get_parameter('timeout').value
        self.rate = self.get_parameter('rate').value
        self.score_threshold = self.get_parameter('score_threshold').value
        
        # QoS Profile
        self.qos = QoSProfile(depth=1)

        self.create_subscription(
            msg_type=DetectionArray,
            topic='detections_3d',
            callback=self.__detections_callback,
            qos_profile=self.qos
        )

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.T_scan_to_base_link = None

        # Create publisher
        self.obj_pose_pub = self.create_publisher(PoseStamped, 'obj_pose', self.qos)

        # # msg to be published
        self.obj_pose = PoseStamped()
    
                
    def __detections_callback(self, msg: DetectionArray):

        detections = msg.detections
        self.obj_pose = PoseStamped()
        self.obj_pose.header = msg.header
        self.obj_pose.header.frame_id = "base_link"
        if detections:
            for detection in detections:
                if detection.class_name == "dharshan" and detection.score > self.score_threshold:
                    self.obj_pose.pose.position.x = detection.bbox3d.center.position.x
                    self.obj_pose.pose.position.y = detection.bbox3d.center.position.y
                    self.obj_pose.pose.position.z = detection.bbox3d.center.position.z
                    self.obj_pose.pose.orientation = detection.bbox3d.center.orientation
                    break
        print("publishing " , (self.obj_pose))
        self.obj_pose_pub.publish(self.obj_pose)
   

def main(args=None):
    rclpy.init(args=args)

    node = PoseEstimatePublisher()
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
    
