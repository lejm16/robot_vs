#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import tf

class Perception:
    def __init__(self, namespace):
        self.ns = namespace
        self.bridge = CvBridge()
        
        # 内部状态
        self.current_pose = None
        self.current_image = None
        self.current_odom = None
        
        # 订阅AMCL全局位置
        self.pose_sub = rospy.Subscriber(
            "/{}/amcl_pose".format(self.ns),
            PoseWithCovarianceStamped,
            self.pose_callback
        )

        # 订阅里程计
        self.odom_sub = rospy.Subscriber(
            "/{}/odom".format(self.ns),
            Odometry,
            self.odom_callback
        )
        
        # 订阅相机图像(仿真中由Gazebo自动发布）
        # self.image_sub = rospy.Subscriber(
        #     "/{}/camera/rgb/image_raw".format(self.ns),
        #     Image,
        #     self.image_callback
        # )
        
        rospy.loginfo("[{}] 感知模块初始化完成".format(self.ns))

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("图像转换失败: {}".format(e))

    def odom_callback(self, msg):
        self.current_odom = msg