#!/usr/bin/env python
# coding=utf-8
# 为从车创建TF转换，不依赖navigation

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import math

class TFBroadcaster:
    def __init__(self):
        rospy.init_node('tf_broadcaster', anonymous=True)
        
        # 获取命名空间
        self.robot_namespace = rospy.get_namespace()
        if self.robot_namespace == '/':
            self.odom_frame = "odom"
            self.base_frame = "base_link"
        else:
            # 移除开头和结尾的'/'
            if self.robot_namespace.startswith('/'):
                self.robot_namespace = self.robot_namespace[1:]
            if self.robot_namespace.endswith('/'):
                self.robot_namespace = self.robot_namespace[:-1]
                
            self.odom_frame = self.robot_namespace + "/odom"
            self.base_frame = self.robot_namespace + "/base_link"
        
        # 创建TF广播器
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        # 订阅里程计信息
        rospy.Subscriber('odom', Odometry, self.odom_callback)
        
        # 订阅主车位置信息
        rospy.Subscriber('multfodom', Float32MultiArray, self.master_pose_callback)
        
        # 初始化变量
        self.master_x = 0.0
        self.master_y = 0.0
        self.master_yaw = 0.0
        self.slave_x = 0.0
        self.slave_y = 0.0
        self.slave_yaw = 0.0
        
        # 获取从车相对于主车的期望位置
        self.target_x = rospy.get_param('~slave_x', -0.8)
        self.target_y = rospy.get_param('~slave_y', 0.8)
        
        # rospy.loginfo("TF Broadcaster initialized. Robot namespace: %s", self.robot_namespace)
        # rospy.loginfo("Odom frame: %s, Base frame: %s", self.odom_frame, self.base_frame)
        # rospy.loginfo("Target position: x=%f, y=%f", self.target_x, self.target_y)
        
    def odom_callback(self, msg):
        """处理从车里程计信息，更新从车位置"""
        self.slave_x = msg.pose.pose.position.x
        self.slave_y = msg.pose.pose.position.y
        
        # 从四元数中提取偏航角
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.slave_yaw = euler[2]
        
        # 发布从车的TF转换
        self.publish_slave_tf()
        
    def master_pose_callback(self, msg):
        """处理主车位置信息"""
        if len(msg.data) >= 3:
            self.master_x = msg.data[0]
            self.master_y = msg.data[1]
            self.master_yaw = msg.data[2]
            
            # 发布主车到从车的TF转换
            self.publish_master_slave_tf()
    
    def publish_slave_tf(self):
        """发布从车odom到base_link的TF转换"""
        current_time = rospy.Time.now()
        
        # 发布从车odom到base_link的转换
        self.tf_broadcaster.sendTransform(
            (self.slave_x, self.slave_y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self.slave_yaw),
            current_time,
            self.base_frame,
            self.odom_frame
        )
        
        # 发布map到从车odom的转换
        self.tf_broadcaster.sendTransform(
            (0, 0, 0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            current_time,
            self.odom_frame,
            "map"
        )
    
    def publish_master_slave_tf(self):
        """发布主车到从车的TF转换"""
        current_time = rospy.Time.now()
        
        # 计算主车base_link到从车期望位置的转换
        # 这里需要考虑主车的朝向
        target_x_rotated = self.target_x * math.cos(self.master_yaw) - self.target_y * math.sin(self.master_yaw)
        target_y_rotated = self.target_x * math.sin(self.master_yaw) + self.target_y * math.cos(self.master_yaw)
        
        # 发布主车base_link到从车期望位置的转换
        self.tf_broadcaster.sendTransform(
            (target_x_rotated, target_y_rotated, 0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            current_time,
            "slave_target",
            "limo1/base_link"
        )
        
        # 发布主车odom到主车base_link的转换
        self.tf_broadcaster.sendTransform(
            (self.master_x, self.master_y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self.master_yaw),
            current_time,
            "limo1/base_link",
            "limo1/odom"
        )
        
        # 发布map到主车odom的转换
        self.tf_broadcaster.sendTransform(
            (0, 0, 0),
            tf.transformations.quaternion_from_euler(0, 0, 0),
            current_time,
            "limo1/odom",
            "map"
        )

if __name__ == '__main__':
    try:
        tf_broadcaster = TFBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
