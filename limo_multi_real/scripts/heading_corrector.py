#!/usr/bin/env python
# coding=utf-8
# 修复车头方向偏差问题

import rospy
import tf
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class HeadingCorrector:
    def __init__(self):
        rospy.init_node('heading_corrector', anonymous=True)
        
        # 获取参数
        self.correction_factor = rospy.get_param('~correction_factor', 1.2)  # 方向校正因子
        self.angle_threshold = rospy.get_param('~angle_threshold', 0.05)  # 角度阈值(弧度)
        
        # 创建发布者
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_corrected', Twist, queue_size=10)
        
        # 创建订阅者
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('multfodom', Float32MultiArray, self.master_pose_callback)
        rospy.Subscriber('odom', Odometry, self.slave_odom_callback)
        
        # 初始化变量
        self.master_yaw = 0.0
        self.slave_yaw = 0.0
        self.heading_diff = 0.0
        self.last_cmd_vel = Twist()
        
        rospy.loginfo("车头方向校正节点已启动")
        
    def master_pose_callback(self, msg):
        """处理主车位置信息"""
        if len(msg.data) >= 3:
            self.master_yaw = msg.data[2]
            self.calculate_heading_diff()
    
    def slave_odom_callback(self, msg):
        """处理从车里程计信息"""
        # 从四元数中提取偏航角
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.slave_yaw = euler[2]
        self.calculate_heading_diff()
    
    def calculate_heading_diff(self):
        """计算主从车头方向差异"""
        # 计算方向差
        diff = self.master_yaw - self.slave_yaw
        
        # 规范化角度差到[-pi, pi]
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        
        self.heading_diff = diff
    
    def cmd_vel_callback(self, msg):
        """处理速度命令，修正车头方向偏差"""
        self.last_cmd_vel = msg
        
        # 创建修正后的速度命令
        corrected_cmd = Twist()
        corrected_cmd.linear.x = msg.linear.x
        corrected_cmd.linear.y = msg.linear.y
        corrected_cmd.linear.z = msg.linear.z
        
        # 如果方向差异超过阈值，应用校正
        if abs(self.heading_diff) > self.angle_threshold:
            # 根据方向差异调整角速度
            # 原始角速度加上一个与方向差成比例的校正项
            correction = self.correction_factor * self.heading_diff
            corrected_cmd.angular.z = msg.angular.z + correction
            
            rospy.loginfo("应用方向校正: 原始角速度=%.2f, 方向差=%.2f, 校正后角速度=%.2f", 
                         msg.angular.z, self.heading_diff, corrected_cmd.angular.z)
        else:
            corrected_cmd.angular.z = msg.angular.z
            rospy.logdebug("方向差在阈值内，无需校正")
        
        # 发布校正后的速度命令
        self.cmd_vel_pub.publish(corrected_cmd)

if __name__ == '__main__':
    try:
        corrector = HeadingCorrector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
