#!/usr/bin/env python
# coding=utf-8
# 从车初始位置校正功能

import rospy
import tf
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry

class InitialPoseCalibrator:
    def __init__(self):
        rospy.init_node('initial_pose_calibrator', anonymous=True)
        
        # 获取参数
        self.slave_x = rospy.get_param('~slave_x', -0.8)
        self.slave_y = rospy.get_param('~slave_y', 0.8)
        self.calibration_timeout = rospy.get_param('~calibration_timeout', 10.0)  # 校准超时时间(秒)
        self.calibration_distance = rospy.get_param('~calibration_distance', 0.1)  # 校准距离阈值
        self.calibration_angle = rospy.get_param('~calibration_angle', 0.1)  # 校准角度阈值(弧度)
        
        # 创建发布者
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        
        # 创建订阅者
        rospy.Subscriber('multfodom', Float32MultiArray, self.master_pose_callback)
        rospy.Subscriber('odom', Odometry, self.slave_odom_callback)
        
        # 初始化变量
        self.master_x = 0.0
        self.master_y = 0.0
        self.master_yaw = 0.0
        self.slave_x = 0.0
        self.slave_y = 0.0
        self.slave_yaw = 0.0
        self.master_data_received = False
        self.slave_data_received = False
        self.calibration_done = False
        
        # TF监听器
        self.tf_listener = tf.TransformListener()
        
        rospy.loginfo("初始位置校正节点已启动")
        
        # 等待数据
        self.wait_for_data()
        
        # 执行校正
        if self.master_data_received and self.slave_data_received:
            self.calibrate_position()
        else:
            rospy.logerr("未能接收到主车或从车数据，校正失败")
    
    def master_pose_callback(self, msg):
        """处理主车位置信息"""
        if len(msg.data) >= 3:
            self.master_x = msg.data[0]
            self.master_y = msg.data[1]
            self.master_yaw = msg.data[2]
            self.master_data_received = True
    
    def slave_odom_callback(self, msg):
        """处理从车里程计信息"""
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
        self.slave_data_received = True
    
    def wait_for_data(self):
        """等待接收主车和从车数据"""
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10Hz
        
        while (not self.master_data_received or not self.slave_data_received) and \
              (rospy.Time.now() - start_time).to_sec() < self.calibration_timeout:
            rospy.loginfo("等待数据: 主车数据=%s, 从车数据=%s", 
                         "已接收" if self.master_data_received else "未接收",
                         "已接收" if self.slave_data_received else "未接收")
            rate.sleep()
    
    def calibrate_position(self):
        """校正从车初始位置"""
        rospy.loginfo("开始校正从车初始位置")
        
        # 计算目标位置（相对于主车）
        target_x_global = self.master_x + self.slave_x * math.cos(self.master_yaw) - self.slave_y * math.sin(self.master_yaw)
        target_y_global = self.master_y + self.slave_x * math.sin(self.master_yaw) + self.slave_y * math.cos(self.master_yaw)
        target_yaw = self.master_yaw
        
        # 计算当前位置与目标位置的差距
        dx = target_x_global - self.slave_x
        dy = target_y_global - self.slave_y
        dyaw = target_yaw - self.slave_yaw
        
        # 规范化角度差
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi
        
        distance = math.sqrt(dx*dx + dy*dy)
        
        rospy.loginfo("当前位置: x=%.2f, y=%.2f, yaw=%.2f", self.slave_x, self.slave_y, self.slave_yaw)
        rospy.loginfo("目标位置: x=%.2f, y=%.2f, yaw=%.2f", target_x_global, target_y_global, target_yaw)
        rospy.loginfo("位置差距: 距离=%.2f, 角度=%.2f", distance, dyaw)
        
        # 如果差距较大，发布初始位置校正
        if distance > self.calibration_distance or abs(dyaw) > self.calibration_angle:
            rospy.loginfo("位置差距较大，进行校正")
            
            # 创建初始位置消息
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.pose.position.x = target_x_global
            pose.pose.pose.position.y = target_y_global
            pose.pose.pose.position.z = 0.0
            
            # 将目标偏航角转换为四元数
            q = tf.transformations.quaternion_from_euler(0, 0, target_yaw)
            pose.pose.pose.orientation.x = q[0]
            pose.pose.pose.orientation.y = q[1]
            pose.pose.pose.orientation.z = q[2]
            pose.pose.pose.orientation.w = q[3]
            
            # 设置协方差
            pose.pose.covariance[6*0+0] = 0.5 * 0.5  # x方向协方差
            pose.pose.covariance[6*1+1] = 0.5 * 0.5  # y方向协方差
            pose.pose.covariance[6*3+3] = math.pi/12.0 * math.pi/12.0  # 角度协方差
            
            # 发布初始位置
            self.initial_pose_pub.publish(pose)
            rospy.loginfo("已发布初始位置校正")
            
            # 短暂停止从车
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
            rospy.sleep(1.0)  # 等待1秒让校正生效
            
            self.calibration_done = True
        else:
            rospy.loginfo("位置差距在可接受范围内，无需校正")
            self.calibration_done = True

if __name__ == '__main__':
    try:
        calibrator = InitialPoseCalibrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
