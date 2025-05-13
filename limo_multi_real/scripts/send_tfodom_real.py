#!/usr/bin/env python
# coding=utf-8
# 主车发送主车坐标以及主车速度给从车 - 适配实际环境，使用UDP通信

import math
import rospy
import tf
import socket
import struct
from numpy import array
from nav_msgs.msg import Odometry  

odom_vx = 0
odom_vy = 0
odom_az = 0

def odom_callback(msg):
    global odom_vx, odom_vy, odom_az
    odom_vx = msg.twist.twist.linear.x
    odom_vy = msg.twist.twist.linear.y
    odom_az = msg.twist.twist.angular.z
    # rospy.logdebug("Odom velocity: vx=%f, vy=%f, az=%f", odom_vx, odom_vy, odom_az)

def publishOdom():
    global odom_vx, odom_vy, odom_az
    rospy.init_node('send_tfodom', anonymous=True)
    
    # 订阅odom话题获取速度信息
    rospy.Subscriber('odom', Odometry, odom_callback)
    
    # 创建UDP套接字用于广播
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
    listener = tf.TransformListener()
    rate = rospy.Rate(50.0)
    
    # 等待tf树建立
    rospy.sleep(5.0)
    # rospy.loginfo("开始发布主车位置和速度信息")
    
    while not rospy.is_shutdown():
        try:
            # 获取主车位置信息，使用odom到base_link的转换
            (trans, rot) = listener.lookupTransform("odom", "base_link", rospy.Time(0))
            
            # 计算偏航角
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
            
            # 打包数据并通过UDP广播
            data = struct.pack("ffffff", trans[0], trans[1], yaw, odom_vx, odom_vy, odom_az)
            soc.sendto(data, ('255.255.255.255', 10000))
            
            # rospy.loginfo("Published: x=%f, y=%f, yaw=%f, vx=%f, vy=%f, az=%f", trans[0], trans[1], yaw, odom_vx, odom_vy, odom_az)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # rospy.logwarn("TF错误: %s", e)
            rospy.sleep(1.0)
            continue
            
        rate.sleep()

if __name__ == '__main__':
    try:
        publishOdom()
    except rospy.ROSInterruptException:
        pass
