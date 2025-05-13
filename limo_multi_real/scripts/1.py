#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # 创建一个新的LaserScan消息
    new_msg = LaserScan()
    new_msg.header = msg.header
    new_msg.angle_min = msg.angle_min
    new_msg.angle_max = msg.angle_max
    new_msg.angle_increment = msg.angle_increment
    new_msg.time_increment = msg.time_increment
    new_msg.scan_time = msg.scan_time
    new_msg.range_min = msg.range_min
    new_msg.range_max = msg.range_max

    # 过滤掉数据为0的点
    new_msg.ranges = [r for r in msg.ranges if r > 0]
    new_msg.intensities = [i for i, r in zip(msg.intensities, msg.ranges) if r > 0]

    # 发布新的LaserScan消息
    scan_pub.publish(new_msg)

if __name__ == '__main__':
    rospy.init_node('scan_filter')
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
    scan_pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)
    rospy.spin()
