#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
以固定频率(默认10Hz)记录无人机飞行位置到CSV文件。
订阅 nav_msgs/Odometry 话题，定时采样位置并写入文件。

用法(launch文件中):
    <node name="drone_position_logger" pkg="test_interface" type="record_drone_position.py"
          output="screen">
        <param name="odom_topic" value="/quad_0/lidar_slam/odom"/>
        <param name="output_dir" value="$(find test_interface)/logs"/>
        <param name="log_rate"    value="10.0"/>
    </node>
"""

import os
import csv
import rospy
import datetime
from nav_msgs.msg import Odometry


class PositionLogger:
    def __init__(self):
        # 从参数服务器读取参数
        self.odom_topic = rospy.get_param("~odom_topic", "/quad_0/lidar_slam/odom")
        self.output_dir = rospy.get_param(
            "~output_dir",
            os.path.expanduser("~/drone_position_logs")
        )
        self.log_rate = float(rospy.get_param("~log_rate", 10.0))  # Hz

        # 最新里程计数据
        self.latest_odom = None
        self.has_data = False

        # 确保输出目录存在
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        # 根据当前时间生成文件名(精确到毫秒, 避免同秒多次启动互相覆盖):
        # drone_position_YYYYMMDD_HHMMSS_mmm.csv
        now = datetime.datetime.now()
        filename = "drone_position_{}_{:03d}.csv".format(
            now.strftime("%Y%m%d_%H%M%S"), now.microsecond // 1000
        )
        self.output_file = os.path.join(self.output_dir, filename)

        # 打开CSV文件并写表头
        self.csv_file = open(self.output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'x', 'y', 'z',
            'vx', 'vy', 'vz',
            'roll', 'pitch', 'yaw'
        ])
        self.csv_file.flush()

        # 订阅odom话题
        self.sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=10
        )

        rospy.loginfo("[PositionLogger] 订阅话题: %s", self.odom_topic)
        rospy.loginfo("[PositionLogger] 输出文件: %s", self.output_file)
        rospy.loginfo("[PositionLogger] 记录频率: %.1f Hz", self.log_rate)

    def odom_callback(self, msg):
        """里程计回调，仅缓存最新数据"""
        self.latest_odom = msg
        self.has_data = True

    def log_loop(self):
        """以固定频率记录位置"""
        rate = rospy.Rate(self.log_rate)
        while not rospy.is_shutdown():
            if self.has_data and self.latest_odom is not None:
                odom = self.latest_odom
                t = odom.header.stamp.to_sec()
                if t == 0.0:
                    t = rospy.get_time()

                px = odom.pose.pose.position.x
                py = odom.pose.pose.position.y
                pz = odom.pose.pose.position.z

                vx = odom.twist.twist.linear.x
                vy = odom.twist.twist.linear.y
                vz = odom.twist.twist.linear.z

                # 从四元数提取欧拉角
                q = odom.pose.pose.orientation
                roll, pitch, yaw = self.quaternion_to_euler(
                    q.x, q.y, q.z, q.w
                )

                self.csv_writer.writerow([
                    "{:.6f}".format(t),
                    "{:.6f}".format(px),
                    "{:.6f}".format(py),
                    "{:.6f}".format(pz),
                    "{:.6f}".format(vx),
                    "{:.6f}".format(vy),
                    "{:.6f}".format(vz),
                    "{:.6f}".format(roll),
                    "{:.6f}".format(pitch),
                    "{:.6f}".format(yaw)
                ])
                self.csv_file.flush()

            rate.sleep()

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """四元数转欧拉角(roll, pitch, yaw)"""
        import math
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def shutdown(self):
        """关闭文件，并在终端打印最终保存位置与完整文件名"""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        print(
            "[PositionLogger] CSV 文件已保存到: {}".format(self.output_file),
            flush=True,
        )


if __name__ == '__main__':
    try:
        rospy.init_node('drone_position_logger', anonymous=True)
        logger = PositionLogger()
        rospy.on_shutdown(logger.shutdown)
        logger.log_loop()
    except rospy.ROSInterruptException:
        pass
