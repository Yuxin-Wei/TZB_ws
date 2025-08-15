#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix

# 全局变量存储经纬度数据
latitudes = []
longitudes = []

# 回调函数，处理GPS数据
def gps_callback(msg):
    global latitudes, longitudes
    # 提取经纬度
    latitude = msg.latitude
    longitude = msg.longitude
    # 添加到列表中
    latitudes.append(latitude)
    longitudes.append(longitude)
    # 打印当前数据
    rospy.loginfo(f"Received GPS Data: Latitude = {latitude}, Longitude = {longitude}")

# 可视化GPS轨迹
def plot_gps_trajectory():
    plt.figure(figsize=(10, 6))
    plt.plot(longitudes, latitudes, 'b-', label='GPS Trajectory')
    if len(longitudes) > 0:
        plt.scatter(longitudes[0], latitudes[0], color='green', label='Start Point')
        plt.scatter(longitudes[-1], latitudes[-1], color='red', label='End Point')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('GPS Trajectory Visualization')
    plt.legend()
    plt.grid(True)
    plt.show()

# 主函数
def main():
    # 初始化ROS节点
    rospy.init_node('gps_visualization_node', anonymous=True)
    # 订阅GPS话题
    rospy.Subscriber('/rtk/global_position/global', NavSatFix, gps_callback)
    rospy.loginfo("GPS Visualization Node Started. Waiting for GPS data...")
    # 保持节点运行，直到手动停止
    rospy.spin()
    # 节点停止后，绘制轨迹
    plot_gps_trajectory()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass