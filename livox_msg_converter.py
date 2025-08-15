#!/usr/bin/env python3
"""
Livox消息转换器
将 livox_laser_simulation/CustomMsg 转换为 livox_ros_driver2/CustomMsg
输入话题：/livox/lidar
输出话题：/livox/lidar0
"""

import rospy
from livox_laser_simulation.msg import CustomMsg as SimCustomMsg
from livox_laser_simulation.msg import CustomPoint as SimCustomPoint
from livox_ros_driver2.msg import CustomMsg as DriverCustomMsg
from livox_ros_driver2.msg import CustomPoint as DriverCustomPoint

class LivoxMessageConverter:
    def __init__(self):
        rospy.init_node('livox_msg_converter', anonymous=True)

        # 消息计数器
        self.msg_count = 0

        # 创建订阅者 - 订阅仿真消息
        self.sub = rospy.Subscriber('/livox/lidar', SimCustomMsg, self.convert_callback, queue_size=10)

        # 创建发布者 - 发布标准驱动消息
        self.pub = rospy.Publisher('/livox/lidar0', DriverCustomMsg, queue_size=10)

        rospy.loginfo("Livox消息转换器已启动")
        rospy.loginfo("输入话题: /livox/lidar (livox_laser_simulation/CustomMsg)")
        rospy.loginfo("输出话题: /livox/lidar0 (livox_ros_driver2/CustomMsg)")

    def convert_callback(self, sim_msg):
        """转换消息回调函数"""
        try:
            # 创建目标消息
            driver_msg = DriverCustomMsg()

            # 复制header信息
            driver_msg.header = sim_msg.header
            driver_msg.header.frame_id = "livox_frame"  # 可以根据需要修改frame_id

            # 复制基本信息
            driver_msg.timebase = sim_msg.timebase
            driver_msg.point_num = sim_msg.point_num
            driver_msg.lidar_id = sim_msg.lidar_id
            driver_msg.rsvd = sim_msg.rsvd

            # 转换点云数据
            driver_msg.points = []
            for sim_point in sim_msg.points:
                driver_point = DriverCustomPoint()
                driver_point.offset_time = sim_point.offset_time
                driver_point.x = sim_point.x
                driver_point.y = sim_point.y
                driver_point.z = sim_point.z
                driver_point.reflectivity = sim_point.reflectivity
                driver_point.tag = sim_point.tag
                driver_point.line = sim_point.line
                driver_msg.points.append(driver_point)

            # 发布转换后的消息
            self.pub.publish(driver_msg)

            # 统计信息
            self.msg_count += 1
            if self.msg_count % 100 == 0:
                rospy.loginfo(f"已转换 {self.msg_count} 条消息, 当前点数: {driver_msg.point_num}")

        except Exception as e:
            rospy.logerr(f"消息转换失败: {e}")

    def run(self):
        """运行转换器"""
        rospy.loginfo("消息转换器正在运行...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("收到停止信号，正在关闭...")
        finally:
            rospy.loginfo(f"总共转换了 {self.msg_count} 条消息")

if __name__ == '__main__':
    try:
        converter = LivoxMessageConverter()
        converter.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")
    except Exception as e:
        rospy.logerr(f"转换器启动失败: {e}")
