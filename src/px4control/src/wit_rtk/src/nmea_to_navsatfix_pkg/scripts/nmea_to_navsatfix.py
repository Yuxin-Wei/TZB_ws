#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

class NMEA2NavSatFixNode:
    def __init__(self):
        rospy.init_node('nmea_to_navsatfix', anonymous=True)
        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1.0)

        # 发送 GPGGA 命令
        self.send_gpgga_command()

        self.pub = rospy.Publisher('/rtk/global_position/global', NavSatFix, queue_size=10)
        self.navsatfix_msg = NavSatFix()
        self.navsatfix_msg.header = Header()
        self.navsatfix_msg.header.frame_id = 'world'

    def send_gpgga_command(self):
        """
        向串口发送 GPGGA 命令
        """
        gpgga_command = "GNGGA,0.05\r\n"  # 示例 GPGGA 命令
        self.serial_port.write(gpgga_command.encode('ascii'))
        rospy.loginfo("Sent GNGGA command: {}".format(gpgga_command.strip()))

    def parse_nmea(self, nmea_sentence):
        try:
            msg = pynmea2.parse(nmea_sentence)
            self.navsatfix_msg.header.stamp = rospy.Time.now()
            if isinstance(msg, pynmea2.types.talker.GGA):
                self.navsatfix_msg.latitude = msg.latitude
                self.navsatfix_msg.longitude = msg.longitude
                self.navsatfix_msg.altitude = float(msg.altitude) + float(msg.geo_sep) if msg.altitude is not None else 0.0  # 确保是浮点数
                self.navsatfix_msg.status.status = int(msg.gps_qual)#定位状态 4是rtk
                try:
                    hdop = float(msg.horizontal_dil) if hasattr(msg, 'horizontal_dil') else 5.0
                except ValueError:
                    hdop = 5.0
                    rospy.logwarn(f"Invalid HDOP value, using default: {hdop}")
                vdop = hdop * 1.5  # 估算VDOP
                # position_covariance = [
                #             (hdop * 0.01)**2, 0.0, 0.0,  # σ_xx（HDOP方差）
                #             0.0, (hdop * 0.01)**2, 0.0,   # σ_yy（HDOP方差）
                #             0.0, 0.0, (vdop * 0.01)**2    # σ_zz（VDOP方差）
                #         ]
                position_covariance = [
                            hdop, 0.0, 0.0,  # σ_xx（HDOP方差）
                            0.0,hdop, 0.0,   # σ_yy（HDOP方差）
                            0.0, 0.0,vdop   # σ_zz（VDOP方差）
                        ]
                self.navsatfix_msg.position_covariance = position_covariance
                self.navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            elif isinstance(msg, pynmea2.types.talker.RMC):
                pass
        except pynmea2.ParseError as e:
            rospy.logwarn("Failed to parse NMEA sentence: {}".format(e))

    def run(self):
        rospy.loginfo("Listening to NMEA data on {} at {} baudrate...".format(self.port, self.baudrate))
        while not rospy.is_shutdown():
            try:
                nmea_sentence = self.serial_port.readline().decode('ascii', errors='ignore').strip()
                if nmea_sentence:
                    # 打印收到的串口消息
                    rospy.loginfo("Received NMEA: {}".format(nmea_sentence))
                    rospy.logdebug("Received NMEA: {}".format(nmea_sentence))  # 调试用
                    self.parse_nmea(nmea_sentence)
                    if not isinstance(self.navsatfix_msg.altitude, float):
                        self.navsatfix_msg.altitude = 0.0  # 确保是浮点数
                    self.pub.publish(self.navsatfix_msg)
            except serial.SerialException as e:
                rospy.logerr("Serial port error: {}".format(e))
                break
        self.serial_port.close()

if __name__ == '__main__':
    try:
        node = NMEA2NavSatFixNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
