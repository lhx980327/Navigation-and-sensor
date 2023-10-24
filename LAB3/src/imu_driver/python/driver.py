#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import sys
from scipy.spatial.transform import Rotation
import math
from imu_driver.msg import imu_msg
from sensor_msgs.msg import Imu, MagneticField


def initialize_serial(port, baud_rate=115200, timeout_duration=3.):
    try:
        return serial.Serial(port, baud_rate, timeout=timeout_duration)
    except Exception as e:
        rospy.logerr(f"Error initializing serial: {e}")
        sys.exit(1)
def parse_imu_data(line):
    if not line.startswith(b'$VNYMR'):
        return None

    s = line.split(b",")

    # Ensure that there are enough fields to unpack
    if len(s) < 13:  # (or 10 if you haven't added the extra 3 fields)
        return None
    
    try:
        yaw, pitch, roll = [float(s[i].decode('utf-8').split('*')[0]) for i in range(1, 4)]
        magx, magy, magz = [float(s[i].decode('utf-8').split('*')[0]) for i in range(4, 7)]
        aclx, acly, aclz = [float(s[i].decode('utf-8').split('*')[0]) for i in range(7, 10)]  # (if you have added aclz)
        gyrx, gyry, gyrz = [float(s[i].decode('utf-8').split('*')[0]) for i in range(10, 13)]  # (if you have added gyrx and gyry)

        return yaw, pitch, roll, magx, magy, magz, aclx, acly, aclz, gyrx, gyry, gyrz
    except ValueError as e:
        rospy.logerr(f"Error parsing IMU data: {e}")
        return None


def euler_to_quaternion(yaw, pitch, roll):
    """
    Convert Euler Angles to Quaternion (YXZ sequence)
    :param yaw: Yaw angle in radians.
    :param pitch: Pitch angle in radians.
    :param roll: Roll angle in radians.
    :return: Quaternion as [qw, qx, qy, qz]
    """

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * sp * cr + sy * cp * sr
    qy = sy * cp * cr - cy * sp * sr
    qz = cy * cp * sr - sy * sp * cr

    return [qw, qx, qy, qz]

def quaternion_to_euler(q0, q1, q2, q3):
    roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1**2 + q2**2))
    pitch = math.asin(2 * (q0 * q2 - q3 * q1))
    yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2**2 + q3**2))
    
    return roll, pitch, yaw

def imu_driver():
    rospy.loginfo("Starting imu_driver function...")
    rate = rospy.Rate(40)  # 40Hz
    port = rospy.get_param('~port', '/dev/ttyUSB0')
    baud_rate = rospy.get_param('~baudrate', 115200)
    
    imu_serial = initialize_serial(port, baud_rate)

    # Only one publisher for the 'imu' topic that uses the custom message type
    imu_pub = rospy.Publisher('imu', imu_msg, queue_size=10)

    seq = 1
    while not rospy.is_shutdown():
        raw_data = imu_serial.readline()
        parsed_data = parse_imu_data(raw_data)

        if parsed_data:
            yaw, pitch, roll, magx, magy, magz, aclx, acly, aclz, gyrx, gyry, gyrz = parsed_data

            quat = euler_to_quaternion(math.radians(yaw), math.radians(pitch), math.radians(roll))

            # Populate the IMU message
            imu_data = Imu()
            imu_data.header.seq = seq
            imu_data.header.stamp = rospy.get_rostime()
            imu_data.header.frame_id = "base_link"
            imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z = quat
            imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z = gyrx, gyry, gyrz
            imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z = aclx, acly, aclz

            # Populate the MagneticField message
            mag_msg = MagneticField()
            mag_msg.header.seq = seq
            mag_msg.header.stamp = rospy.get_rostime()
            mag_msg.header.frame_id = "base_link"
            mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z = magx, magy, magz

            # Now, we'll combine both the IMU and MagneticField messages into the custom imu_msg
           # Now, we'll combine both the IMU and MagneticField messages into the custom imu_msg
            combined_msg = imu_msg()
            combined_msg.Header = imu_data.header  # Use 'header' from the standard IMU message
            combined_msg.IMU = imu_data
            combined_msg.MagField = mag_msg
            combined_msg.raw_data = raw_data.decode('utf-8')


            # Publish the combined message
            imu_pub.publish(combined_msg)

            seq += 1
            rate.sleep()  # Sleep to maintain the loop rate at 40Hz
if __name__ == "__main__":
    try:
        rospy.init_node('imu_node')
        imu_driver()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted")
