#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import utm
import serial
import sys
from std_msgs.msg import Header
from GNSS_ros_driver.msg import gps_msg

rospy.init_node('gps_node')

def initialize_serial(port, baud_rate=4800 ,timeout_duration=3):
    try:
        return serial.Serial(port, baud_rate, timeout=timeout_duration)
    except Exception as e:
        print(f"Error initializing serial: {e}")
        sys.exit(1)
def parse_gps_data(line):
    elements = line.split(",")
    if  "$GNGGA" not in elements[0]:
        return None
    if not elements[2] or not elements[4]:
        rospy.logwarn("Incomplete GPS data received: "+line)
        return None

    #Parsing UTC time
    utc_time = float(elements[1])
    hours,remainder= divmod(utc_time,10000)
    minutes,seconds = divmod(remainder,100)
    total_seconds = hours* 3600+ minutes*60+seconds
    nsecs = int((total_seconds * (10**9)) % (10**9))

    #Parsing Latitude
    raw_latitude = float(elements[2])
    lat_degree, lat_minute =divmod(raw_latitude,100)
    latitude = lat_degree +lat_minute/60
    if elements[3] =='S':
        latitude*=-1

    #Parsing longtitude
    raw_longtitude = float(elements[4])
    long_degree,long_minute = divmod(raw_longtitude,100)
    longtitude = long_degree + long_minute/60
    if elements[5] =="W":
        longtitude*=-1

    #Parsing Altitude
    altitude = float(elements[9])

    rtk = int(elements[6]) if elements[6] else 0  # Assuming rtk is an integer. Adjust if it's a different type.

    return total_seconds,nsecs,latitude,longtitude,altitude,rtk

def gps_driver():
   print("Starting gps_driver function...")
   port = rospy.get_param('~port', '/dev/pts/5')
   baud_rate = rospy.get_param('~baudrate',4800)

   gps_serial =initialize_serial(port,baud_rate)
   pub = rospy.Publisher('/gps', gps_msg, queue_size=10)

   while not rospy.is_shutdown():
       raw_data = gps_serial.readline()
       
       decoded_data = raw_data.decode('utf-8', 'ignore')
       gps_data = decoded_data.strip()
       rospy.loginfo("Received data: %s", gps_data)
       parsed_data =parse_gps_data(gps_data)
       
       msg = gps_msg()
       if parsed_data:
           utc_secs,utc_nsecs,lat,lon,alt,rtk = parsed_data
           
           utm_data = utm.from_latlon(lat,lon)

           #update message fields
           msg.header.stamp.secs = int(utc_secs)
           msg.header.stamp.nsecs = int(utc_nsecs)
           msg.header.frame_id = 'GPS1_Frame'
           msg.Latitude = lat
           msg.Longtitude = lon
           msg.Altitude = alt
           msg.UTM_easting, msg.UTM_northing, msg.Zone, msg.Letter = utm_data
           msg.rtk = rtk
           print("Received data:", gps_data)
           rospy.loginfo("Publishing GPS data: %s", msg)
           pub.publish(msg)

           


if __name__ == "__main__":
    try:
        gps_driver()
    except rospy.ROSInterruptException:

        rospy.logerr("Interrupted")