import os
import pandas as pd
import time
import rospy
import sys

"""
Author: S. M. Asjad
Created on: Oct 16, 2022

Updated by: Vishnu Rohit Annadanam
Updated on: Oct 21, 2023
"""


try :
    from gps_driver.msg import gps_msg
except:
    try:
        from gpsdriver.msg import gps_msg
    except:
        try:
            from lab1.msg import gps_msg

        except:
            try :
                from LAB1.msg import gps_msg

            except :
                
                try :
                    from gps_driver_package.msg import gps_msg

                except:

                    print("Open the evaluator script & change gps_driver.msg on line 14 to <your_package_name>.msg because correct convention was not followed")

                    sys.exit(0)


DATA = 0

def printer(title, expected, received):

    print(" ")
    print(title + " : ")
    print("Expected : " + str(expected))
    print("Received : " + str(received))
    print(" ")


def callback(data):

    global DATA

    print("callback occured")
    print(" ")

    try:
        printer("Time Stamp for Seconds",2*3600+24*60+58 , data.Header.stamp.secs)   # hh*3600 + mm*60 + ss
        printer("Time Stamp for Nano-Seconds",int(0.40*10**9) , data.Header.stamp.nsecs) # .ss * 10^9
    except:
        printer("Time Stamp for Seconds",2*3600+24*60+58 , data.header.stamp.secs)   # hh*3600 + mm*60 + ss
        printer("Time Stamp for Nano-Seconds",int(0.40*10**9) , data.header.stamp.nsecs) # .ss * 10^9

    try:
        printer("Latitude",34.186864 , data.Latitude)  # DD+mm.mm/60 , multiply by -1 if there is S
    except:
        printer("Latitude",34.186864 , data.latitude)  # DD+mm.mm/60 , multiply by -1 if there is S

    try:
        printer("Longitude", -119.5781328 , data.Longitude) #DDD+mm.mm/60 , multiply by -1 if there is W
    except:
        printer("Longitude", -119.5781328 , data.longitude) #DDD+mm.mm/60 , multiply by -1 if there is W

    try:
        try:
            try:
                printer("Easting",262407.8383650945 , data.UTM_easting)
            except:
                printer("Easting",262407.8383650945 , data.utm_Easting)
        except:
            printer("Easting",262407.8383650945 , data.Utm_easting)
    except:
        try:
            try:
                printer("Easting",262407.8383650945 , data.UTM_Easting)
            except:
                printer("Easting",262407.8383650945 , data.utm_easting)
        except:
            printer("Easting",262407.8383650945 , data.Utm_Easting)

    try:
        try:
            try:
                printer("Northing",3785880.2858478767 , data.UTM_northing)
            except:
                printer("Northing",3785880.2858478767 , data.utm_Northing)
        except:
            printer("Northing",3785880.2858478767 , data.Utm_northing)
    except:
        try:
            try:
                printer("Northing",3785880.2858478767 , data.UTM_Northing)
            except:
                printer("Northing",3785880.2858478767 , data.utm_northing)
        except:
            printer("Northing",3785880.2858478767 , data.Utm_Northing)
    

    #if DATA == "None":
    DATA = 1



rospy.init_node('Evaluator', anonymous=True)

rospy.Subscriber("gps", gps_msg, callback)


while DATA == 0:

    print("Waiting for topic")

    time.sleep(0.1)




print("Note : Your latitude & longitude can have more precision but they should be the same upto the 6th or 7th decimal places of the 'expected' value ")
print("Your Easting & Northing should be accurate upto 0.1 meters")

print("Closing Node")

sys.exit(0)
