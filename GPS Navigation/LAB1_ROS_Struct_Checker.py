import os
import pandas as pd
import time
import rospy
import sys

DATA = "BREAD"
argv=sys.argv
port = str(argv[1])

def callback(data):

    global DATA

    print("callback occured")
    #if DATA == "None":
    DATA = data

if __name__ == "__main__":

    home_dir = os.getcwd()
    home_files = os.listdir(os.getcwd())
    #global DATA

    print(home_files)

    assert "src" in  home_files, "No src folder found"

    os.chdir(os.getcwd()+"/src")

    files = os.listdir(os.getcwd())

    assert ( "gps_driver" not in files or "gps-driver" not in files )==True, "ROS Package naming is not correct"

    if ("gps_driver" in files):
        package = "gps_driver"
    else:
        package = "gps-driver"

    
    os.chdir(os.getcwd()+"/"+package)

    assert("msg" not in os.listdir(os.getcwd()+"/msg") and "launch" not in os.listdir(os.getcwd()+"/launch") and "python" not in os.listdir(os.getcwd()+"/python"))==True, "Incorrect folder names, do you have 'python' 'msg' and 'launch' folders?"

    assert("gps_msg.msg" not in (os.getcwd()+"/msg"))==True, "No gps_msg.msg file found or is the naming convention correct ?"

    assert("driver.py" not in (os.getcwd()+"/python")), "No driver.py file found in python folder"

    assert("driver.launch" not in (os.getcwd()+"/launch"))== True, "No driver.launch file found in launch folder"

    os.chdir(home_dir)



    os.system("catkin_make")

    os.system('screen -S ros_node -dm roslaunch "'+package+'" driver.launch port:="'+port+'"')

    print("Screen Running, your ROS node should start within 10 seconds.")

    time.sleep(10)

    rospy.init_node('Evaluator', anonymous=True)
    

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
                    assert False, "unable to import gps_msg.msg, have you sourced devel/setup.bash in the terminal?"


    
    rospy.Subscriber("gps", gps_msg, callback)

    cur_time = time.time()
    try :
        while "BREAD" in DATA and time.time()-cur_time<30:

            time.sleep(0.5)
            print("waiting for topic")
            print(" ")

        if "BREAD" in DATA:
            assert False, "not publishing over topic gps. Either node never initialized or topic name was incorrect"

    except:
        pass

    print(" ")
    print(" ")
    print(" ")
    os.system("clear")
    print("Screen Dumping Values received messages")

    try :

        try:
            if DATA.Header.frame_id.upper() != "GPS1_FRAME":
                assert False, "Incorrect Frame ID"

                print("Seconds : ")
                print(DATA.Header.stamp.secs)

        except:
            if DATA.header.frame_id.upper() != "GPS1_FRAME":
                assert False, "Incorrect Frame ID"

                print("Seconds : ")
                print(DATA.header.stamp.secs)
        
        print("\nLatitude : ")
        try:
            print(abs(DATA.Latitude))
        except:
            print(abs(DATA.latitude))
        print("\nLongitude : ")
        try:
            print(abs(DATA.Longitude))
        except:
            print(abs(DATA.longitude))
        print("\nEasting : ")
        try:
            try:
                try:
                    print(abs(DATA.UTM_easting))
                except:
                    print(abs(DATA.UTM_Easting))
            except:
                print(abs(DATA.utm_easting))
        except:
            try:
                try:
                    print(abs(DATA.Utm_easting))
                except:
                    print(abs(DATA.Utm_Easting))
            except:
                print(abs(DATA.utm_Easting))
        print("\nNorthing : ")
        try:
            try:
                try:
                    print(abs(DATA.UTM_northing))
                except:
                    print(abs(DATA.UTM_Northing))
            except:
                print(abs(DATA.utm_northing))
        except:
            try:
                try:
                    print(abs(DATA.Utm_northing))
                except:
                    print(abs(DATA.Utm_Northing))
            except:
                print(abs(DATA.utm_Northing))
        print("\nZone : ")
        try:
            print(abs(DATA.Zone))
        except:
            print(abs(DATA.zone))
        print("\nLetter : ")
        try:
            print(DATA.Letter) 
        except:
            print(DATA.letter)

        ###############################################################################
        # Uncomment these lines if Hanu plans to use HDOP and UTC
        # print("\nHDOP : ")
        # print(DATA.HDOP) 
        # print("\nUTC : ")
        # print(DATA.UTC) 
               

    except:

        os.system("screen -S ros_node -X quit")

        assert False, "Structure Failed. Review Handout for Correct Structure"

    
    os.system("screen -S ros_node -X quit")

    print(" ")
    print(" ")
    print(" ")
    print("End of screen dump")
    print(" ")
    print(" ")
    print(" ")
    
    print("The repository structure is correct")
    print(" ")
    print(" ")
    print(" ")



