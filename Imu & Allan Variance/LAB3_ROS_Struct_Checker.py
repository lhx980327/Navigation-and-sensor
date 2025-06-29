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

    assert ( "imu_driver" in files )==True, "ROS Package naming is not correct"

    if ("imu_driver" in files):
        package = "imu_driver"

    
    os.chdir(os.getcwd()+"/"+package)

    assert("msg" not in os.listdir(os.getcwd()+"/msg") and "launch" not in os.listdir(os.getcwd()+"/launch") and "python" not in os.listdir(os.getcwd()+"/python"))==True, "Incorrect folder names, do you have 'python' 'msg' and 'launch' folders?"

    assert("imu_msg.msg" not in (os.getcwd()+"/msg"))==True, "No imu_msg.msg file found or is the naming convention correct ?"

    assert("driver.py" not in (os.getcwd()+"/python") or "imu_driver.py" not in (os.getcwd()+"/python")) == True, "No driver.py file found in python folder"

    assert("driver.launch" not in (os.getcwd()+"/launch"))== True, "No driver.launch file found in launch folder"

    os.chdir(home_dir)



    os.system("catkin_make")

    screens = os.popen("screen -ls").read()
    if "No Sockets found in" in screens:
        WhyYouReadingMyCode="Get OUT"
    else:
        os.system("sudo killall screen")

    os.system('screen -S ros_node -dm roslaunch "'+package+'" driver.launch port:="'+port+'"')

    print("Screen Running, your ROS node should start within 10 seconds.")

    time.sleep(10)

    rospy.init_node('Evaluator', anonymous=True)
    

    try :
        from imu_driver.msg import imu_msg
    except:
        assert False, "unable to import imu_msg.msg, have you sourced devel/setup.bash in the terminal?"


    
    rospy.Subscriber("imu", imu_msg, callback)

    cur_time = time.time()
    try :
        while "BREAD" in DATA and time.time()-cur_time<10:

            time.sleep(0.5)
            print("waiting for topic")
            print(" ")

        if "BREAD" == DATA:
            assert False, "not publishing over topic imu. Either node never initialized or topic name was incorrect"

    except:
        if "BREAD" == DATA:
            assert False, "not publishing over topic imu. Either node never initialized / topic name was incorrect / Node never published to topic imu. You can verify this by running rostopic echo imu after launching the node"

    print("Received Message on Topic")
    print(" ")
    print(" ")
    print(" ")

    print("Screen Dumping Values")

    try :

        print(DATA.Header)        
        print(DATA.MagField)
        print(DATA.IMU)      

    except:

        assert False, "Names of Header file do not match the names of message file requested in Handout"

    
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



