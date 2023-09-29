#make a catkin workspace.
#copy the /src file to the workspace
$ source ./deve1/setup.bash
$ catkin_make

Running the Publisher:
$ rosrun beginner_tutorials talker.py 

Running the Subscriber:
$ rosrun beginner_tutorials listener.py

Running the Service:
$ rosrun beginner_tutorials add_two_ints_server.py

Running the Client:
$  rosrun beginner_tutorials add_two_ints_client.py x , y #x,y can be any int value.
