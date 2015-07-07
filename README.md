# robo-car
Building our robotic car 
 
INSTALL:  
a. In the catkin source folder checkout our repo  
$ cd ~/catkin_ws/src  
$ git clone https://github.com/zoltan-fedor/robo_car  
  
b. build the catkin package using catkin_make:  
$ cd ~/catkin_ws  
$ catkin_make  
  
c. install python tornado (as websocket server):  
$ sudo pip install tornado  
  
  
To install Python Tkinter (required for drive_control_publisher_manual.py module) on the remote ROS station (laptop) run:  
$ sudo apt-get install python-tk  


To start the robo-car nodes use:  
$ roslaunch robo_car robo_car.launch  
  
  