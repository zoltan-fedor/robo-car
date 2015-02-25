#!/usr/bin/env python
from Tkinter import *
import rospy
from std_msgs.msg import String

##############
## this file is used to manually provide control by using the 5-1-2-3 buttons on the numlock (we can't use the up-left-down-right buttons as those are considerd special keys by Tkinter)
## when these buttons are pressed, then the related U-L-D-R message is sent to the drive_control.py module
## hence you can drive the car from here
## one thing that Tkinter requires a display, so you can't do this on the car itself, but you need to do it on your laptop and remote connect to the ROS core running on the car

root = Tk()
####
# initialize the ROS publisher and node
pub = rospy.Publisher('drive_control_publish', String, queue_size=10)
rospy.init_node('drive_control_publisher_manual', anonymous=False)
    
def key(event): # called by keyboard events
    print "pressed", repr(event.char)
    if (event.char == '5'):
        print "U"
        pub.publish("U")
    if (event.char == '2'):
        print "D"
        pub.publish("D")
    if (event.char == '1'):
        print "L"
        pub.publish("L")
    if (event.char == '3'):
        print "R"
        pub.publish("R")

def callback(event): # called by mouse events
    frame.focus_set()
    print "clicked at", event.x, event.y

frame = Frame(root, width=290, height=200) # define the display frame
frame.bind("<Key>", key) # bind all keyboard presses
frame.bind("<Button-1>", callback) # bind all mouse lcick
frame.pack()

root.mainloop()