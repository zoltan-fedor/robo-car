#!/usr/bin/env python
import rospy
from std_msgs.msg import String
 
def talker():
    pub = rospy.Publisher('drive_control_publish', String, queue_size=10)
    rospy.init_node('drive_control_publisher', anonymous=False)
    rate = rospy.Rate(0.2) # 0.2hz (how many times per second) 
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish("U")
        pub.publish("D")
        pub.publish("D")
        pub.publish("D")
        pub.publish("U")
        pub.publish("U")
        pub.publish("D")
        pub.publish("L")
        pub.publish("L")
        pub.publish("L")
        pub.publish("R")
        rate.sleep()
 
if __name__ == '__main__':
    try:
       talker()
    except rospy.ROSInterruptException:
        pass