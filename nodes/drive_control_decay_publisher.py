#!/usr/bin/env python
import rospy
from std_msgs.msg import String
 
def talker():
    pub = rospy.Publisher('drive_control_decay_publish', String, queue_size=10)
    rospy.init_node('drive_control_decay_publisher', anonymous=False)
    rate = rospy.Rate(5) # 5hz (5 times per second)
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        pub.publish("decay")
        rate.sleep()
 
if __name__ == '__main__':
    try:
       talker()
    except rospy.ROSInterruptException:
        pass