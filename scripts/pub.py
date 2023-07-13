#!/usr/bin/env python3

import rospy 
from msg_pub_sub.msg import xyz

#Publisher
def pub(msg):
    pub = rospy.Publisher("topic01", xyz, queue_size=10)
    rospy.init_node("pub01", anonymous=True)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()

    
if __name__ == "__main__":
    try:
        pub(xyz)
    except rospy.ROSInterruptException:
        pass 

    