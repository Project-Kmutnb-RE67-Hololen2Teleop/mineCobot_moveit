#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def publish_once():
    # Initialize the ROS node
    rospy.init_node('one_time_publisher', anonymous=True)

    # Create the publisher
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub = rospy.Publisher("/gripper_trigger",String,queue_size=10)

        # Create and publish the message
        msg = String()
        msg.data = "off"  #on/off
        pub.publish(msg)

        rospy.loginfo("Published: %s", msg.data)
        rate.sleep()
if __name__ == '__main__':
    try:
        publish_once()
    except rospy.ROSInterruptException:
        pass
