#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def battery_publisher():
    rospy.init_node('battery_publisher', anonymous=True)
    pub = rospy.Publisher('/battery', Float32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    battery_level = 50.0  # Hardcoded for demo, matches planner
    while not rospy.is_shutdown():
        pub.publish(battery_level)
        rospy.loginfo("Published battery level: %f", battery_level)
        rate.sleep()

if __name__ == '__main__':
    try:
        battery_publisher()
    except rospy.ROSInterruptException:
        pass
