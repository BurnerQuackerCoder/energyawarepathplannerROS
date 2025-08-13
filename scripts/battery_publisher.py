#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def battery_publisher():
    rospy.init_node('battery_publisher', anonymous=True)
    pub = rospy.Publisher('/battery', Float32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    battery_level = 25.0  # Hardcoded for demo, matches planner
    log_once = True

    while not rospy.is_shutdown():
        pub.publish(battery_level)
        if log_once:
            rospy.loginfo("Published battery level: %f", battery_level)
            log_once = False
        rate.sleep()

if __name__ == '__main__':
    try:
        battery_publisher()
    except rospy.ROSInterruptException:
        pass
