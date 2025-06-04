#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped

# Fill these from your map.yaml
MAP_ORIGIN_X = -100.0  # origin[0]
MAP_ORIGIN_Y = -100.0  # origin[1]
MAP_RESOLUTION = 0.05  # resolution

def point_callback(msg):
    world_x = msg.point.x
    world_y = msg.point.y

    mx = int((world_x - MAP_ORIGIN_X) / MAP_RESOLUTION)
    my = int((world_y - MAP_ORIGIN_Y) / MAP_RESOLUTION)

    rospy.loginfo(f"World: ({world_x:.2f}, {world_y:.2f}) -> Map Cell: (mx: {mx}, my: {my})")

def main():
    rospy.init_node("clicked_point_to_cell", anonymous=True)
    rospy.Subscriber("/clicked_point", PointStamped, point_callback)
    rospy.loginfo("Click anywhere in RViz using 'Publish Point' to get map cell coordinates...")
    rospy.spin()

if __name__ == '__main__':
    main()
