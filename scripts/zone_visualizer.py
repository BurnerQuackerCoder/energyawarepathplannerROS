#!/usr/bin/env python3

import rospy
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import MapMetaData
from std_msgs.msg import String, Float32

class ZoneVisualizer:
    def __init__(self):
        rospy.init_node('zone_visualizer', anonymous=True)

        # Get the path to the YAML file from a ROS parameter
        self.yaml_path = rospy.get_param('~yaml_path')
        if not self.yaml_path:
            rospy.logerr("Required parameter 'yaml_path' is not set. Please provide the path to your energy_aware_planner_params.yaml.")
            return

        # Publisher for the markers
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1, latch=True)

        # Wait for the map metadata to be published once to get resolution and origin
        rospy.loginfo("ZoneVisualizer: Waiting for map metadata...")
        try:
            self.map_metadata = rospy.wait_for_message('/map_metadata', MapMetaData, timeout=10.0)
            rospy.loginfo("ZoneVisualizer: Map metadata received.")
            self.visualize_zones()
        except rospy.ROSException as e:
            rospy.logerr(f"ZoneVisualizer: Timed out waiting for /map_metadata. Is map_server running? Error: {e}")

    def load_zones_from_yaml(self):
        try:
            with open(self.yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                # The zones are nested under the planner's namespace
                return data.get('NavfnROS', {}).get('zone_terrain_definitions', [])
        except Exception as e:
            rospy.logerr(f"ZoneVisualizer: Failed to load or parse YAML file {self.yaml_path}: {e}")
            return []

    def cell_to_world(self, mx, my):
        # Convert map cell coordinates to world coordinates for marker placement
        resolution = self.map_metadata.resolution
        origin_x = self.map_metadata.origin.position.x
        origin_y = self.map_metadata.origin.position.y

        # Center of the cell in world coordinates
        wx = origin_x + (mx + 0.5) * resolution
        wy = origin_y + (my + 0.5) * resolution
        return wx, wy

    def visualize_zones(self):
        zones = self.load_zones_from_yaml()
        if not zones:
            rospy.logwarn("ZoneVisualizer: No zone definitions found in YAML file.")
            return

        marker_array = MarkerArray()
        zone_id = 0

        for zone in zones:
            min_mx, max_mx = zone.get('min_mx'), zone.get('max_mx')
            min_my, max_my = zone.get('min_my'), zone.get('max_my')
            zone_name = zone.get('name', 'unnamed_zone')

            # --- ADDED: Robustness Check ---
            # Check if any coordinate is missing before doing math
            if any(v is None for v in [min_mx, max_mx, min_my, max_my]):
                rospy.logwarn(f"ZoneVisualizer: Skipping malformed zone '{zone_name}'. It is missing one or more coordinate keys (min_mx, max_mx, min_my, max_my).")
                continue # Skip to the next zone in the list
            # --- END ADDED ---

            # Calculate center and dimensions in world coordinates
            center_mx = (min_mx + max_mx) / 2.0
            center_my = (min_my + max_my) / 2.0
            center_wx, center_wy = self.cell_to_world(center_mx, center_my)

            width_m = (max_mx - min_mx + 1) * self.map_metadata.resolution
            height_m = (max_my - min_my + 1) * self.map_metadata.resolution

            # --- Zone Box Marker ---
            box_marker = Marker()
            box_marker.header.frame_id = "map"
            box_marker.header.stamp = rospy.Time.now()
            box_marker.ns = "energy_zone_boxes"
            box_marker.id = zone_id
            box_marker.type = Marker.CUBE
            box_marker.action = Marker.ADD
            box_marker.pose.position.x = center_wx
            box_marker.pose.position.y = center_wy
            box_marker.pose.position.z = 0.01
            box_marker.pose.orientation.w = 1.0
            box_marker.scale.x = width_m
            box_marker.scale.y = height_m
            box_marker.scale.z = 0.01
            box_marker.color.r, box_marker.color.g, box_marker.color.b, box_marker.color.a = (1.0, 0.0, 0.0, 0.4)
            box_marker.lifetime = rospy.Duration()
            marker_array.markers.append(box_marker)
            zone_id += 1

            # --- Zone Text Label Marker ---
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "energy_zone_labels"
            text_marker.id = zone_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = center_wx
            text_marker.pose.position.y = center_wy
            text_marker.pose.position.z = 0.5
            text_marker.scale.z = 0.5
            text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = (1.0, 1.0, 1.0, 1.0)
            text_marker.text = zone_name
            text_marker.lifetime = rospy.Duration()
            marker_array.markers.append(text_marker)
            zone_id += 1

        rospy.loginfo(f"ZoneVisualizer: Publishing {len(marker_array.markers) // 2} zones as markers.")
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        ZoneVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass