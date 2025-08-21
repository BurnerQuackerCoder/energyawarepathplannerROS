#!/usr/bin/env python
import rospy
import rospkg
import sys
from PIL import Image, ImageDraw

def edit_map_for_demo():
    if len(sys.argv) < 2:
        print("Usage: demo_map_editor.py [true|false]")
        return

    use_barrier_str = sys.argv[1].lower()
    use_barrier = (use_barrier_str == 'true')

    print("Map Editor Script: use_barrier is set to %s" % use_barrier)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('enawpl')
    map_dir = pkg_path + '/maps/'
    
    original_map_path = map_dir + 'maze_og.pgm'
    active_map_path = map_dir + 'maze.pgm'

    # --- Map properties from maze.yaml ---
    map_resolution = 0.05
    map_origin_x = -1.0
    map_origin_y = -10.1

    # --- CORRECTED Barrier coordinates in METERS ---
    # These values will produce the exact same zone as your original
    # min_mx: 70, max_mx: 100, min_my: 70, max_my: 100
    barrier_min_x = 2.5
    barrier_max_x = 4.0
    barrier_min_y = -6.6
    barrier_max_y = -5.1

    try:
        img = Image.open(original_map_path)
        
        if use_barrier:
            print("Map Editor: Barrier is ON. Drawing black rectangle on map.")
            draw = ImageDraw.Draw(img)

            # Convert world coordinates (meters) to image coordinates (pixels)
            img_height = img.height
            
            min_px = int((barrier_min_x - map_origin_x) / map_resolution)
            max_px = int((barrier_max_x - map_origin_x) / map_resolution)
            
            min_py_ros = int((barrier_min_y - map_origin_y) / map_resolution)
            max_py_ros = int((barrier_max_y - map_origin_y) / map_resolution)

            # Flip y-axis for the image's top-left origin
            min_py_img = img_height - max_py_ros
            max_py_img = img_height - min_py_ros

            # Draw the black obstacle rectangle
            draw.rectangle([min_px, min_py_img, max_px, max_py_img], fill=0)

        img.save(active_map_path)
        print("Map Editor: Active map '%s' is ready." % active_map_path)

    except Exception as e:
        print("Error in map editor script: %s" % e)

if __name__ == '__main__':
    edit_map_for_demo()