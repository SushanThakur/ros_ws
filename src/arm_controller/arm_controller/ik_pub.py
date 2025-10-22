import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
import sys

import xml.etree.ElementTree as ET
from svg.path import parse_path
import matplotlib.pyplot as plt

AUTO_RUN = True
SVG_FILE = "svg/SUSHANT.svg"

SCALE_FACTOR = 0.001
PEN_DOWN_Z = 0.07
PEN_UP_Z = 0.15

node_name = 'ik_cmd_pub'
# default_orient = [0.0, 0.0, 0.707, -0.707] # Quaternion Representation
default_orient = [0.7068251814624406, 0.7073873723632136, 0.0011261755880557509, 0.0] # Quaternion Representation

test_entity = [
    [0.4,0.3,0.1],
    [0.3,0.3,0.1],
    [0.3,0.2,0.1],
    [0.4,0.2,0.1],
    [0.4,0.1,0.1],
    [0.3,0.1,0.1],
]

# draw_entity = [
# [-0.057 ,0.342 ,0.200],
# [-0.057 ,0.334 ,0.100],
# ]

def extract_coordinates_from_svg(svg_file):
    # Parse SVG file
    try:
        tree = ET.parse(svg_file)
        root = tree.getroot()
    except FileNotFoundError:
        print(f"Error: The file '{svg_file}' was not found.")
        return []
    except Exception as e:
        print(f"An error occurred while parsing SVG: {e}")
        return []

    # Namespace for SVG
    ns = {'svg': 'http://www.w3.org/2000/svg'}
    
    # List to store all coordinates
    coordinates = []

    # Extract <path> elements
    for path_elem in root.findall('.//svg:path', ns):
        path_data = path_elem.get('d')

        if not path_data:
            continue
            
        try:
            # Parse the path data
            path = parse_path(path_data)
            
            # Extract coordinates from each segment in the path
            for segment in path:
                segment_coords = []
                # Handle different types of path segments
                if hasattr(segment, 'start'):
                    segment_coords.append((segment.start.real, segment.start.imag))  # Start point
                if hasattr(segment, 'end'):
                    segment_coords.append((segment.end.real, segment.end.imag))  # End point
                if hasattr(segment, 'control1'):  # For curves (e.g., CubicBezier)
                    segment_coords.append((segment.control1.real, segment.control1.imag))  # Control point 1
                if hasattr(segment, 'control2'):  # For curves (e.g., QuadraticBezier)
                    segment_coords.append((segment.control2.real, segment.control2.imag))  # Control point 2
                
                coordinates.append({
                    'type': segment.__class__.__name__,
                    'points': segment_coords
                })
                
        except Exception as e:
            print(f"Error parsing path data: {e}")
            continue
    
    # Extract coordinates from <circle> elements
    for circle in root.findall('.//svg:circle', ns):
        cx = float(circle.get('cx', 0))
        cy = float(circle.get('cy', 0))
        r = float(circle.get('r', 0))
        coordinates.append({
            'type': 'Circle',
            'center': (cx, cy),
            'radius': r
        })
    
    # Extract coordinates from <rect> elements
    for rect in root.findall('.//svg:rect', ns):
        x = float(rect.get('x', 0))
        y = float(rect.get('y', 0))
        width = float(rect.get('width', 0))
        height = float(rect.get('height', 0))
        coordinates.append({
            'type': 'Rectangle',
            'top_left': (x, y),
            'bottom_right': (x + width, y + height)
        })
    
    # Extract coordinates from <polygon> elements
    for polygon in root.findall('.//svg:polygon', ns):
        points = polygon.get('points', '').strip()
        if points:
            point_list = []
            for point in points.split():
                x, y = map(float, point.split(','))
                point_list.append((x, y))
            coordinates.append({
                'type': 'Polygon',
                'points': point_list
            })

    return coordinates

class IkPublisher(Node):
    def __init__(self):
        super().__init__(node_name)

        self.ik_pub_ = self.create_publisher(Pose, node_name, 10)
        self.joy_sub_ = self.create_subscription(Joy, 'joy', self.joy_call, 10)

        if AUTO_RUN:
            self.timer_call = self.create_timer(1, self.ik_pub_call)

        self.point_index = 0
        self.last_button_state = 0
        
    def joy_call(self, msg):
            button = msg.buttons[2]
            axis = msg.axes[7]
            if axis == 1.0 and not button == self.last_button_state:
                self.ik_pub_call()
            self.last_button_state = axis

    def ik_pub_call(self):
        self.get_logger().info("Drawing started")
        coordinates = extract_coordinates_from_svg(SVG_FILE)
        draw_entity = []
        last_point = [0.0, 0.0]
        for item in coordinates:
            for a,b in item['points']:
                tx = 1 * (a*SCALE_FACTOR - 0.4)
                ty = -1 * (b*SCALE_FACTOR - 0.43)
                if item['type']=='Move':
                    tz = PEN_UP_Z
                elif item['type']=='Line':
                    tz = PEN_DOWN_Z
                if not last_point == [a,b]:
                    # print(f'[{tx:.3f} ,{ty:.3f} ,{tz:.3f}],')
                    draw_entity.append([tx, ty, tz])
                    plt.plot(tx, ty, 'o')
                last_point = [a,b]
        plt.show()
        while self.point_index < len(draw_entity):
            pub_pose = Pose()
            pub_pose.orientation.w = default_orient[0]
            pub_pose.orientation.x = default_orient[1]
            pub_pose.orientation.y = default_orient[2]
            pub_pose.orientation.z = default_orient[3]

            pub_pose.position.x = draw_entity[self.point_index][0]
            pub_pose.position.y = draw_entity[self.point_index][1]
            pub_pose.position.z = draw_entity[self.point_index][2]

            self.ik_pub_.publish(pub_pose)
            self.get_logger().debug(f"Published pose {self.point_index + 1}: {pub_pose}")
            time.sleep(0.5)
            self.point_index += 1

        self.get_logger().info("Drawing stopped")
        self.point_index = 0

        if AUTO_RUN:
            self.destroy_node()
            sys.exit()

def main():
    try:
        rclpy.init()
        node = IkPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()