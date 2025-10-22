import xml.etree.ElementTree as ET
from svg.path import parse_path
import matplotlib.pyplot as plt
import numpy as np

SVG_PATH = 'svg/TEST.svg'

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

svg_file = SVG_PATH
coordinates = extract_coordinates_from_svg(svg_file)

# coordinates [
    # [0] = {type: '', point: [,]}
    # [1] = {type: '', point: [,]}
    # [2] = {type: '', point: [,]}
# ]

new_coordinates = []

last_point = [0.0, 0.0]
for item in coordinates:
    for a,b in item['points']:
        tx = 1 * (a/100 - 0.1)
        ty = -1 * (b/100 - 0.35)
        if item['type']=='Move':
            tz = 0.2
        elif item['type']=='Line':
            tz = 0.1
        if not last_point == [a,b]:
            print(f'[{tx:.3f} ,{ty:.3f} ,{tz:.3f}],')
            plt.plot(tx, ty, 'o')
        last_point = [a,b]

plt.show()


