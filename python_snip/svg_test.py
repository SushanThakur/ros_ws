import xml.etree.ElementTree as ET
from svg.path import parse_path
import matplotlib.pyplot as plt
import numpy as np

SVG_PATH = 'workspace.svg'

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

def plot_coordinates(coordinates):
    # Create a new figure
    fig, ax = plt.subplots()

    # Plot each shape
    for item in coordinates:
        if item['type'] in ['Line', 'CubicBezier', 'QuadraticBezier']:
            # Extract x and y coordinates
            x, y = zip(*item['points'])
            ax.plot(x, y, 'b-', label='Path' if item['type'] == 'Line' else 'Curve')
            # Plot control points for curves
            if item['type'] in ['CubicBezier', 'QuadraticBezier']:
                for point in item['points'][1:-1]:  # Control points
                    ax.plot(point[0], point[1], 'ro', label='Control Point' if point == item['points'][1] else "")

        elif item['type'] == 'Circle':
            # Plot circle
            circle = plt.Circle(item['center'], item['radius'], fill=False, color='g', label='Circle')
            ax.add_patch(circle)

        elif item['type'] == 'Rectangle':
            # Plot rectangle
            x, y = item['top_left']
            width = item['bottom_right'][0] - x
            height = item['bottom_right'][1] - y
            rect = plt.Rectangle((x, y), width, height, fill=False, color='m', label='Rectangle')
            ax.add_patch(rect)

        elif item['type'] == 'Polygon':
            # Plot polygon
            x, y = zip(*item['points'])
            ax.fill(x, y, 'c', alpha=0.3, label='Polygon')  # Filled polygon for visibility

    # Set plot properties
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('SVG Geometrical Coordinates')
    ax.grid(True)
    ax.plot(0, 0, 'o')
    ax.set_aspect('equal')  # Equal aspect ratio to preserve SVG proportions

    # Invert y-axis to match SVG coordinate system (origin at top-left)
    ax.invert_yaxis()

    # Add legend (avoid duplicate labels)
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys())

    # Auto-adjust axis limits
    plt.autoscale()
    plt.tight_layout()
    plt.show()

def print_coordinates(coordinates):
    # Print extracted coordinates in a readable format
    for item in coordinates:
        print(f"\nShape: {item['type']}")
        if item['type'] in ['Line', 'CubicBezier', 'QuadraticBezier', 'Arc']:
            print("Points:")
            for point in item['points']:
                print(f"  ({point[0]:.2f}, {point[1]:.2f})")
        elif item['type'] == 'Circle':
            print(f"Center: ({item['center'][0]:.2f}, {item['center'][1]:.2f})")
            print(f"Radius: {item['radius']:.2f}")
        elif item['type'] == 'Rectangle':
            print(f"Top-Left: ({item['top_left'][0]:.2f}, {item['top_left'][1]:.2f})")
            print(f"Bottom-Right: ({item['bottom_right'][0]:.2f}, {item['bottom_right'][1]:.2f})")
        elif item['type'] == 'Polygon':
            print("Points:")
            for point in item['points']:
                print(f"  ({point[0]:.2f}, {point[1]:.2f})")


svg_file = SVG_PATH
coordinates = extract_coordinates_from_svg(svg_file)

# coordinates [
    # [0] = {type: '', point: [,]}
    # [1] = {type: '', point: [,]}
    # [2] = {type: '', point: [,]}
    
	# Rectangle
	# Top-Left: ()
    # Bottom-Right: ()
# ]

# last_point = [0.0, 0.0]
# for item in coordinates:
#     if item['type'] == 'Rectangle':
#         continue
#     for a,b in item['points']:
#         tx = 1 * (a/100 - 0.1)
#         ty = -1 * (b/100 - 0.35)
#         if item['type']=='Move':
#             tz = 0.2
#         elif item['type']=='Line':
#             tz = 0.1
#         if not last_point == [a,b]:
#             print(f'[{tx:.3f} ,{ty:.3f} ,{tz:.3f}],')
#             plt.plot(tx, ty, 'o')
#         last_point = [a,b]

# plt.show()

# print_coordinates(coordinates)
for item in coordinates:
    print(item)
# plot_coordinates(coordinates)


