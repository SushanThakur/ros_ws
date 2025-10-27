frame_id = "base_link"
joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7', 'tool_joint']
gripper_joints = ['grip_left_joint', 'grip_right_joint']

open_pos = [0.0, 0.0]
close_pos = [-0.04, 0.04]

SVG_FILE = "svg/workspace.svg"

SCALE_FACTOR = 0.001       # Converts SVG px → meters
PEN_DOWN_Z = 0.07          # Z height when drawing
PEN_UP_Z = 0.15            # Z height when moving
ST_WIDTH = 1.0

default_orient = [
    0.7068251814624406,
    0.7073873723632136,
    0.0011261755880557509,
    0.0
]

PATH = 'recording/pos.txt'