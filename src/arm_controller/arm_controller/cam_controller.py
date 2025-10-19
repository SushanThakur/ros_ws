#!/home/sushant/ros_ws/ignore/test_env/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import mediapipe as mp
import time

class CamController(Node):

    def __init__(self):
        super().__init__('cam_controller')

        self.cam_sub = self.create_subscription(Image, 'cam_publisher', self.cam_call, 10)
        self.br = CvBridge()

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(False, 1)
        self.mpDraw = mp.solutions.drawing_utils
        self.pTime = 0

    def ard_map(self, value, fromLow, fromHigh, toLow, toHigh):
        return float((value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow)

    def cam_call(self, msg):
        self.get_logger().info('Receiving video frame')

        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        frame = cv.flip(frame, 1)
        imgRGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.hands.process(imgRGB)

        h,w,c = frame.shape
        centerX, centerY = w//2, h//2

        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks:
                x4 = y4 = x8 = y8 = 0
                cx4 = cy4 = cx8 = cy8 = 0

                for id, lm in enumerate(handLms.landmark):
                    if id==9:
                        cx, cy = int(lm.x * w), int(lm.y * h)
                        cv.circle(frame, (cx, cy), 5, (0, 255, 0), cv.FILLED)
                        cv.line(frame, (centerX, centerY), (cx, cy), (255, 255, 255), 2)
                        
                        x = self.ard_map(lm.x, 0, 1, -1, 1)
                        y = self.ard_map(lm.y, 0, 1, -1, 1)
                        cv.putText(frame, f'{x:.2f},{y:.2f}', (cx, cy), cv.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0), 1)

                    elif id == 4:
                        x4, y4 = lm.x, lm.y
                        cx4, cy4 = int(lm.x * w), int(lm.y * h)
                        cv.circle(frame, (cx4, cy4), 2, (0, 255, 0), cv.FILLED)
                    elif id == 8:
                        x8, y8 = lm.x, lm.y
                        cx8, cy8 = int(lm.x * w), int(lm.y * h)
                        cv.circle(frame, (cx8, cy8), 2, (0, 255, 0), cv.FILLED)

                 # Draw line between thumb & index tip
                cv.line(frame, (cx4, cy4), (cx8, cy8), (255, 255, 255), 1)

                dist = ((x4 - x8)**2 + (y4 - y8)**2)**0.5
                if dist < 0.04 :
                    cv.putText(frame, 'Closed', (cx4, cy4), cv.FONT_HERSHEY_COMPLEX, 0.7, (0,255,0), 2)

        cTime = time.time()
        fps = 1 / (cTime - self.pTime) if cTime != self.pTime else 0
        self.pTime = cTime

        cv.circle(frame, (centerX, centerY), 4, (255, 0, 0), cv.FILLED)
        cv.putText(frame, f'FPS: {int(fps)}', (10, 40), cv.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)

        cv.imshow("Hand Tracking", frame)
        cv.waitKey(1)

def main():
    try:
        rclpy.init()
        cam_controller = CamController()
        rclpy.spin(cam_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            cam_controller.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()