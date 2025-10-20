import time
import cv2 as cv
import mediapipe as mp

cam = cv.VideoCapture(0)
mpHands = mp.solutions.hands
hands = mpHands.Hands(False, 1)
mpDraw = mp.solutions.drawing_utils

def ard_map(value, fromLow, fromHigh, toLow, toHigh):
    return float((value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow)

pTime = 0
# hand_closed = False

while True:
    success, frame = cam.read()
    if not success:
        break

    frame = cv.flip(frame, 1)  # Flip for mirror effect
    imgRGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
    results = hands.process(imgRGB)

    h, w, c = frame.shape
    centerX, centerY = w // 2, h // 2

    if results.multi_hand_landmarks:
        for handLms in results.multi_hand_landmarks:
            x4 = y4 = x8 = y8 = 0
            cx4 = cy4 = cx8 = cy8 = 0

            for id, lm in enumerate(handLms.landmark):
                if id == 9:
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    cv.circle(frame, (cx, cy), 5, (0, 255, 0), cv.FILLED)
                    cv.line(frame, (centerX, centerY), (cx, cy), (255, 255, 255), 2)
                    
                    x = ard_map(lm.x, 0, 1, -1, 1)
                    y = ard_map(lm.y, 0, 1, -1, 1)
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
    fps = 1 / (cTime - pTime) if cTime != pTime else 0
    pTime = cTime

    cv.circle(frame, (centerX, centerY), 4, (255, 0, 0), cv.FILLED)
    cv.putText(frame, f'FPS: {int(fps)}', (10, 40), cv.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)

    cv.imshow("Hand Tracking", frame)
    if cv.waitKey(1) & 0xFF == 27:
        break

cam.release()
cv.destroyAllWindows()
