import cv2
import mediapipe as mp
import numpy as np
import time
from warp_field import *
DEBUG = True
if DEBUG:
    while True:
        try:
            from robot_control import *
            break
        except TimeoutError:
            print("TIMEOUT")

def normis(x, x1, x2, r=True):
    d = max(x1,x2) - min(x1,x2)
    y=x*d
    normx = min(x1,x2)+y
    if r: normx = round(normx, 4)
    return normx

def get_len(x, y):
    return (x**2 + y**2 )**(1/2)

def get_area2area_coords(x, y, fx1, fy1, fx2, fy2, tx1, ty1, tx2, ty2, clampmin=0.0, clampmax=1.0):
    w1 = fx2-fx1
    w2 = tx2-tx1
    h1 = fy2-fy1
    h2 = ty2-ty1
    fx, fy = x*w1, y*h1
    rx = (fx-tx1)/w2
    ry = (fy-ty1)/h2
    #rx, ry = x*dw, y*dh
    rx = min(max(clampmin, rx), clampmax)
    ry = min(max(clampmin, ry), clampmax)
    return (rx, ry)


dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

cv2.namedWindow("Arm2Arm", cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty("Arm2Arm",cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

#cv2.namedWindow("Arm2Arm")
warper = Warper(cache=True)

    
field_size = (500, 300)
mp_hands = mp.solutions.hands
k=1
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, model_complexity=0)
in_use = False
in_use_temp = 0
gripb = False
angle = 90
_, _, xs, ys = cv2.getWindowImageRect("Arm2Arm")
forupd = False
first_grab = True
cv2.getWindowImageRect("Arm2Arm")
fcap = cv2.VideoCapture(1)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS,60)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
#round(field_size[0]/3*5) round(field_size[1]*2)
data = {0:[-0.200, 0.1,0.15]}
cx, cy = 1.0, 1.0
himg = cv2.imread("main.jpg")

if DEBUG:
    move_robot(*robot_positions["work_position"])
print("home")
time.sleep(3)
while True:
    bg = np.zeros([ys, xs, 3], dtype=np.uint8)
    #robot.stopl()
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)
    xnn, ynn = round(field_size[0]/3*5), round(field_size[1]*2)
    xdd = ys/ynn
    frame = cv2.resize(frame, (round(xnn*xdd-1), round(ynn*xdd-1)))
    if not ret:
        raise Exception("No cap!")
    h, w = frame.shape[:2]
    results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    data = {}
    if results.multi_hand_landmarks:
        #hm = list(set(lm.value for lm in list(mp_hands.HandLandmark)) & set([0]))
        for handLandmarks in results.multi_hand_landmarks:
            #pass
            #for point in mp_hands.HandLandmark:
            for point in [0, 4, 5, 8, 17, 16, 20]:
                normlm = handLandmarks.landmark[point]
                x, y, z = normlm.x, normlm.y, normlm.z
                iax, iay = get_area2area_coords(x, y, 0, 0, w, h, w//5*1, h//2, w//5*4, h)
                fcoords = mp_drawing._normalized_to_pixel_coordinates(x, y, w, h)
                if point != 0:
                    frame = cv2.circle(frame, fcoords, 5, (0, 255, 0), -1)
                frame = cv2.putText(frame, f"{point}: {round(x, 2)}/{round(y, 2)}/{round(z, 2)} | {round(iax, 1)}/{round(iay, 1)}", fcoords, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2)
                #data[point] = [normis(x*5//4, -0.4, 0.1), normis(y*2,-0.3, 0.3), normis(z, 0.01, 0.29), x, y, z]
                data[point] = [normis(iay, -0.375, -0.125), normis(iax,-0.125, 0.250), normis(z, 0.01, 0.29), x, y, z]
    if not all(i in data for i in [0, 4, 5, 8, 17, 16, 20]):
        bg = cv2.putText(bg, f"No hand", (xs-300, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (64, 255, 255), 2)
        in_use = False
    else:
        k=h/round(get_len(data[0][3]-data[5][3], data[0][4]-data[5][4]), 5)
        if k < 1650:
            bg = cv2.putText(bg, f"Too close", (xs-300, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (64, 255, 255), 2)
            in_use = False
        elif k > 5000:
            bg = cv2.putText(bg, f"Too far", (xs-300, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (64, 255, 255), 2)
            in_use = False
        else:
        #print(k)
        #print(abs(k*data[4][0]-k*data[17][0]))
            if abs(data[4][3]-data[20][3]) < 0.04 and abs(data[4][4]-data[20][4]) < 0.04 and abs(data[4][5]-data[20][5]) < 0.05 and in_use:
                angle += 5
                forupd = True
            elif abs(data[4][3]-data[16][3]) < 0.04 and abs(data[4][4]-data[16][4]) < 0.04 and abs(data[4][5]-data[16][5]) < 0.05 and in_use:
                angle -= 5
                forupd = True
            if abs(data[4][3]-data[17][3]) < 0.04 and abs(data[4][4]-data[17][4]) < 0.04 and abs(data[4][5]-data[17][5]) < 0.05 and in_use_temp <= 0:
                in_use = not in_use
                in_use_temp = 5
            elif in_use_temp > 0:
                in_use_temp -= 1
            if abs(data[4][3]-data[8][3]) < 0.05 and abs(data[4][4]-data[8][4]) < 0.05 and abs(data[4][5]-data[8][5]) < 0.075 and in_use_temp <= 0 and in_use:
                gripb = not gripb
                first_grab = True
                forupd = True
                in_use_temp = 15
            elif in_use_temp > 0:
                in_use_temp -= 1
                
            if in_use:
                bg = cv2.putText(bg, f"In use: {in_use}", (xs-300, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                bg = cv2.putText(bg, f"Grip: {gripb} | Angle: {angle}", (xs-300, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                frame = cv2.rectangle(frame, (w//5*1, h//2), (w//5*4, h), (0, 0, 0), 3)
                frame = cv2.rectangle(frame, (w//5*1, h//2), (w//5*4, h), (255, 255, 255), 1)
                infield = cv2.resize(warper.warp_field(fcap.read()[1]), [w//5*4-w//5*1, h - h//2])
                inframe = frame.copy()
                inframe[ h//2:h, w//5*1:w//5*4] = infield
                frame = cv2.addWeighted(frame,0.45,inframe,0.55,0)
            else:
                bg = cv2.putText(bg, f"In use: {in_use}", (xs-300, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                bg = cv2.putText(bg, f"Grip: {gripb} | Angle: {angle}", (xs-300, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            dx, dy = abs(data[0][3]-cx), abs(data[0][4]-cy)
            if ((dx > 0.015 or dy > 0.015) or forupd) and in_use:
                #if DEBUG:
                #    robot.stopl()
                cx, cy = data[0][3], data[0][4]
                if DEBUG:
                    if gripb and first_grab:
                        move_robot(data[0][0], data[0][1], 0.22, angle, True)
                        grip.gripper_action(0, wait=True)
                        first_grab = False
                    elif gripb and not first_grab:
                        move_robot(data[0][0], data[0][1], 0.23, angle, False)
                        grip.gripper_action(0, wait=False)
                    else:
                        move_robot(data[0][0], data[0][1], 0.23, angle, False)
                        grip.gripper_action(50, wait=False)
                forupd = False
            frame = cv2.circle(frame, [int(data[0][3]*w), int(data[0][4]*h)], 8, (255, 255, 255), -1)
            frame = cv2.circle(frame, [int(data[0][3]*w), int(data[0][4]*h)], 7, (0, 0, 0), -1)
            frame = cv2.circle(frame, [int(data[0][3]*w), int(data[0][4]*h)], 6, (128, 0, 255), -1)
    bg[0:h, 0:w] =  cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    bg[ys-700:ys, xs-300:xs] = himg
    #cv2.imwrite(f"save/{time.time()}.jpg", bg)
    cv2.imshow("Arm2Arm", bg)
    if cv2.waitKey(1000//30) == ord("q"):
        print("End")
        break
cap.release()
cv2.destroyAllWindows()

