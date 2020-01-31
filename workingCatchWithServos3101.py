from picar import front_wheels, back_wheels
from picar.SunFounder_PCA9685 import Servo
import picar
import cv2
import numpy as np
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import argparse
import imutils
import time
import random


picar.setup()

fw = front_wheels.Front_Wheels()
bw = back_wheels.Back_Wheels()
camera_servo = Servo.Servo(1)
catch_servo = Servo.Servo(2)
picar.setup()

fw.offset = 10
catch_servo.offset = 0
fw.turn(78)

catch_servo.write(105)

servopos = True
servopos1 = False
camera_servo.write(70)
motor_speed = 40

PULL_UP = 105
PULL_DOWN = 90
just_ignored_ring = False
ball_catched = False

cap = WebcamVideoStream(src=0).start()
fps = FPS().start()
center = (0, 0)
error_x = 0

time.sleep(3)

def catch_ball():
    global ball_catched, servopos, servopos1, just_ignored_ring
#     bw.speed = motor_speed
    if not ball_catched:
        if len(contours_orange) > 0:
            c = max(contours_orange, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0:
                print("hello")
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                cv2.circle(res, center, 5, (0, 0, 255), -1)

                cv2.drawContours(frame, c, -1, (0, 0, 255), 1)
                cv2.circle(frame, center, 7, (0, 0, 255), -1)
                cv2.line(frame, (center[0], 0), (center[0], 480), (0, 0, 255), 1)
                cv2.line(frame, (0, center[1]), (648, center[1]), (0, 0, 255), 1)

                cv2.drawContours(res, c, -1, (0, 0, 255), 2)
                cv2.circle(res, center, 7, (0, 0, 255), -1)
                cv2.line(res, (center[0], 0), (center[0], 480), (0, 0, 255), 1)
                cv2.line(res, (0, center[1]), (648, center[1]), (0, 0, 255), 1)

                bw.speed = motor_speed
                print("x is " + str(center[0]))
                print("y is " + str(center[1]))
                print()
                if center[0] < 85 and center[1] > 80:
                    fw.turn(38)
                    bw.backward()

                elif 85 < center[0] < 170 and center[1] > 80:
                    fw.turn(63)
                    bw.backward()

                elif 230 < center[0] < 315:
                    fw.turn(93)
                    bw.backward()

                elif center[0] > 315 and center[1] > 80:
                    fw.turn(108)
                    bw.backward()

                elif 170 < center[0] < 230 and center[1] > 80:
                    fw.turn(78)

                    if center[1] > 150:
                        camera_servo.write(45)

                if not servopos:
                    if 170 < center[0] < 230 and center[1] > 200:
                        camera_servo.write(45)
                        fw.turn(78)

                        catch_servo.write(PULL_DOWN)
                        bw.forward()
                        time.sleep(2)
                        bw.stop()
                        print("checkin' if ball catched???")
                        
                        if 170 < center[0] < 230 and center[1] > 200:
                            camera_servo.write(60)
                            first_ball_catched = True
                            ball_catched = True
                            print("ball catched")
                            bw.speed = motor_speed
                            # fw.turn(38)
                            bw.forward()
                            # fw.turn(78)
                            time.sleep(1)
    #                         servopos1 = True
                        else:
                            print("ball not catched.. try again")
                if servopos:
                    if center[1] > 185:
                        servopos = False
            else:
                print("no hello")
                camera_servo.write(70)
                fw.turn(random.randint(38,108))
                print("no balls")
                bw.speed = motor_speed
                time.sleep(1)
                if just_ignored_ring == False:
                    bw.forward()
                else:
                    bw.backward()
                    time.sleep(2)
                    just_ignored_ring = False
    
def ignore_ring_while_finding_ball():
    global just_ignored_ring()
    if len(contours_green) > 0:
        c = max(contours_green, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] > 2000:
            print("ignoring ringast")
            bw.backward()
            fw.turn(38)
            camera_servo.write(70)            
#             bw.speed = motor_speed
#             bw.backward()
            time.sleep(3)
            just_ignored_ring = True
            # fw.turn(108)
            # time.sleep(2)


def find_ring_and_put_ball():
    global ball_catched, servopos, servopos1

    if len(contours_green) > 0:
        c = max(contours_green, key=cv2.contourArea)
        M = cv2.moments(c)
        print(M["m00"])
        if M["m00"] > 10:
            center_green = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(res_green, center_green, 5, (0, 0, 255), -1)

            cv2.drawContours(frame, c, -1, (0, 0, 255), 1)
            cv2.circle(frame, center_green, 7, (255, 0, 0), -1)
            cv2.line(frame, (center_green[0], 0), (center_green[0], 480), (255, 0, 0), 1)
            cv2.line(frame, (0, center_green[1]), (648, center_green[1]), (255, 0, 0), 1)

            cv2.drawContours(res_green, c, -1, (0, 0, 255), 2)
            cv2.circle(res_green, center_green, 7, (255, 0, 0), -1)
            cv2.line(res_green, (center_green[0], 0), (center_green[0], 480), (255, 0, 0), 1)
            cv2.line(res_green, (0, center_green[1]), (648, center_green[1]), (255, 0, 0), 1)

            bw.speed = motor_speed

            if center_green[0] < 85:
                fw.turn(38)
                bw.backward()

            elif 85 < center_green[0] < 170:
                fw.turn(63)
                bw.backward()

            elif 230 < center_green[0] < 315:
                fw.turn(93)
                bw.backward()

            elif center_green[0] > 315:
                fw.turn(108)
                bw.backward()

            if not servopos1:
                if 150 < center_green[1] < 200:
                    camera_servo.write(45)

                elif center_green[1] > 200:
                    fw.turn(78)
                    catch_servo.write(PULL_UP)
                    print("ball released")
                    fw.turn(60)
                    camera_servo.write(70)
                    bw.forward()
                    # fw.turn(38)                  
                    time.sleep(5)
                    ball_catched = False
                    
            if servopos1:
                if center[1] > 185:
                    servopos1 = False

        else:
            camera_servo.write(70)
            fw.turn(random.randint(38,108))
            print("no rings")
#             bw.speed = motor_speed
            bw.forward()
            time.sleep(1)



while (1):
    frame = cap.read()
    frame = imutils.resize(frame, width=400)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of orange color in HSV
    lower_orange = np.array([1, 62, 239])
    upper_orange = np.array([23, 184, 255])
    # define range of green color in HSV
    lower_green = np.array([46, 101, 42])
    upper_green = np.array([95, 255, 243])

    # Threshold the HSV image to get only orange colors
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=mask)

    _, contours_orange, hierarchy_orange = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # Bitwise-AND mask and original image
    res_green = cv2.bitwise_and(frame, frame, mask=mask_green)

    _, contours_green, hierarchy_green = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    if not ball_catched:
        catch_ball()        
        ignore_ring_while_finding_ball()
#         catch_ball()
    else:
        find_ring_and_put_ball()
    
    cv2.imshow('frame', frame)
    cv2.imshow('mask_green', mask_green)
    cv2.imshow('mask', mask)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        bw.stop()
        break

cv2.destroyAllWindows()
