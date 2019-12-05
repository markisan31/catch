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


picar.setup()

fw = front_wheels.Front_Wheels()
bw = back_wheels.Back_Wheels()
camera_servo = Servo.Servo(1)
catch_servo = Servo.Servo(2)
picar.setup()

fw.offset = 10
#camera_servo.offset = 30
catch_servo.offset = 0
fw.turn(78)

catch_servo.write(105)
servopos = True
servopos1 = False
turn_back = False
camera_servo.write(70)
motor_speed = 40

PULL_UP = 105
PULL_DOWN = 90
ball_catched = False

def nothing(x):
    pass

cap = WebcamVideoStream(src=0).start()
fps = FPS().start()
center = (0, 0)
error_x = 0

time.sleep(1)

while(1):
    #     if turn_back == True:
    # #         bw.forward()
    #         camera_servo.write(70)
            
        # Take each frame
    frame = cap.read()
    frame = imutils.resize(frame, width=400)
        #_, frame = cap.read()

        # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define range of orange color in HSV
    ##    lower_orange = np.array([5,75,125])
    lower_orange = np.array([1,62,239])
    upper_orange = np.array([23,184,255])
    lower_gray = np.array([100,60,100])
    upper_gray = np.array([130,255,255])

        # Threshold the HSV image to get only orange colors
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    _,contours,hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE )
    if ball_catched == False:
            
        if len(contours) > 0 :
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0 :
                turn_back = False
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                cv2.circle(res, center, 5, (0, 0, 255), -1)
                    
                cv2.drawContours(frame,c,-1,(0, 0, 255),1)
                cv2.circle(frame, center, 7, (0, 0, 255), -1)
                cv2.line(frame,(center[0],0),(center[0],480),(0,0,255),1) 
                cv2.line(frame,(0,center[1]),(648,center[1]),(0,0,255),1) 
                    
                cv2.drawContours(res,c,-1,(0, 0, 255),2)
                cv2.circle(res, center, 7, (0, 0, 255), -1)
                cv2.line(res,(center[0],0),(center[0],480),(0,0,255),1) 
                cv2.line(res,(0,center[1]),(648,center[1]),(0,0,255),1)
                print("x: " + str(center[0]))
                print("y: " + str(center[1]))

                bw.speed = motor_speed
                if center[0] < 85 and center[1] > 150:
                    fw.turn(38)
                    bw.backward()
                    print("Ball is on full left")
                elif center[0] > 85 and center[0] < 170 and center[1] > 150:
                    fw.turn(63)
                    bw.backward()
                    print("ball is on left")
                elif center[0] > 230 and center[0] < 315 :
                    fw.turn(93)
                    bw.backward()
                    print("ball is on right")
                elif center[0] > 315 and center[1] > 150:
                    fw.turn(108)
                    bw.backward()
                    print("ball is on full right")
                elif center[0]  > 170 and center[0] < 230 and center[1] > 150:
                    fw.turn(78)
                    print("ball is on center")
                if servopos == False :
                    print("hello")
                    if center[0]  > 170 and center[0] < 230 and center[1] > 200 :
                        fw.turn(78)
                            #catch_servo.write(100)
                            #time.sleep(0.2)
                            #catch_servo.write(110)
                            #time.sleep(0.2)
                        catch_servo.write(PULL_DOWN)
                        camera_servo.write(70)
    #                         bw.stop()  
                        ball_catched = True
    #                         motor_speed = 0
                        print("ball catched")
                        servopos1 = True
                if servopos == True :
                    print(center[1])
                        
                    if center[1] > 185 :
                        print("yes")
                        camera_servo.write(45)
                        servopos = False
    #                 else:
    #                     catch_servo.write(105)
            else :
                turn_back = True
                    
            
    #     cv2.imshow('frame',frame)
        #cv2.imshow('mask',mask)
        #cv2.imshow('res',res)
    k = cv2.waitKey(1) & 0xFF
    

    ###############################


        # Threshold the HSV image to get only gray colors

        
    mask_gray = cv2.inRange(hsv, lower_gray, upper_gray)

        # Bitwise-AND mask and original image
    res_gray = cv2.bitwise_and(frame,frame, mask= mask_gray)

    _,contours,hierarchy = cv2.findContours(mask_gray, cv2.RETR_EXTERNAL , cv2.CHAIN_APPROX_SIMPLE )
    if ball_catched == True:
        if len(contours) > 0 :
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M["m00"] > 0 :
                center_gray = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                cv2.circle(res_gray, center_gray, 5, (0, 0, 255), -1)
                    
                cv2.drawContours(frame,c,-1,(0, 0, 255),1)
                cv2.circle(frame, center_gray, 7, (255, 0, 0), -1)
                cv2.line(frame,(center_gray[0],0),(center_gray[0],480),(255,0,0),1) 
                cv2.line(frame,(0,center_gray[1]),(648,center_gray[1]),(255,0,0),1) 
                    
                cv2.drawContours(res_gray,c,-1,(0, 0, 255),2)
                cv2.circle(res_gray, center_gray, 7, (255, 0, 0), -1)
                cv2.line(res_gray,(center_gray[0],0),(center_gray[0],480),(255,0,0),1) 
                cv2.line(res_gray,(0,center_gray[1]),(648,center_gray[1]),(255,0,0),1) 
                bw.speed = motor_speed
                print("x: " + str(center[0]))
                print("y: " + str(center[1]))
                if center_gray[0] < 85 :
                    fw.turn(38)
                    bw.backward()
                    print("grey found")
                elif center_gray[0] > 85 and center_gray[0] < 170 :
                    fw.turn(63)
                    bw.backward()
                    print("gray found")
                elif center_gray[0] > 230 and center_gray[0] < 315 :
                    fw.turn(93)
                    bw.backward()
                    print("gray found")
                elif center_gray[0] > 315 :
                    fw.turn(108)
                    bw.backward()
                    print("gray found")
                if servopos1 == False:
                        
                    if center_gray[1] > 200 :
                        fw.turn(78)
                            #catch_servo.write(100)
                            #time.sleep(0.2)
                            #catch_servo.write(110)
                            #time.sleep(0.2)
                        catch_servo.write(PULL_UP)
                            # camera_servo.write(70)
    #                         bw.forward()
                            
                            # print("ball is on center")
#                         ball_catched = False
                        turn_back = True
                        bw.forward()
                        fw.turn(58)
                        time.sleep(5)
                        ball_catched = False
                if servopos1 == True :
                    print(center[1])
                    print("mis toimub?")
                    if center[1] > 185 :  
                        camera_servo.write(45)
                        servopos1 = False
                    # elif center[1] > 250 :
                        # camera_servo.write(45)
                    # else:
                        # catch_servo.write(105)
            elif M["m00"] == 0 :
                fw.turn(63)
                bw.backward()
                    # print("no balls detected")
    #                 bw.stop()

    cv2.imshow('frame',frame)
#     cv2.imshow('mask_gray',mask_gray)
    # #cv2.imshow('res_gray',res_gray)
    # k = cv2.waitKey(1) & 0xFF

    # ######################################



    # error_x = round((center[0] - 320) / 320 * 100)
    # steering = round(error_x * min(1, ((center[1] + 120) / 480)))
    # if (center[1] > 360) and abs(error_x) < 10:    
    #     print("In position")
    # else:
    #     #print(error_x)
    #     print(steering)

    
    
    
    if k == 27:
        bw.stop()
        break

cv2.destroyAllWindows()
