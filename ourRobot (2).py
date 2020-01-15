#import NEWUSER as user
import time
import math
from multiprocessing import Process, Queue
import copy
import pickle
import matplotlib.pyplot as plt
import newBallTracker as tracker



from picar.SunFounder_PCA9685 import Servo
from picar import back_wheels
import imutils # pip3 install imutils
import picar
import numpy as np
import cv2
from ourDetectBalls import detectBalls
import getData

picar.setup()
steer_servo = Servo.Servo(0)
pan_servo = Servo.Servo(1)
tilt_servo = Servo.Servo(2)
bw = back_wheels.Back_Wheels()
bw.stop()
time.sleep(2)

print("\n\n\n")

pan_servo.write(90)
tilt_servo.write(90)
steer_servo.write(90)

my_robot_number = 0    #Minu roboti number, (markeri peal kirjas)
my_robot_data = []

start_time = time.time()
count = 0
resize = 2
found = False
gotcha = False




#cap = cv2.VideoCapture(-1)
center = (0, 0)
center_gray = (0, 0)
error_x = 0

o_par = [1, 19, 300, 8, 8, 14]
lower_orange = np.array([16, 94, 210])     
upper_orange = np.array([106, 188, 255])

blue_par = [1, 100, 300, 20, 35, 83]
lower_blue = np.array([1, 1, 1]) 
upper_blue = np.array([106, 90, 100])

res_x = 1920
res_y = 1080
coordinatorData = [0,0]
data_time = 0.0
data_count = 0
data_balls = []
data_rings = []
data_robots = []
new_robots = []
new_balls = []
new_rings = []

margin = 80  #pixels

def getFrame():
    _, frame1 = cap.read()
    frameRGB = cv2.resize(frame1, (int(640/resize), int(480/resize)))
    frame = cv2.cvtColor(frameRGB, cv2.COLOR_RGB2HSV)
    return frame


def mainFunction(balls, rings, robots):
    #Sinu kood siia
    #Example - saa minu roboti andmed
    for robot in robots:
        if robot[0] == my_robot_number:
            my_robot_data = robot
    print("Minu roboti asukoht: ", my_robot_data[1], " ja nurk: ", my_robot_data[2])
    pass


q = Queue()
p = Process(target=getData.getData, args=(q,))
p.start()

def readData():
    #print("READ DATA")
    global coordinatorData
    updated = False
    while not updated:
        new_globalData = q.get()
        if new_globalData[1] > coordinatorData[1]:
            coordinatorData = new_globalData
            #print("Sain uued andmed")
            updated = True
        elif new_globalData[1] == coordinatorData[1]:
            #print("Eelmised andmed kordusid")
            updated = False
        else:
            print("Andmeid ei saanud")
            updated = False
        #print(globalData)
        time.sleep(0.5)
  

def processData():
    global coordinatorData, data_time, data_counts, data_balls, data_rings, data_robots, new_robots, new_balls, new_rings

    #print(len(coordinatorData))
    
    #print("doin something")
    data_time = coordinatorData[0]           #Kellaaeg, millal server saatis andmed
    data_count = coordinatorData[1]          #Andmepaki number
    data_balls = coordinatorData[2]          #Pallide koordinaadid jms. [[x, y, r, inside], [x2, y2, r2, inside2], ...]
    data_rings = coordinatorData[3]          #Rongaste koordinaadid jms. [[x, y, r, count], [x2, y2, r2, count2], ...]
    data_robots = coordinatorData[4]         #Robotite koordinaadid jms. [[num, [x, y], angle], [num2, [x2, y2], angle2], ...]
    
    #print(data_robots)
    #print(data_balls)
    print("delay serveri ja roboti vahel", round(time.time() - data_time, 2))


    time.sleep(0.01)

    #print(data_time, data_count, balls, rings, robots)
 
    #print("co data", coordinatorData)
    balls_x = []
    balls_y = []
    rings_x = []
    rings_y = []
    corners_x = [0, 1920]
    corners_y = [0, 1080]
    plt.clf()
    
    new_robots = copy.deepcopy(data_robots)
    for i, robot in enumerate(new_robots):
        new_robots[i][1][1] = res_y - int(data_robots[i][1][1])
        new_robots[i][2] = math.radians(new_robots[i][2])
        
        if new_robots[i][0] == 0:
            our_robot_x = new_robots[i][1][0]
            our_robot_y = new_robots[i][1][1]
            our_angle = new_robots[i][2]
            plt.scatter(our_robot_x, our_robot_y, color="g")

            #print(our_angle, "new dx", math.cos(our_angle)*100, "new dy", (math.sin(our_angle)*100))
            plt.arrow(our_robot_x, our_robot_y, math.cos(our_angle)*100, math.sin(our_angle)*100)
        
    new_balls = copy.deepcopy(data_balls)
    for i, ball in enumerate(new_balls):
        new_balls[i][1] = res_y - int(data_balls[i][1])
        
        balls_y.append(new_balls[i][1])
        balls_x.append(new_balls[i][0])
        
    new_rings = copy.deepcopy(data_rings)
    for i, ring in enumerate(new_rings):
        new_rings[i][1] = res_y - int(data_rings[i][1])
    
        rings_y.append(new_rings[i][1])
        rings_x.append(new_rings[i][0])
        
    plt.scatter(corners_x, corners_y, color="w")
    plt.scatter(balls_x, balls_y, color="r")
    plt.scatter(rings_x, rings_y, color="b")
    plt.pause(0.05)

    

 
    return coordinatorData
    
def getRobot(num=0):
    #print("getting new robot data")
    for robot in new_robots:
        if robot[0] == num:
            #print("Minu roboti asukoht: ", robot[1], " ja nurk: ", robot[2])
            return robot
        
def getClosestBall(pos1, isInside=False):
    min_dist = 999999
    for ball in new_balls:
        if ball[3] == isInside:
            distance = ((ball[0]-pos1[0])**2 + (ball[1]-pos1[1])**2)**0.5
            if distance < min_dist:
                min_dist = distance
                position = [ball[0], ball[1]]                
    return position

def getClosestRing(pos1, numInside):   #Get closest ring that has X or less balls inside
    min_dist = 999999
    for ring in new_rings:
        #print("rings:", ring)
        if ring[3] <= numInside:
            distance = ((ring[0]-pos1[0])**2 + (ring[1]-pos1[1])**2)**0.5
            if distance < min_dist:
                min_dist = distance
                position = [ring[0], ring[1]]
    return position

def calculateAbsAngle(pos1, pos2):
    abs_angle = math.atan2(pos2[1]-pos1[1], pos2[0]-pos1[0])
    return abs_angle


def calculateTurnAngle(robot_angle, abs_angle):
    #print(robot_angle, abs_angle)
    turn_angle = math.radians((math.degrees(abs_angle) - math.degrees(robot_angle) + 540)%360 - 180)        
    return turn_angle
   
def calculateDist(pos1, pos2):
    distance = ((pos1[0]-pos2[0])**2+(pos1[1]-pos2[1])**2)**0.5
    return distance

def turnTo(robot_angle, target_angle):
    #print("rob angle ja target ang", robot_angle, target_angle)
    turn_angle = calculateTurnAngle(robot_angle, target_angle)
    print("turn angle", math.degrees(turn_angle))

    bw.turn(math.degrees(turn_angle))
    

def goTo(robot_angle, robot_pos, target_pos):
    target_angle = calculateAbsAngle(robot_pos, target_pos)
    
    turnTo(robot_angle, target_angle)
    bw.stop()
    
    distance = calculateDist(robot_pos, target_pos) - margin
    if distance < 0:
        distance = 0
    #print(distance, "distance")
    
    bw.straight(distance*0.3333)
    bw.stop()  
    
    


def detectOrange():
    frame = getFrame()
    balls = detectBalls("usb cam view", frame, True, lower_orange, upper_orange, o_par[0], o_par[1], o_par[2], o_par[3], o_par[4], o_par[5])
    return balls

def detectBlue():
    frame = getFrame()
    rings = detectBalls("usb cam view", frame, True, lower_blue, upper_blue, b_par[0], b_par[1], b_par[2], b_par[3], b_par[4], b_par[5])
    return rings


def data():
    readData()
    processData()

try:
    for i in range(1000):
        try:
            data()
            data()
            print("next iteration", i)
            #print(len(coordinatorData), "read")
            #getData()  #kirjutab serveri saadetud andmed global muutujatesse
            our_robot = getRobot(0)
            #print("ourrobot",our_robot)
            robot_angle = our_robot[2]
            robot_pos = our_robot[1]
            
            target_ball = getClosestBall(robot_pos)   #kirjutab lahima palli koordinaadid muutujasse
            plt.scatter(target_ball[0], target_ball[1], color="k", s=15)
            plt.pause(1.03)
            #print("robot angle is:", robot_angle)
            
            
            
            goTo(robot_angle, robot_pos, target_ball)
            
            tracker.captureDeliver(False)
            
            #time.sleep(1)   
            data()
            data()
            #getData()  #kirjutab serveri saadetud andmed global muutujatesse
            our_robot = getRobot(0)
            robot_angle = our_robot[2]
            robot_pos = our_robot[1]
            #print("robot angle is:", robot_angle)

            
            target_ring = getClosestRing(robot_pos, 3)    #kirjutab lahima ronga, milles on <4 palli, koordinaadid muutujasse
            plt.scatter(target_ring[0], target_ring[1], color="w", s=15)
            plt.pause(1.03)

            goTo(robot_angle, robot_pos, target_ring)
            tracker.captureDeliver(True)

            #time.sleep(1)
            
        except Exception as e:
            print("error ----", e)
            #tracker.stop()
            time.sleep(0.1)
            #break
            
except Exception as e:
    print("final error", e)
    tracker.stop()