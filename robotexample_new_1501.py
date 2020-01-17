# coding=utf-8
import NEWUSER as user
import time
import math
import sys
sys.path.insert(1, './remote_control/remote_control')
from driver import camera
from picar import back_wheels, front_wheels
import picar
import Watcher
#import watcher

db_file = "/home/pi/SunFounder_PiCar-V/remote_control/remote_control/driver/config"
fw = front_wheels.Front_Wheels(debug=False, db=db_file)
bw = back_wheels.Back_Wheels(debug=False, db=db_file)
cam = camera.Camera(debug=False, db=db_file)
cam.ready()

image_size = [1920, -1080] #GPS camera image size in pixels
center_point = [830, -660] #center point of circle
camera_height = 510     #cm from ground


my_robot_number = 2     # Minu roboti number, (markeri peal kirjas)
cm = 3                  # camera pixels per 1 cm on the floor
robot_length = 17 * cm  # distance from marker center to ballgrabber
min_r = 26 * cm         # robots minimal turning radius
marker_height = 12      # cm from ground


#global robot_position_history
robot_position_history = list()
my_robot_data = []
bw.ready()
fw.ready()
SPEED = 50
#bw_status = 0
time.sleep(1)

def taylor_sin(angle_degrees):  #faster way to calculate sinus (angle in degrees)
    if angle_degrees > 180:
        angle_degrees -= 360
    elif angle_degrees < -180:
        angle_degrees += 360

    a = angle_degrees * 3.14 / 180
    taylor_sin = a - (a**3)/6 + (a**5)/120 - (a**7)/5040 + (a**9)/362880
    
    return round(taylor_sin, 3)

def taylor_cos(angle_degrees):  #faster way to calculate cosinus (angle in degrees)
    if angle_degrees > 180:
        angle_degrees -= 360
    elif angle_degrees < -180:
        angle_degrees += 360

    a = angle_degrees * 3.14 / 180
    taylor_cos = 1 - (a**2)/2 + (a**4)/24 - (a**6)/720 + (a**8)/40320
    
    return round(taylor_cos, 3)

def robot_definer(robots, my_robot_number): # checks robots visibility, removes perspective effect, returns robot with grabber as origin for local coordinates[num, [x_grabber, y_grabber], fi]
    #global robot_position_history
    my_robot_visibility = False
    
    try:
        for robot in robots:
            if robot[0] == my_robot_number:
                my_robot_visibility = True
                my_robot_data = robot
                robot_position_history.append(my_robot_data)
                
    except Exception as e:
        pass
    
    if my_robot_visibility == False:
        print ("Error 404: Robot not found! ")
        pass
    else:
        #print ("pos on camera " + str(robot_position_history[-1]))
        robot_on_ground = robot_position_history[-1]
        
        robot_on_ground[1] = de_perspective(image_size[0], image_size[1], camera_height, marker_height, robot_position_history[-1][1][0], robot_position_history[-1][1][1])
        #print ("pos on ground " + str(robot_on_ground))

        grabber_position = robot_sharp_end(robot_on_ground, robot_length)
        #print ("pos of grabber " + str(grabber_position))
        
        return grabber_position

def robot_sharp_end(robot, robot_length):     #use robots ballgrabber as robots coordinates origin
    fi = robot[2]
    x = robot[1][0] + robot_length * taylor_cos(fi)
    y = robot[1][1] + robot_length * taylor_sin(fi)
    robot[1][0] = round(x)
    robot[1][1] = round(y)
    return robot


def de_perspective(image_size_x, image_size_y, camera_height, marker_height, x_visible, y_visible): #projects markers visible position to true position on the ground
    x_midpoint = image_size_x / 2
    y_midpoint = image_size_y / 2
    h = marker_height
    H = camera_height

    x_true = x_visible * (1 - h / H) + h / H * x_midpoint
    y_true = y_visible * (1 - h / H) + h / H * y_midpoint

    return [round(x_true), round(y_true)]

def ball_vector(balls, robot):
    ball_vectors = list()   #place-vectors for all free balls with robot as origin
    #balls = sorted(balls)   #just some sorting to reduce clutter. Is now implemented in GPS
    for ball in balls:
        if ball[3] == False:    #ball status 0 - free
            
            if math.sqrt((center_point[0] - ball[0]) ** 2 + (center_point[1] - ball[1]) ** 2) < 600: # Calculate ball distance from center. Avoid balls in the edge (outside of circle R=550 pixels)
                ball_vectors.append([ball[0] - robot[1][0], (ball[1] - robot[1][1])])  #If all good, add to ball vector list
        
    return ball_vectors #[[_x0, _y0], [_x1,_y1] ...]

def ball_chooser(ball_vectors, robot, min_r):  #Find ball that is easiest to travel to
    nice_balls = list()
    
    for ball in ball_vectors:
        distance_to_ball = round(math.sqrt(ball[0] ** 2 + ball[1] ** 2))
        angle_to_ball = round((180 * math.atan2(ball[1], ball[0]) / 3.14) - robot[2])
        
        if angle_to_ball <= -180:
            angle_to_ball += 360
        
        road_to_ball = round((abs(angle_to_ball) * 3.14 * min_r / 180) + distance_to_ball)  # approximate road to travel to reach ball
       
        if distance_to_ball > (min_r * 2 * abs(taylor_sin(angle_to_ball))) and distance_to_ball > 6*cm:   # avoid balls right next to robot (too close to reach) or in grabber
            nice_balls.append([road_to_ball, distance_to_ball, angle_to_ball, ball[0], ball[1]])

    best_balls = sorted(nice_balls)  # balls sorted by road to travel
    
    best_ball = best_balls[0]
    best_ball.pop(0)     #remove road_to_ball. it is not needed
    
    return best_ball  # [distance_to_ball, angle_to_ball, x, y, ball index]

def lykkaSISSE():   #robot pushes balls into ring and goes away
    global goal_type
    print("OLD goal type %s" % goal_type)
    bw.forward()
    bw.speed = 30
    time.sleep(2)
    
    cam.turn_up(-175) #open gate
    
    bw.speed = 50
    global my_ball_counter
    my_ball_counter = 0
    goal_type = "BALL"
    bw.backward()
    time.sleep(4)
    
    
   
    print("NEW goal type %s" % goal_type)
    
    print(my_ball_counter)


def myballs(ball_vectors): #counts number of balls in grabber
    counter = 0
    for ball in ball_vectors:
        distance_to_ball = round(math.sqrt(ball[0] ** 2 + ball[1] ** 2))
        if distance_to_ball < 6*cm:
            counter += 1
    return counter

def ring_chooser(rings, robot, myballs): #Chooses ring to put balls into
    nice_rings = list()
    ring_vector = list()
    for ring in rings:
        if ring[3] + myballs <= 5:  #Avoids putting too many balls in one ring
            ring_vector.append([ring[0]-robot[1][0],ring[1]-robot[1][1]])
            distance_to_ring = round(math.sqrt((ring_vector[-1][0]) ** 2 + (ring_vector[-1][1]) ** 2))
            
            angle_to_ring = round((180 * math.atan2(ring_vector[-1][1], ring_vector[-1][0]) / 3.14) - robot[2])
            if angle_to_ring <= -180:
                angle_to_ring += 360
                
            if distance_to_ring > (min_r * 2 * abs(taylor_sin(angle_to_ring))): 
                nice_rings.append([distance_to_ring, angle_to_ring, ring[0], ring[1]])

    best_rings = sorted(nice_rings)  # rings sorted by distance
    best_ring = best_rings[0] 

    return best_ring    #return closest ring [distance_to_ring, angle_to_ring, x, y]


def go_to(goal, goal_type):  # [distance_to_goal, angle_to_goal, x, y, ball index]

    angle_error = goal[1]
    distance_error = goal[0]
    print ("i'm going to %s: angle %s, distance %s" % (goal_type, angle_error, distance_error))

    if goal_type == "BALL":  #then go to best ball
        if distance_error > 0:  # drive the robot towards ball
            bw.speed = SPEED
            bw.forward()
            
            #time.sleep(1)
            if abs(angle_error) > 2:                # aim the robot at the goal
                fw.turn(90 + 0.8 * angle_error)     # angle control gain Kp
            
            else: fw.turn_straight()
            
        if distance_error < 60 and abs(angle_error) < 30:
            cam.turn_up(-175)
            print (">>>--------gate open--------<<<")
            time.sleep(0.2)
            cam.ready()
        
            
    
    elif goal_type == "RING": # go to best ring
                            
            if distance_error > 55:  # drive the robot towards goal
                
                bw.speed = SPEED 
                bw.forward()
            
                if abs(angle_error) > 2:  # aim the robot at the goal
                    fw.turn(90 + 0.8 * angle_error)  # angle control gain Kp
                    # print("Pööran ringi poole"  + str(90 + 0.8 * angle_error))
                else:
                    fw.turn_straight()
                    
                        
            elif distance_error < 55 and abs(angle_error) < 20: # Ring is in a good position
                lykkaSISSE()
                
                
def robot_clear_path_check(my_robot_data, rings, robots, goal_type, path_length = 15*cm, path_width = 15*cm):     #check if there is something in front of robot
    fi = (my_robot_data[2])
    x_path = my_robot_data[1][0] + path_length * taylor_cos(fi)        #point in front of robot
    y_path = my_robot_data[1][1] + path_length * taylor_sin(fi)        #point in front of robot
    
    if goal_type == "BALL":
        for ring in rings:

            if math.sqrt((ring[0] - x_path)**2 + (ring[1] - y_path)**2) < path_width:   #ring is in robots path
                angle_to_obstacle = math.degrees(math.atan2(ring[1] - my_robot_data[1][1], ring[0] - my_robot_data[1][0]))
                
                if angle_to_obstacle > fi: 
                    fw.turn_left()
                    time.sleep(0.2)
                else:
                    fw.turn_right()
                    time.sleep(0.2)
                #bw.speed = 40
                bw.forward()
                

                print("<<<<---------------------------Avoiding ring--------------------------->>>>")

    for robot in robots:
        if robot[0] != 2:
            if math.sqrt((robot[1][0] - x_path)**2 + (robot[1][1] - y_path)**2) < 2*path_width:  # other robot is in robots path
                angle_to_obstacle = math.degrees(math.atan2(robot[1][1] - my_robot_data[1][1], robot[1][0] - my_robot_data[1][0]))

                if angle_to_obstacle > fi:
                    fw.turn_left()
                else:
                    fw.turn_right()
                bw.speed = 40
                bw.forward()
                time.sleep(0.1)

                print("<<<<---------------------------Avoiding robot--------------------------->>>>")
            
            elif math.sqrt((robot[1][0] - x_path)**2 + (robot[1][1] - y_path)**2) < 60: # other robot is too close in front
                #time.sleep(0.5)
                print ("|||||---Backing up---|||||")
                bw.backward()
                time.sleep(1)

def stick_test(robot_position_history): #   Check if robot has moved


    if abs(robot_position_history[-25][1][0]-robot_position_history[-1][1][0])<10 and abs(robot_position_history[-25][1][1]-robot_position_history[-1][1][1])<10:

        if len(robot_position_history)>50 and abs(robot_position_history[-50][1][0] - robot_position_history[-1][1][0]) < 10 and abs(robot_position_history[-50][1][1] - robot_position_history[-1][1][1]) < 10:
            print("I'm SO stuck")
            bw.forward()
            time.sleep(2)
        else:
            print("I'm stuck")
            bw.backward()
            time.sleep(2)


def coordinator():
    goal = None
    global goal_type
    goal_type = "BALL"
    global my_ball_counter
    my_ball_counter = 0
    data_time_last = 0
    ring_counter = 0
    cycle_counter = 0
    while 1:
        try:
            #Andmete kirjutamine muutujatesse
            coordinatorData = user.read()
            data_time = coordinatorData[1][0]          #Kellaaeg, millal server saatis andmed

            if data_time !=  data_time_last:           #Uuenda ainult siis, kui on uued andmed
                data_time_last = data_time
                data_count = coordinatorData[1][1]     # Andmepaki number

                balls = coordinatorData[1][2]          # Pallide koordinaadid jms. [[x, y, r, inside], [x2, y2, r2, inside2], ...]
                for ball in balls:
                    ball[1] = -ball[1]                  # -y

                if len(coordinatorData[1][3]) >= ring_counter:      # if rings disappear, use previous data
                    rings = coordinatorData[1][3]                   # Rongaste koordinaadid jms. [[x, y, r, count], [x2, y2, r2, count2], ...]
                    ring_counter = len(coordinatorData[1][3])
                for ring in rings:
                    if ring[1]>0:
                        ring[1] = -ring[1]                 # -y

                robots = coordinatorData[1][4]
                for robot in robots:
                    if robot[1][1]>0:
                        robot[1][1] = -robot[1][1]                 # -y


                #print ("data_time", str(data_time))

            # Robotite koordinaadid jms. [[num, [x, y], angle], [num2, [x2, y2], angle2], ...]

            # print(data_time, data_count, balls, rings, robots)

            # print("Delay serveri ja kliendi vahel: ", round(time.time() - data_time, 3), " s")   # Kui delay pidevalt kasvab, leia yhe tsykli labimise aeg ja anna teada.

                my_robot_data = robot_definer(robots, my_robot_number)
            

                cycle_counter += 1
                if cycle_counter > 30:   #Check if robot has moved
                    stick_test(robot_position_history)
                    cycle_counter = 0
                
                print("data %s" % my_robot_data)
                
                ball_v = ball_vector(balls, my_robot_data)
                
                print("ball data %s" % ball_v)
               
                if len(ball_v)>0:
                    best_ball = ball_chooser(ball_v, my_robot_data, min_r)
                    print("best ball: "+ str(best_ball))
                else:
                    bw.stop()
                
                print ("my balls: " + str(myballs(ball_v)))
                
                if myballs(ball_v) > my_ball_counter:
                    my_ball_counter = myballs(ball_v)
            
                if my_ball_counter >= 3 or my_ball_counter >= len(ball_v) :#  platsil pole rohkem vabu palle
                    print ("Got all the balls, go to ring")
                   
                    goal_type = "RING"
                    goal = ring_chooser(rings, my_robot_data, 5)
                    #print (goal)
                else:
                    goal = best_ball
                    
                    goal_type = "BALL"
                    if best_ball[0] < 200 and abs(best_ball[1]) < 30: # if visible from on-board camera: dist, angle
   #                     watcher_final.main_watcher()
                        print("Good time to activate The Watcher()")
                
                robot_clear_path_check(my_robot_data, rings, robots, goal_type)
                
                
                if goal == None:
                    break
                else:
                    go_to(goal, goal_type)
                
                
                print()

            #time.sleep (0.2)
            #
        except :#IndexError, TypeError, ReadingError) as error:
            #print(error)
            #print("Andmeid veel ei ole")
            pass


goal = None
global goal_type
goal_type = "BALL"
my_ball_counter = 0
data_time_last = 0
ring_counter = 0
coordinator()
