import numpy as np
import cv2
import time
import math
from robomaster import robot
from robomaster import camera
import csv
from heapdict import heapdict
from matplotlib import pyplot as plt

maze = []
maze_backup = []
maze_display = None
maze_locations = []
to_explore = heapdict() 
explored = []

class Location(object):
    x = 0
    y = 0
    val = '0'
    parent = None
    dist = float('inf')

    def __init__(self, x, y, parent):
        self.x = x
        self.y = y
        self.val = maze[x][y]
        self.parent = parent
        self.dist = float('inf')

    def __eq__(self, other_loc):
        return self.x == other_loc.x and self.y == other_loc.y

    def __str__(self):
        return f"[{self.x}, {self.y}, {self.val}]"

def findLocation(x, y):
    for location in maze_locations:
        if location.x == x and location.y == y:
            return location
    return None

def explore(loc, goal):
    if loc in explored:
        return None
    if loc.x == goal[0] and loc.y == goal[1]:
        return loc

    #maze_display[loc.x][loc.y] = 5

    explored.append(loc)
    for i in range(2):
        new_dist = loc.dist + 1
        if 0 <= loc.x < len(maze) and 0 <= loc.y + 1 - 2 * i < len(maze[loc.x]):
            child_loc = findLocation(loc.x, loc.y + 1 - 2 * i)
            if child_loc.val != '1' and new_dist < child_loc.dist:
                child_loc.dist = new_dist
                child_loc.parent = loc
                if f"{child_loc.x},{child_loc.y}" in to_explore:
                    to_explore[f"{child_loc.x},{child_loc.y}"] = new_dist
        if 0 <= loc.x + 1 - 2 * i < len(maze) and 0 <= loc.y < len(maze[loc.x + 1 - 2 * i]):
            child_loc = findLocation(loc.x + 1 - 2 * i, loc.y)
            if child_loc.val != '1' and new_dist < child_loc.dist:
                child_loc.dist = new_dist
                child_loc.parent = loc
                if f"{child_loc.x},{child_loc.y}" in to_explore:
                    to_explore[f"{child_loc.x},{child_loc.y}"] = new_dist

    return None

def retrace(loc):
    path = []
    while loc is not None:
        path.append([loc.x, loc.y])
        loc = loc.parent
    path.reverse()
    return path

def djikstra(start, goal):
    start = [start[1],start[0]]
    goal = [goal[1],goal[0]]
    global maze_display
    maze_display = None
    global maze_locations
    maze_locations = []
    global to_explore
    to_explore = heapdict() 
    global explored
    explored = []
    for i in range(len(maze)):
        for j in range(len(maze[i])):
            to_explore[f"{i},{j}"] = float('inf')
            this_loc = Location(i, j, None)
            if start[0] == i and start[1] == j:
                this_loc.dist = 0
            maze_locations.append(this_loc)
    to_explore[f"{start[0]},{start[1]}"] = 0
    startLocation = to_explore.peekitem()
    print(f"Starting at {startLocation[0]}")

    """
    # Display map, also uncomment line in explore that says maze_display = ...
    maze_display = np.array(maze, dtype=np.uint8)
    plt.imshow(maze_display, interpolation='none')
    # plt.savefig('frames/map0.png')
    plt.show()
    """

    result = None
    #counter = 0
    while not to_explore.peekitem() is None:
        next_up = to_explore.popitem()[0]
        next_up = next_up.split(',')
        locale = findLocation(int(next_up[0]), int(next_up[1]))
        # print(f"{locale.x}, {locale.y}")
        result = explore(locale, goal)
        if result is not None:
            break

        #counter = counter + 1
        #if counter % 250 == 0:
            #plt.imshow(maze_display, interpolation='none')
            # plt.savefig(f'frames/map{counter}.png')
            # plt.show()

    final_path = retrace(result)
    #print(final_path)

    """
    #Display final path
    for spot in final_path:
        maze_display[spot[0]][spot[1]] = 4

    plt.imshow(maze_display, interpolation='none')
    # plt.savefig(f'frames/map{counter}.png')
    plt.show()
    """

    for i in range(len(final_path)):
        final_path[i] = [final_path[i][1], final_path[i][0]]

    return final_path[1:]


x, y, z = [15, 8, 0]
x_offset, y_offset = [15*0.26,8*0.26]

def sub_position_handler(position_info):
    global x
    global y
    global x_offset
    global y_offset
    x, y, z = position_info
    #print(f"{x} {y}")
    x = x + x_offset
    y = y + y_offset
    print(f"{x} {y}")
    #print("chassis position: x:{0}, y:{1}, z:{2}".format(x, y, z))

def sub_attitude_info_handler(attitude_info):
    global z
    z, pitch, roll = attitude_info
    #print("chassis attitude: yaw:{0}, pitch:{1}, roll:{2} ".format(z, pitch, roll))

def rotate(amt):
    if amt == 0:
        return
    #ep_chassis.drive_speed(x=0, y=0, z=-amt/2, timeout=None)
    #time.sleep(2.015)
    #ep_chassis.drive_speed(x=0, y=0, z=0, timeout=None)
    #time.sleep(0.1)
    ep_chassis.move(x=0, y=0, z=amt).wait_for_completed()


def isBlock():
    if True:
        return False
    else:
        return True

def isRobot():
    if True:
        return False
    else:
        return True

def move_between_coords(gx, gy):
    global x
    global y
    global z

    y_diff = gy - y
    x_diff = gx - x

    unit_size = 0.272
    angle_diff = 0
    angle_correction = 0.75

    global last_up_down

    print("Moving on Y")
    if y_diff < -0.5:
        angle_diff = 0-z
        if angle_diff == 270:
            angle_diff = -90
        elif angle_diff == -270:
            angle_diff = 90
        print(f"up {angle_diff}")
        #ep_chassis.move(x=0, y=0, z=0 if angle_diff == 0 else (angle_diff + angle_correction if angle_diff > 0 else angle_diff - angle_correction)).wait_for_completed()
        rotate(0 if angle_diff == 0 else (angle_diff + angle_correction if angle_diff > 0 else angle_diff - angle_correction))
        time.sleep(0.2)
        z = 0
        last_up_down = True
        while (isRobot()):
            print("Enemy Robot Spotted")
            time.sleep(1)
        if(dist_sense < 450 or isBlock()):
            maze[gy][gx] = '1'
            print(maze)
            #dodge(True, False)
            return False
        ep_chassis.drive_speed(x=unit_size*2, y=0, z=0, timeout=None)
        time.sleep(0.5)
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=None)
        time.sleep(0.5)
    elif y_diff > 0.5:
        angle_diff = 180-z
        if angle_diff == 270:
            angle_diff = -90
        elif angle_diff == -270:
            angle_diff = 90
        print(f"down {angle_diff}")
        #ep_chassis.move(x=0, y=0, z=0 if angle_diff == 0 else (angle_diff + angle_correction if angle_diff > 0 else angle_diff - angle_correction)).wait_for_completed()
        rotate(0 if angle_diff == 0 else (angle_diff + angle_correction if angle_diff > 0 else angle_diff - angle_correction))
        time.sleep(0.2)
        z = 180
        last_up_down = False
        while (isRobot()):
            print("Enemy Robot Spotted")
            time.sleep(1)
        if(dist_sense < 450 or isBlock()):
            maze[gy][gx] = '1'
            print(maze)
            #dodge(False, False)
            return False
        ep_chassis.drive_speed(x=unit_size*2, y=0, z=0, timeout=None)
        time.sleep(0.5)
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=None)
        time.sleep(0.5)
    y = gy

    ep_chassis.move(x=0, y=0, z=0).wait_for_completed()

    print("Moving on X")
    if x_diff > 0.5:
        angle_diff = 270-z
        if angle_diff == 270:
            angle_diff = -90
        elif angle_diff == -270:
            angle_diff = 90
        print(f"right {angle_diff}")
        #ep_chassis.move(x=0, y=0, z=0 if angle_diff == 0 else (angle_diff + angle_correction if angle_diff > 0 else angle_diff - angle_correction)).wait_for_completed()
        rotate(0 if angle_diff == 0 else (angle_diff + angle_correction if angle_diff > 0 else angle_diff - angle_correction))
        time.sleep(0.2)
        z = 270
        while (isRobot()):
            print("Enemy Robot Spotted")
            time.sleep(1)
        if(dist_sense < 450 or isBlock()):
            maze[gy][gx] = '1'
            print(maze)
            #dodge(False, False)
            return False
        #ep_chassis.move(x=abs(x_diff)*unit_size, y=0, z=0).wait_for_completed()
        ep_chassis.drive_speed(x=unit_size*2, y=0, z=0, timeout=None)
        time.sleep(0.5)
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=None)
        time.sleep(0.5)
    elif x_diff < -0.5:
        angle_diff = 90-z
        if angle_diff == 270:
            angle_diff = -90
        elif angle_diff == -270:
            angle_diff = 90
        print(f"left {angle_diff}")
        #ep_chassis.move(x=0, y=0, z=0 if angle_diff == 0 else (angle_diff + angle_correction if angle_diff > 0 else angle_diff - angle_correction)).wait_for_completed()
        rotate(0 if angle_diff == 0 else (angle_diff + angle_correction if angle_diff > 0 else angle_diff - angle_correction))
        time.sleep(0.2)
        z = 90
        while (isRobot()):
            print("Enemy Robot Spotted")
            time.sleep(1)
        if(dist_sense < 450 or isBlock()):
            maze[gy][gx] = '1'
            print(maze)
            #dodge(False, False)
            return False
        #ep_chassis.move(x=abs(x_diff)*unit_size, y=0, z=0).wait_for_completed()
        ep_chassis.drive_speed(x=unit_size*2, y=0, z=0, timeout=None)
        time.sleep(0.5)
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=None)
        time.sleep(0.5)
    x = gx

    ep_chassis.move(x=0, y=0, z=0).wait_for_completed()

    return True

def path_and_move(goal):
    global x
    global y
    print(f"x: {x}, y: {y}, planning to {goal}")
    path = djikstra((x,y),goal)
    print(path)
    while True:
        reached_goal = False
        for spot in path:
            print(f"Moving to point: {spot}")
            if not move_between_coords(spot[0], spot[1]):
                break
            elif spot == goal:
                reached_goal = True
        if reached_goal:
            print("Reached the end goal")
            break
        else:
            path = djikstra((x,y),goal)
            print(f"New path: {path}")
            if path == []:
                maze = maze_backup
                print("RESET MAZE!")
                break

dist_sense = 0

def sub_data_handler(sub_info):
    global dist_sense
    dist_sense = sub_info[0]
    #if dist_sense < 450:
    #    print("BLOCK DETECTED!!!")
          
def findWall(img):
    # Reading the image
    #img = cv2.imread('Project2\Wall (7).jpg')
    # Showing the output
    # cv2.imshow("Block", img)

    # convert to hsv colorspace
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # lower bound and upper bound for Blue color
    lower_bound = np.array([100, 75, 75])
    upper_bound = np.array([115, 255, 255])

    # find the colors within the boundaries
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    #define kernel size
    kernel = np.ones((7,7),np.uint8)

    # Remove unnecessary noise from mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.erode(mask,kernel,iterations = 1)

    # Segment only the detected region
    segmented_img = cv2.bitwise_and(img, img, mask=mask)

    # Canny Edge Detection
    output = cv2.Canny(segmented_img, 50, 200, None, 3)
    lines = cv2.HoughLines(output, 1, np.pi / 180, 100, None, 0, 0)
    #houghout = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)

    avgline = None
    avg_y = None
    if lines is not None:
        avgline = [lines[0][0][0], lines[0][0][1]]
        rho = avgline[0]
        theta = avgline[1]
        b = math.sin(theta)
        y0 = b * rho
        avg_y = y0
        for line in lines:
            rho = line[0][0]
            theta = line[0][1]
            b = math.sin(theta)
            y0 = b * rho
            avgline = [avgline[0] + rho, avgline[1] + theta]
            avg_y = avg_y + y0
        avgline = [avgline[0]/(len(lines)+1), avgline[1]/(len(lines)+1)]
        avg_y = avg_y / (len(lines)+1)
    else:
        return [0, 0]
    # Draw the lines
    #if avgline is not None:
    #    rho = avgline[0]
    #    theta = avgline[1]
    #    a = math.cos(theta)
    #    b = math.sin(theta)
    #    x0 = a * rho
    #    y0 = b * rho
    #    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
    #    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
    #    cv2.line(houghout, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)
    #output = houghout

    # Showing the output
    #scale_percent = 30 # percent of original size
    #width = int(output.shape[1] * scale_percent / 100)
    #height = int(output.shape[0] * scale_percent / 100)
    #dim = (width, height)
    
    # resize image
    #resized = cv2.resize(output, dim, interpolation = cv2.INTER_AREA)
    return [avgline[1]*180/np.pi - 90, avg_y] 

def get_wall_specs(img):
    angle, dist = findWall(img)
    if angle == 0 and dist == 0:
        angle = -5
    else:
        if angle > 2.2:
            angle = -2.0
        elif angle < -1.7:
            angle = 2.0
        else:
            angle = 0
        
        # Old = 300, 265
        if dist > 330:
            dist = -10
        elif dist < 280:
            dist = 10
        else:
            dist = 0
    print(f"Angle: {angle} Dist: {dist}")
    return angle, dist

def trackLego(img):
    return [0, 0]

def trackPlatform(img):
    return [0, 0]

def calibrate(ep_camera, task):
    print("Calibrating")
    time.sleep(2)
    while True:
        try:
            time.sleep(0.05)
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            
            angle = None # Angle relative to thing
            hor_dist = None # Horizontal Distance
            dep_dist = None # Depth Distance
            # Each value assigning function should return a number or None if the value is ok
            # The number technically doesn't have to be the actual value to move,
            # just a strong enough positive or negative
            if task == 1:
                # Finding the lego
                angle, dep_dist = trackLego(img)
                hor_dist = None
            elif task == 2:
                # Finding the blue wall
                angle, dep_dist = get_wall_specs(img)
                hor_dist = None
            elif task == 3:
                # Finding the platform
                angle, dep_dist = trackPlatform(img)
                hor_dist = None
            else:
                print("Invalid Task")

            # Basically, the function will check if any of the 3 values
            # are not None and then calibrate on them, with priority being on angle alignment
            # if the functions called to assign those value do so rightly, then
            # even as the robot moves forward it can stop to fix its rotation if needed
            if (angle is not None) and (angle > 0.045 or angle < -0.045):
                if angle > 0.045:
                        print("Rotating Left")
                        ep_chassis.drive_speed(x=0, y=0, z=-10, timeout=None)
                elif angle < -0.045:
                        print("Rotating Right")
                        ep_chassis.drive_speed(x=-0, y=0, z=10, timeout=None)   
            else:
                print("Angle is None or satisfied!")
                if hor_dist is not None and (hor_dist > 0.1 or hor_dist < -0.1):
                    if hor_dist < -0.1: #right
                        ep_chassis.drive_speed(x=0, y=-0.1, z=0, timeout=None)
                        print("Moving Right")
                    elif hor_dist > 0.1: #left
                        ep_chassis.drive_speed(x=0, y=0.1, z=0, timeout=None)
                        print("Moving Left")
                    else:                             
                        print("Halting Horizontal Motion")
                        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=None)
                else:
                    print("Horizontal Distance is None or satisfied!")
                    if dep_dist is not None and (dep_dist > 0.1 or dep_dist < -0.1):
                        if dep_dist < -0.1: #backward
                            ep_chassis.drive_speed(x=-0.1, y=0, z=0, timeout=None)
                            print("Moving Backward")
                        elif dep_dist > 0.1: #forward
                            ep_chassis.drive_speed(x=0.05, y=0, z=0, timeout=None)
                            print("Moving Forward")
                    else: 
                        print("Halting Motion, all values are None or Satisfied")
                        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=None)
                        ep_chassis.move(x=0, y=0, z=0).wait_for_completed()
                        return
            cv2.imshow("img", img)
            cv2.waitKey(10)
        except KeyboardInterrupt:
            print('Exiting')

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    ep_sensor = ep_robot.sensor
    ep_sensor.sub_distance(freq=5, callback=sub_data_handler)

    #ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    #ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    time.sleep(1)

    platform = (0,0)
    # ---------------------------------------------
    # Things to change for side and robot
    # ---------------------------------------------
    if False: # If on left side of arena
        x, y, z = [1, 8, 0]
        platform = (2,6)
    else:
        x, y, z = [15, 8, 0]
        platform = (14,6)

    #R1Left (Robot 1 on left side of arena (left to viewer behind R1))
    #R1Right (Robot 1 on right side of arena (right to viewer behind R1))
    #R2Left (Robot 2 on left side of arena (left to viewer behind R2))
    #R2Right (Robot 2 on right side of arena (right to viewer behind R2))

    csvfile = csv.reader(open('R2Right.csv', mode='r'))
    for row in csvfile:
        maze.append(row)

    maze_backup = maze

    race_count = 1

    # Robot 1:
    while (True):
        """
        # ---------------------------------------------
        # ROBOT 1
        # ---------------------------------------------

        # Set arm to start position (low enough to grab)
        ep_arm.moveto(160, -20).wait_for_completed()

        # Open up gripper
        ep_gripper.open(power=50)

        # Find and move to lego pile
        path_and_move((8,5))
        
        #print("Calibrating and Moving towards the lego pile")
        calibrate(ep_camera, 1)

        # Grab lego
        print("Grabbing Lego")
        ep_gripper.close(power=50)
        time.sleep(4)
        ep_gripper.pause()

        # Raise arm
        ep_arm.moveto(160, -10).wait_for_completed()

        # Escape Metal Bar Jail
        path_and_move((8,2))

        # Turn to face the blue wall
        angle_diff = 0-z
        if angle_diff == 270:
            angle_diff = -90
        elif angle_diff == -270:
            angle_diff = 90
        print(f"left {angle_diff}")
        rotate(angle_diff)
        z = 0
        time.sleep(0.2)

        # Find and move to blue wall
        print("Calibrating and Moving towards blue wall")
        calibrate(ep_camera, 2)
        # Raise arm
        ep_arm.moveto(200, 80).wait_for_completed()
        # Move closer
        print("Moving closer to blue wall")
        ep_chassis.move(x=0.2, y=0, z=0, xy_speed=0.3).wait_for_completed()

        # Release lego
        print("Releasing Brick")
        ep_gripper.open(power=50)
        time.sleep(2)

        # Back up
        ep_chassis.move(x=-0.2, y=0, z=0, xy_speed=0.3).wait_for_completed()
        # Lower arm
        ep_arm.moveto(200, 10).wait_for_completed()

        # Update rotation
        z = 0

        # Back to square one
        #path_and_move((15,8))
        #ep_gripper.close(power=50)
        #print(f"Finished {race_count} loop")
        #race_count += 1
        #time.sleep(3)

        """
        # ---------------------------------------------
        # ROBOT 2
        # ---------------------------------------------
        # Sleep in late cause Robot 1 is not gonna be as fast
        print("honk shoo... honk shoo.... mimimi.....")
        time.sleep(7)

        # Set arm to start position (low enough to grab)
        ep_arm.moveto(160, -20).wait_for_completed()

        # Open up gripper
        ep_gripper.open(power=50)

        # Find and move to blue wall
        path_and_move((8,2))
        
        #print("Calibrating and Moving towards the lego pile")
        calibrate(ep_camera, 1)

        # Grab lego
        print("Grabbing Lego")
        ep_gripper.close(power=50)
        time.sleep(4)
        ep_gripper.pause()

        # Raise arm
        ep_arm.moveto(160, -10).wait_for_completed()

        # Move to platform
        path_and_move(platform)

        # Calibrate on platform
        calibrate(ep_camera, 3)

        # Raise arm
        ep_arm.moveto(200, 80).wait_for_completed()
        # Move closer
        print("Moving closer to platform")
        ep_chassis.move(x=0.2, y=0, z=0, xy_speed=0.3).wait_for_completed()

        # Release lego
        print("Releasing Brick")
        ep_gripper.open(power=50)
        time.sleep(2)

        # Back up
        ep_chassis.move(x=-0.2, y=0, z=0, xy_speed=0.3).wait_for_completed()
        # Lower arm
        ep_arm.moveto(200, 10).wait_for_completed()

        #ep_gripper.close(power=50)
        #print(f"Finished {race_count} loop")
        #race_count += 1
        #time.sleep(3)
        #"""




    
    ep_chassis.unsub_position()
    ep_chassis.unsub_attitude()
    ep_sensor.unsub_distance()

    ep_robot.close()