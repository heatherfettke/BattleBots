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


x, y, z = [0, 0, 0]
x_offset, y_offset = [15*0.26,5*0.26]

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

def move_between_coords(gx, gy, gz):
    global x
    global y
    global z
    unit_size = 0.26
    gx = gx*0.26
    gy = gy*0.26
    print(f"moving to {gx}, {gy}")
    print(f"{x}, {y}, {z}")
    y_diff = y-gy
    if y_diff > unit_size:
        print("moving left")
        ep_chassis.move(x=0, y=0, z=90-z).wait_for_completed()
    elif y_diff < -unit_size:
        print("moving right")
        ep_chassis.move(x=0, y=0, z=-90-z).wait_for_completed()
    ep_chassis.move(x=abs(y_diff), y=0, z=0).wait_for_completed()
    print(f"{x}, {y}, {z}")
    x_diff = x-gx
    if x_diff > unit_size:
        print("moving down")
        ep_chassis.move(x=0, y=0, z=0-z).wait_for_completed()
    elif x_diff < -unit_size:
        print("moving up")
        ep_chassis.move(x=0, y=0, z=180-z).wait_for_completed()
    ep_chassis.move(x=abs(x_diff), y=0, z=0).wait_for_completed()
    print(f"{x}, {y}, {z}")
    if gz is not None:
        ep_chassis.move(x=0.5, y=0, z=z-gz).wait_for_completed()
        print(f"{x}, {y}, {z}")
    ep_chassis.move(x=0, y=0, z=0).wait_for_completed()
    return True

def dodge(up_or_down, left_side):
    # up = true
    # left_side = true
    ep_chassis.move(x=0, y=-0.1 if up_or_down else 0.1, z=0).wait_for_completed()
    ep_chassis.move(x=-0.3 if left_side else 0.3).wait_for_completed()

def path_and_move(goal):
    global x
    global y
    print(f"x: {x}, y: {y}")
    path = djikstra((round(x/0.26),round(y/0.26)),goal)
    print(path)
    while True:
        reached_goal = False
        for spot in path:
            print(f"Moving to point: {spot}")
            if not move_between_coords(spot[0], spot[1], None):
                break
            elif spot == goal:
                reached_goal = True
        if reached_goal:
            print("Reached the end goal")
            break
        else:
            path = djikstra((round(x/0.26),round(y/0.26)),goal)
            print(f"New path: {path}")

            
        

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
                # Finding the other robot
                angle = None
                hor_dist = track_op_Lego(img)
                dep_dist = None
            elif task == 4:
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
                        ep_chassis.drive_speed(x=0, y=0, z=-5, timeout=None)
                elif angle < -0.045:
                        print("Rotating Right")
                        ep_chassis.drive_speed(x=-0, y=0, z=5, timeout=None)   
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

    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_info_handler)
    time.sleep(1)

    csvfile = csv.reader(open('arena.csv', mode='r'))
    for row in csvfile:
        maze.append(row)

    # Robot 1:
    while (True):
        # Set arm to start position (low enough to grab)
        ep_arm.moveto(200, 0).wait_for_completed()

        # Open up gripper
        ep_gripper.open(power=50)

        # Find and move to lego pile
        #path_and_move((9,4))
        ep_chassis.move(x=1, y=0, z=0).wait_for_completed()
        print("One")
        ep_chassis.move(x=0, y=0, z=90).wait_for_completed()
        print("Two")
        ep_chassis.move(x=1, y=0, z=0).wait_for_completed()
        print("Three")
        print("Calibrating and Moving towards the lego pile")
        #calibrate(ep_camera, 1)

        # Grab lego
        print("Grabbing Lego")
        ep_gripper.close(power=50)
        time.sleep(4)
        ep_gripper.pause()

        """
        # Raise arm
        ep_arm.moveto(200, 20).wait_for_completed()

        # Escape Metal Bar Jail

        # Find and move to blue wall
        print("Calibrating and Moving towards blue wall")
        #calibrate(ep_camera, 2)
        # Raise arm
        #ep_arm.moveto(200, 80).wait_for_completed()
        # Move closer
        print("Moving closer to blue wall")
        ep_chassis.move(x=0.366, y=0, z=0, xy_speed=0.3).wait_for_completed()

        # Release lego
        print("Releasing Brick")
        ep_gripper.open(power=50)

        # Back up
        ep_chassis.move(x=-0.2, y=0, z=0, xy_speed=0.3).wait_for_completed()

        # Rotate 180 around


        # Back to square one
        """
    
    ep_chassis.unsub_position()
    ep_chassis.unsub_attitude()

    ep_robot.close()