import numpy as np
import cv2
import time
import math
from robomaster import robot
from robomaster import camera



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

    # Robot 1:
    while (True):
        # Set arm to start position (low enough to grab)
        ep_arm.moveto(200, 0).wait_for_completed()

        # Open up gripper
        ep_gripper.open(power=50)

        # Find and move to lego pile
        print("Calibrating and Moving towards the lego tower")
        #calibrate(ep_camera, 1)

        # Grab lego
        print("Grabbing Lego")
        ep_gripper.close(power=70)
        time.sleep(4)
        ep_gripper.pause()

        # Raise arm
        ep_arm.moveto(200, 20).wait_for_completed()

        # Escape Metal Bar Jail

        # Find and move to blue wall
        print("Calibrating and Moving towards blue wall")
        #calibrate(ep_camera, 2)
        # Raise arm
        ep_arm.moveto(200, 80).wait_for_completed()
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