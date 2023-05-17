import cv2
import numpy as np
from numpy import savetxt
from robomaster import robot
from robomaster import camera
import time
from ultralytics import YOLO

def findcenter(contours):
    center= []
    leftTx = np.min(contours[0],axis=0)
    leftTy = np.min(contours[0],axis=0)

    rightTx = np.max(contours[0],axis=0)
    Bottomy = np.max(contours[0],axis=0)
    # print(Bottomy[0][1])
    center.append((leftTx[0][0]+rightTx[0][0])/2)
    center.append((leftTy[0][1]+Bottomy[0][1])/2)

    return center

def drawCross(image,center):
    height = image.shape[0]
    width = image.shape[1]

    heightB = round(center[1])
    widthB = round(center[0])
    # Drawing the lines'
    # print(heightB)
    cv2.line(image, (0, heightB), (width, heightB), (0, 0, 255), 2)
    cv2.line(image, (widthB, 0), (widthB, height), (0, 0, 255), 2)


model = YOLO('./runs/detect/yolov8n_custom12/weights/best.pt')  # load an official detection model
legoList = [0.0,1.0,4.0,5.0,6.0,7.0,11.0]
def call_roboflow_api(img):
    resp = model.predict(img,show=True)
    return resp

def trackPlatform(ep_chassis,ep_gripper,image,center):
    Tru_height = image.shape[0]/2
    Tru_width = image.shape[1]/2
    legodist = 383
    if center[0] < Tru_width-20:
            print("right rot")
            ep_chassis.drive_speed(x=0, y=0, z=-3, timeout=None)
    elif center[0] > Tru_width+20:
            print("left rot")
            ep_chassis.drive_speed(x=-0, y=0, z=3, timeout=None)   
    else:
        if center[1] < legodist:
            ep_chassis.drive_speed(x=0.1, y=0, z=0, timeout=None)
        else:
            time.sleep(0.2)
            ep_chassis.drive_speed(x=0, y=0, z=0, timeout=None)
            ep_gripper.open(power=50)
            time.sleep(1)
            ep_gripper.pause()
            exit(1)

def trackLego(ep_chassis,ep_gripper,image,center):
    Tru_height = image.shape[0]/2
    Tru_width = image.shape[1]/2
    legodist = 247
    if center[0] < Tru_width-20:
            print("right rot")
            ep_chassis.drive_speed(x=0, y=0, z=-3, timeout=None)
    elif center[0] > Tru_width+20:
            print("left rot")
            ep_chassis.drive_speed(x=-0, y=0, z=3, timeout=None)   
    else:
        if center[1] < legodist:
            ep_chassis.drive_speed(x=0.1, y=0, z=0, timeout=None)
        else:
            ep_chassis.drive_speed(x=0, y=0, z=0, timeout=None)
            ep_gripper.close(power=50)
            time.sleep(1)
            ep_gripper.pause()
            exit(1)
            return

def findPlatform(img):
    results = call_roboflow_api(img)
    results = results[0]
    print(results)
    boxes = results.boxes
    # print(box.xywh)][0
    tempx = [0,0]
    # print("# of items detected: {}".format(len())) //red goal 6 and 7 yellow goal 11
    if(len(boxes)!= 0):
        box = boxes[0]  # returns one box
        boxMem = box.xywh.cpu()
        for index, detection in enumerate(boxes):
            print(detection.cls[0].item())
            if(detection.cls[0].item() == 6.0 or detection.cls[0].item() == 7.0):
                position = [round(detection.xywh[0][0].item()), round(detection.xywh[0][1].item())]
                if(position[1] > tempx[0]):                
                    tempx = [position[1],index]
        if(tempx[0] == 0 & tempx[1] == 0):
            center = [0,0]
        else :
                center = [boxes.xywh[tempx[1]][0].item(), boxes.xywh[tempx[1]][1].item()]
    else: 
         center =[0,0]
    return center




def findLego(img):
    results = call_roboflow_api(img)
    results = results[0]
    boxes = results.boxes
    # print(box.xywh)
    tempx = [0,0]
    # print("# of items detected: {}".format(len()))
    if(len(boxes)!= 0):
        box = boxes[0]  # returns one box
        boxMem = box.xywh.cpu()
        for index, detection in enumerate(boxes):
            # print(detection.cls[0].item())
            if(detection.cls[0].item() != 1 & index < len(boxes)):
                position = [round(detection.xywh[0][0].item()), round(detection.xywh[0][1].item())]
                if(position[1] > tempx[0]):                
                    tempx = [position[1],index]
        center = [boxes.xywh[tempx[1]][0].item(), boxes.xywh[tempx[1]][1].item()]
    else: 
         center =[0,0]
    return center


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    ep_gripper = ep_robot.gripper

    # img = cv2.imread('Lego.png')
    # img,center = findLego(img)
    # drawCross(img,center)
    # cv2.imshow("img", img)
    # cv2.waitKey(0)
    ep_gripper.open(power=50)
    # time.sleep(1)
    # ep_gripper.pause()
    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=10)
            # cv2.imgwrtie
            center = findPlatform(img)
            print(center)
            if not center:
                print("object not detected")
            else:
                drawCross(img,center)
                # trackLego(ep_chassis,ep_gripper,img,center)
                # trackPlatform(ep_chassis,ep_gripper,img,center)
            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)