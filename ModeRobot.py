from roboflow import Roboflow
import cv2
import numpy as np
import time
from ultralytics import YOLO
from robomaster import robot
from robomaster import camera


# ROBOFLOW_API_KEY = "oIcDJovVAvTV8Yg3AT20"
# ROBOFLOW_MODEL = "cmsc477_project3/1" # eg xx-xxxx--#
# ROBOFLOW_SIZE = 360

# rf = Roboflow(api_key="oIcDJovVAvTV8Yg3AT20")
# project = rf.workspace().project("cmsc477_project3")
# model = project.version(3).model



model = YOLO('./runs/detect/yolov8n_custom12/weights/best.pt')  # load an official detection model
def call_roboflow_api(img):
    resp = model.predict(img,show=True)
    return resp

# Use vid instead of ep_camera to use your laptop's webcam
# vid = cv2.VideoCapture(0)

# ep_robot = robot.Robot()
# ep_robot.initialize(conn_type="ap")
# ep_camera = ep_robot.camera
# ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis


    tag_size=0.16 # tag size in meters
    time.sleep(0.5)
    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            results = call_roboflow_api(img)
            results = results[0]
            # cv2.imshow("img", img)
            # # cv2.waitKey(10)
            # preds = response['predictions']
            # for pred in preds:
            #     print(pred['x'])
            #     print(pred['y'])
            print(results.boxes[0].xywh[0][0].item())
            print(results.boxes[0].cls)
            # boxes = results[0].boxes
            # box = boxes[0]  # returns one box
            # print(box.xywh)
            # boxMem = box.xywh.cpu()
            # print(boxMem[0][0].item())

            

            # print(response)


        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)