from pupil_apriltags import Detector
import cv2
import numpy as np
import time
from robomaster import robot
from robomaster import camera

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

def find_pose_from_tag(K, detection):
    m_half_size = tag_size / 2

    marker_center = np.array((0, 0, 0))
    marker_points = []
    marker_points.append(marker_center + (-m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, -m_half_size, 0))
    marker_points.append(marker_center + (-m_half_size, -m_half_size, 0))
    _marker_points = np.array(marker_points)

    object_points = _marker_points
    image_points = detection.corners

    pnp_ret = cv2.solvePnP(object_points, image_points, K, distCoeffs=None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if pnp_ret[0] == False:
        raise Exception('Error solving PnP')

    r = pnp_ret[1]
    p = pnp_ret[2]

    return p.reshape((3,)), r.reshape((3,))


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis


    tag_size=0.16 # tag size in meters

    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray.astype(np.uint8)

            K = np.array([[184.752, 0, 320], [0, 184.752, 180], [0, 0, 1]])

            results = at_detector.detect(gray, estimate_tag_pose=False)
            if (results == []):
                print("backing up")
                ep_chassis.drive_speed(x=-0.1, y=0, z=0, timeout=None)
            else:
                for res in results:
                    tagid = res.tag_id
                    goal_tag = 13
                    print(res.tag_id)
                    if tagid == goal_tag:
                        pose = find_pose_from_tag(K, res)
                        print(pose)
                        rot, jaco = cv2.Rodrigues(pose[1], pose[1])
                        print(rot)

                        pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                        img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                        cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                        print(pose)
                                        #use pose[0][3]
                        # Code to move or rotate based on conditions
                        # print("tag:{}".format(tagid))
                        # print("x:{} y:{} Z{}".format(rot[0][0],rot[0][1],rot[0][2]))
                        if tagid == goal_tag:
                            if pose[0][2] > 0.26:
                                ep_chassis.drive_speed(x=0.1, y=0, z=0, timeout=10)
                            elif pose[0][0] < -0.1 and pose[0][0] > -0.3: #righ
                                ep_chassis.drive_speed(x=0, y=-0.1, z=0, timeout=10)
                                print("right mot")
                            elif pose[0][0] > 0.1 and pose[0][0] < 0.3: #left
                                ep_chassis.drive_speed(x=0, y=0.1, z=0, timeout=10)
                                print("left  mot")
                            else: 
                                print("zeroing trans")
                                ep_chassis.drive_speed(x=0, y=0, z=0, timeout=10)
                                if rot[0][1] > 0.3 and rot[0][1] < 0.1:
                                    ep_chassis.drive_speed(x=0.1, y=0, z=0, timeout=10)
                                elif rot[0][1] > -0.3 and rot[0][1] < -0.1:
                                    ep_chassis.drive_speed(x=-0.1, y=0, z=0, timeout=10)   
                                else:
                                        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=10)

            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print('Exiting')
            exit(1)

