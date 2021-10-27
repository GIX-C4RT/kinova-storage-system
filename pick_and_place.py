from time import time

import cv2
import numpy as np
import imutils

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from arm_control import cartesian_action_movement_absolute, move_to_home_position, set_gripper_position, twist_command, cartesian_action_movement_relative

import utilities
from pid import PID

IP_ADDRESS = "192.168.1.10"
ARUCO_DICT = cv2.aruco.DICT_ARUCO_ORIGINAL

aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT)
aruco_params = cv2.aruco.DetectorParameters_create()

args = utilities.parseConnectionArguments()
# print(args.ip)



def get_marker_coordinates(frame, id):
    '''return coordinates (center_x, center_y, angle) of the specifed ArUco marker'''
    width = frame.shape[1]
    height = frame.shape[0]
    center_frame = (width // 2, height // 2)
    
    # detect ArUco markers
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
    if ids is not None:
        ids = ids.flatten()
        if id in ids:
            # get corner values
            idx = np.where(ids==id)[0][0]
            c4 = corners[idx]
            # print(c4)
            # find center point, angle
            (cX, cY), _, angle = cv2.minAreaRect(c4)
            # print(cX, cY, angle)



            # display code
            # TODO: disable for performance
            # display bounding box
            cv2.drawContours(frame, [c4.astype(int)], 0, (0, 0, 255), 5)
            # center of box
            cv2.circle(frame, (int(cX), int(cY)), 7, (255, 255, 255), -1)
            # label box
            cv2.putText(frame, str(id), (int(cX) - 20, int(cY) - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            # cv2.imshow("id", frame)
            # center of screen
            cv2.circle(frame, center_frame, 7, (0, 255, 0), -1)
            # label box
            cv2.putText(frame, "center_frame", center_frame,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # normalize the coordinates
            # print(frame.shape)
            cX = (cX / width) * 2 - 1
            cY = (cY / height) * 2 - 1
            # angle = np.deg2rad(angle)

            return np.array([cX, cY, angle])
    return None


def go_to_marker(connection, id):
    '''move the hand above the marker with the specified ID'''
    (cap, base, base_cyclic) = connection

    x_pid = PID(0.0, 0, 0)
    x_pid.send(None)

    y_pid = PID(0.0, 0, 0)
    y_pid.send(None)

    theta_pid = PID(0.2, 0, 0)
    theta_pid.send(None)

    x_tol = 0.003 # x translation tolerance
    y_tol = 0.01 # y translation tolerance
    theta_tol = 5 # angular tolerance (deg)

    y_offset = 0.2

    goal_reached = False

    while not goal_reached:
        # capture a frame
        ret, frame = cap.read()
        frame = imutils.resize(frame, width=300)
        # detect ArUco markers
        coords = get_marker_coordinates(frame, id)
        
        # check if the specified marker is detected
        if coords is not None:
            x, y, theta = coords
            print("got coords:", coords)
            # move gripper above box
            y = y + y_offset
            print(coords)
            if (abs(x) <= x_tol) and (abs(y) <= y_tol) and (np.abs(theta) <= theta_tol):
                print("goal reached")
                # stop the arm from moving
                base.Stop()
                break
            # calculate appropriate velocities
            t = time()
            vel_x = x_pid.send([t, x, 0])
            vel_y = y_pid.send([t, y, 0])
            # # turn towards the closest side
            # if theta > 45:
            #     theta -= 90
            vel_theta = theta_pid.send([t, theta, 0])
            # ang_vel_z = 0
            print(vel_x, vel_y, vel_theta)
            twist_command(base, (vel_x, vel_y, 0, 0, 0, vel_theta))
        # display frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break
    # close any open OpenCV windows
    cv2.destroyAllWindows()


def pick(connection, id):
    '''pick up the box with the ArUco marker with the specified ID'''
    (cap, base, base_cyclic) = connection

    go_to_marker(connection, id)
    
    # lower arm
    feedback = base_cyclic.RefreshFeedback()
    dz = -feedback.base.tool_pose_z + 0.05 # 10cm above 0 z
    cartesian_action_movement_relative(base, base_cyclic, (0,0,dz,0,0,0))

    # close gripper
    set_gripper_position(base, 0.15)

    # raise the arm
    cartesian_action_movement_relative(base, base_cyclic, (0,0,-dz,0,0,0))

    # close any open OpenCV windows
    cv2.destroyAllWindows()

    # stop the arm from moving
    base.Stop()


def place(connection, id):
    '''places a box on the ArUco marker with the specified ID'''
    pass


def setup(connection):
    (cap, base, base_cyclic) = connection
    # open the gripper
    set_gripper_position(base, 0.0)

    # go to good position
    feedback = base_cyclic.RefreshFeedback()
    # print(feedback.base)
    starting_pose = (0.4, 0, 0.5, 180, 0, 90)
    cartesian_action_movement_absolute(base, base_cyclic, starting_pose)


if __name__ == "__main__":
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        # open video stream
        cap = cv2.VideoCapture("rtsp://" + args.ip + "/color")

        connection = (cap, base, base_cyclic)

        # do stuff
        setup(connection)

        # pick up box 0
        pick(connection, 0)

        setup(connection)


        # clean up:
        cap.release()