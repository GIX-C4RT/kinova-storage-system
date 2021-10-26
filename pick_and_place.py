import utilities
import cv2
import numpy as np
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from arm_control import cartesian_action_movement_absolute, move_to_home_position, twist_command, cartesian_action_movement_relative

import imutils

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

    

def pick(connection, id):
    '''pick up the box with the ArUco marker with the specified ID'''
    (cap, base, base_cyclic) = connection

    trans_tol = 0.05 # translational tolerance (undefined units)
    ang_tol = 5 # angular tolerance (deg)
    x_gain = .01 # x proportional gain
    y_gain = 0.01 # y propotional gain
    ang_gain = .1 # angular velocity proportional gain

    goal_reached = False

    while not goal_reached:
        # capture a frame
        ret, frame = cap.read()
        frame = imutils.resize(frame, width=300)
        # detect ArUco markers
        coords = get_marker_coordinates(frame, id)
        
        # move gripper above box
        if coords is not None:
            x, y, angle = coords
            print(coords)
            if (np.linalg.norm([x, y]) <= trans_tol) and (np.abs(angle) <= ang_tol):
                print("goal reached")
                # stop the arm from moving
                base.Stop()
                break
            # calculate appropriate velocities
            vel_x = -x * x_gain
            vel_y = -y * y_gain
            if angle > 45:
                angle -= 90
            # angle = angle + 45
            print(angle)
            ang_vel_z = angle * ang_gain
            # ang_vel_z = 0
            print(vel_x, vel_y, ang_vel_z)
            twist_command(base, (vel_x, vel_y, 0, 0, 0, ang_vel_z))
        # display frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break
    
    # lower arm
    feedback = base_cyclic.RefreshFeedback()
    dz = -feedback.base.tool_pose_z + 0.05 # 10cm above 0 z
    cartesian_action_movement_relative(base, base_cyclic, (0,0,dz,0,0,0))

    

    # close any open OpenCV windows
    cv2.destroyAllWindows()

    # stop the arm from moving
    base.Stop()

def place(connection, id):
    '''places a box on the ArUco marker with the specified ID'''
    pass

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
        # go to good position
        feedback = base_cyclic.RefreshFeedback()
        print(feedback.base)
        starting_pose = (0.4, 0, 0.5, 180, 0, 90)
        cartesian_action_movement_absolute(base, base_cyclic, starting_pose)
        # pick up box 0
        pick(connection, 0)


        # clean up:
        cap.release()