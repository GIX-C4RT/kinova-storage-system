from time import time

import cv2
import numpy as np
import imutils
from numpy.lib.function_base import disp

from pid import PID
from arm import Arm

class StorageDevice:
    def __init__(self, ip_address="192.168.1.10", username="admin", password="admin"):
        '''Initialize the storage device.'''
        # connect to the arm
        self.arm = Arm(ip_address, username, password)

        # move the arm to the starting position
        self.arm.pose((0.4, 0, 0.5, 180, 0, 90))
        self.arm.grip(0)

    
    def disconnect(self):
        '''Disconnect storage device.'''
        self.arm.disconnect()

    def get(item_id):
        pass

    def store(item_id):
        pass

    def pick(self, item_id, display_frames=False):
        '''Pick up the box with the ArUco marker with the specified ID.'''

        self.go_to_marker(item_id, display_frames=display_frames)
        
        # lower arm
        pose = self.arm.get_pose()
        dz = -pose[2] + 0.05 # 5cm above 0 z
        self.arm.move_relative((0,0,dz,0,0,0))

        # close gripper
        self.arm.grip(0.15)

        # raise the arm
        self.arm.move_relative((0,0,-dz,0,0,0))

        # stop the arm from moving
        self.arm.stop()


    def place(self, marker_id, display_frames=False):
        '''Place the box on the ArUco marker with the specified marker_id.'''
        '''pick up the box with the ArUco marker with the specified ID'''

        self.go_to_marker(marker_id, aruco_dict=cv2.aruco.DICT_5X5_50,
            display_frames=display_frames)
        
        # lower arm
        pose = self.arm.get_pose()
        dz = -pose[2] + 0.06 # 6cm above 0 z
        self.arm.move_relative((0,0,dz,0,0,0))

        # open gripper
        self.arm.grip(0)

        # raise the arm
        self.arm.move_relative((0,0,-dz,0,0,0))

        # stop the arm from moving
        self.arm.stop()

    def go_to_marker(self, marker_id, aruco_dict=cv2.aruco.DICT_ARUCO_ORIGINAL, display_frames=False):
        '''move the hand above the marker with the specified ID'''
        y_offset = 0.05

        # PID controllers
        I = .01
        # x_pid = PID(0.2, 0.01, 0.02)
        x_pid = PID(0.2, 0.0, 0.0)
        x_pid.send(None)

        # y_pid = PID(0.1, 0.01, 0.01)
        y_pid = PID(0.1, 0.0, 0.0)
        y_pid.send(None)

        # theta_pid = PID(1.5, 0.05, 0.05)
        theta_pid = PID(1.5, 0.0, 0.0)
        theta_pid.send(None)

        x_tol = 0.003 # x translation tolerance
        y_tol = 0.01 # y translation tolerance
        theta_tol = 5 # angular tolerance (deg)


        goal_reached = False

        while not goal_reached:
            # detect ArUco markers
            if display_frames:
                coords, frame = self.get_marker_coordinates(marker_id, aruco_dict=aruco_dict, return_frame=True)
                width = frame.shape[1]
                height = frame.shape[0]
                target_point = (width // 2, height // 2 - int(y_offset * height / 2))
                # draw target point of box
                blue = (255, 0, 0)
                cv2.circle(frame, target_point, 5, blue, -1)
                cv2.imshow("Frame", frame)
                cv2.waitKey(20)
            else:
                coords = self.get_marker_coordinates(marker_id, aruco_dict=aruco_dict)
            
            # check if the specified marker is detected
            if coords is not None:
                print("marker detected")
                x, y, theta = coords
                # move gripper above box
                y = y + y_offset
                x_reached = (abs(x) <= x_tol)
                y_reached = (abs(y) <= y_tol)
                theta_reached = (np.abs(theta) <= theta_tol)
                if x_reached and y_reached and theta_reached:
                    print("goal reached")
                    # stop the arm from moving
                    self.arm.stop()
                    break
                # calculate appropriate velocities
                t = time()
                vel_x = x_pid.send([t, x, 0])
                vel_y = y_pid.send([t, y, 0])
                # # turn towards the closest side
                if theta > 45:
                    theta -= 90
                vel_theta = -theta_pid.send([t, theta, 0])
                # ang_vel_z = 0
                # print(vel_x, vel_y, vel_theta)
                self.arm.twist((vel_x, vel_y, 0, 0, 0, vel_theta))
            else:
                # no marker detected
                print("no marker detected")
                # stop moving
                self.arm.twist((0, 0, 0, 0, 0, 0))
        # close any open OpenCV windows
        cv2.destroyAllWindows()

    def get_marker_coordinates(self, marker_id, return_frame=False, aruco_dict=cv2.aruco.DICT_ARUCO_ORIGINAL):
        '''return coordinates (center_x, center_y, angle) of the specifed ArUco marker'''
        coords = None

        _, frame = self.arm.cap.read()
        frame = imutils.resize(frame, width=300)
        
        width = frame.shape[1]
        height = frame.shape[0]
        center_frame = (width // 2, height // 2)
        
        # detect ArUco markers
        aruco_dict = cv2.aruco.Dictionary_get(aruco_dict)
        aruco_params = cv2.aruco.DetectorParameters_create()
        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)
        if marker_ids is not None:
            marker_ids = marker_ids.flatten()
            for c4, mid in zip(corners, marker_ids):
                # print(c4)
                # find center point, angle
                (cX, cY), _, angle = cv2.minAreaRect(c4)
                # print(cX, cY, angle)
                if angle > 45:
                    angle -= 90

                # normalize the coordinates
                # print(frame.shape)
                cX_norm = (cX / width) * 2 - 1
                cY_norm = (cY / height) * 2 - 1
                # angle = np.deg2rad(angle)
                if (mid == marker_id):
                    coords =  np.array([cX_norm, cY_norm, angle])


                # display code
                if return_frame:
                    color = (0, 255, 0) if (mid == marker_id) else (0, 0, 255)
                    white = (255, 255, 255)
                    black = (0, 0, 0)
                    # draw bounding box
                    cv2.drawContours(frame, [c4.astype(int)], 0, color, 3)
                    # draw center of box
                    cv2.circle(frame, (int(cX), int(cY)), 5, color, -1)
                    # label box
                    cv2.putText(frame, f"id: {mid}", (int(cX) - 20, int(cY) - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, white, 2)
                    # label angle
                    cv2.putText(frame, f"{angle:.0f} deg", (int(cX) - 20, int(cY) + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, white, 2)

        if return_frame:
            return coords, frame
        return coords




if __name__ == "__main__":
    # initialize device
    device =  StorageDevice()

    # test device
    print(device.arm.get_pose())

    item_id = 0
    # while True:
    #     coords, frame = device.get_marker_coordinates(item_id, return_frame=True)
    #     # print(coords)
    #     cv2.imshow("Frame", frame)
    #     if cv2.waitKey(20) & 0xFF == ord('q'):
    #         break

    # device.go_to_marker(item_id, display_frames=False)

    device.pick(item_id, display_frames=False)

    placement_id = 0
    device.place(placement_id, display_frames=False)

    # disconnect from device
    device.arm.grip(0)
    device.disconnect()