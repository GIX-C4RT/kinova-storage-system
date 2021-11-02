import time
import sys
import os
import threading

import cv2

from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient


from kortex_api.autogen.messages import Session_pb2, Base_pb2, BaseCyclic_pb2

import utilities


# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

class Arm():
    def __init__(self, ip_address="192.168.1.10", username="admin", password="admin"):
        '''Return an instance of the Arm class. Must be used in a with statement.'''
        self.ip = ip_address
        self.username = username
        self.password = password

        # Connect to arm and open video stream.
        self.device_connection = utilities.DeviceConnection.createTcpConnection(self)
        self.router = self.device_connection.__enter__()
        # Create required services
        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(self.router)
        # open video stream
        self.cap = cv2.VideoCapture("rtsp://" + self.ip + "/color")

    
    def disconnect(self):
        '''Disconnect from arm and close video stream.'''
        self.device_connection.__exit__(None, None, None)
        self.cap.release()

    # Create closure to set an event after an END or an ABORT
    def check_for_end_or_abort(self, e):
        """Return a closure checking for END or ABORT notifications
        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """
        def check(notification, e = e):
            print("EVENT : " + \
                Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
            or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check


    def stop(self):
        self.base.Stop()

    def get_pose(self):
        feedback = self.base_cyclic.RefreshFeedback()
        # print(feedback)
        pose = (feedback.base.tool_pose_x, feedback.base.tool_pose_y,
            feedback.base.tool_pose_z, feedback.base.tool_pose_theta_x,
            feedback.base.tool_pose_theta_y, feedback.base.tool_pose_theta_z)
        return pose


    def get_angles(self):
        feedback = self.base_cyclic.RefreshFeedback()
        angles = []
        for actuator in feedback.actuators:
            # print(actuator.position)
            angles.append(actuator.position)
        return tuple(angles)


    def get_feedback(self):
        return self.base_cyclic.RefreshFeedback()


    def home(self):
        '''Send the arm to its home position.'''
        # Make sure the arm is in Single Level Servoing mode
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
        
        # Move arm to ready position
        print("Moving the arm to its home position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == "Home":
                action_handle = action.handle

        if action_handle == None:
            print("Can't reach home position. Exiting.")
            sys.exit(0)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )
        
        self.base.ExecuteActionFromReference(action_handle)
        
        # Leave time to action to complete
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Home position reached.")
        else:
            print("Timeout on action notification wait.")
        return finished

    def twist(self, velocities=(0,0,0,0,0,0)):
        '''execute twist command with specified velocities in m/s and rad/s'''

        command = Base_pb2.TwistCommand()

        command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
        command.duration = 0

        twist = command.twist

        twist.linear_x, twist.linear_y, twist.linear_z, \
            twist.angular_x, twist.angular_y, twist.angular_z = velocities

        print("Sending the twist command")
        self.base.SendTwistCommand(command)

        return True

    def angles(self, angles):
        
        print("Starting angular action movement ...")
        action = Base_pb2.Action()
        action.name = "Example angular action movement"
        action.application_data = ""

        actuator_count = self.base.GetActuatorCount()

        if (actuator_count.count != len(angles)):
            print("Length of angles must match number of joints.")
            return

        # Place arm straight up
        for joint_id in range(actuator_count.count):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = angles[joint_id]

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )
        
        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Angular movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

    def pose(self, pose):
        
        print("Starting Cartesian action movement ...")
        action = Base_pb2.Action()
        action.name = "Cartesian action movement"
        action.application_data = ""

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = pose[0]         # (meters)
        cartesian_pose.y = pose[1]    # (meters)
        cartesian_pose.z = pose[2]    # (meters)
        cartesian_pose.theta_x = pose[3] # (degrees)
        cartesian_pose.theta_y = pose[4] # (degrees)
        cartesian_pose.theta_z = pose[5] # (degrees)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Cartesian movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

    def move_relative(self, delta):

        feedback = self.base_cyclic.RefreshFeedback()

        x = feedback.base.tool_pose_x + delta[0]         # (meters)
        y = feedback.base.tool_pose_y + delta[1]    # (meters)
        z = feedback.base.tool_pose_z + delta[2]    # (meters)
        theta_x = feedback.base.tool_pose_theta_x + delta[3] # (degrees)
        theta_y = feedback.base.tool_pose_theta_y + delta[4] # (degrees)
        theta_z = feedback.base.tool_pose_theta_z + delta[5] # (degrees)
        pose = (x, y, z, theta_x, theta_y, theta_z)

        self.pose(pose)


    def pose_2(self, pose):
        
        constrained_pose = Base_pb2.ConstrainedPose()

        feedback = self.base_cyclic.RefreshFeedback()

        cartesian_pose = constrained_pose.target_pose
        cartesian_pose.x = pose[0]          # (meters)
        cartesian_pose.y = pose[1]          # (meters)
        cartesian_pose.z = pose[2]          # (meters)
        cartesian_pose.theta_x = pose[3]    # (degrees)
        cartesian_pose.theta_y = pose[4]    # (degrees)
        cartesian_pose.theta_z = pose[5]    # (degrees)

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Reaching cartesian pose...")
        self.base.PlayCartesianTrajectory(constrained_pose)

        print("Waiting for movement to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Angular movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

    def grip(self, position):
        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.finger_identifier = 1
        finger.value = position

        tol = 0.01
        self.base.SendGripperCommand(gripper_command)

        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_command)
            if len (gripper_measure.finger):
                # print("Current position is : {0}".format(gripper_measure.finger[0].value))
                if abs(gripper_measure.finger[0].value -position) < tol:
                    break
            else: # Else, no finger present in answer, end loop
                break


if __name__ == "__main__":
    # initialize arm
    arm = Arm()
    arm.home() # go to home position
    arm.grip(0) # open gripper

    # test functions

    print(arm.get_pose())
    print(arm.get_angles())

    # spin end effector for 5 seconds
    arm.twist((0, 0, 0, 0, 0, 10))
    time.sleep(2)
    arm.stop()

    arm.grip(.5) # close gripper halfway

    arm.angles((0, 0, 0, 0, 0, 0)) # point straight up

    # terminate
    arm.home()
    arm.disconnect()