#!/usr/bin/env python

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import (
    Image,
    PointCloud2,
    CameraInfo
)
from std_msgs.msg import (
    UInt16,
    Int64,
    Float32,
    Header,
    String
)
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge, CvBridgeError

import argparse
import traceback

from controller import WidowX
from policy import *
from config import *
from utils import *


class HumanExecutor:

    def __init__(self, execute=False, calibrate=False):
        # WidowX controller interface
        self.widowx = WidowX()

        # Register subscribers
        self.pc_subscriber = rospy.Subscriber(
            "/camera/depth/color/points", PointCloud2, self.update_pc)
        self.camerainfo_subscriber = rospy.Subscriber(
            "/camera/depth/camera_info", CameraInfo, self.save_caminfo)
        self.action_subscriber = rospy.Subscriber(
            "/target_action", String, self.take_action)
        self.rgb_subscriber = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.update_rgb)

        # Store latest RGB-D
        self.rgb = None
        self.depth = None
        self.pc = None

        self.bridge = CvBridge()

        self.cm = CALIBRATION_MATRIX

        self.robot_coordinates = []
        self.camera_coordinates = []

        self.execute = execute
        self.calibrate = calibrate

    def update_rgb(self, data):
        self.rgb = self.bridge.imgmsg_to_cv2(data)

    def update_d(self, data):
        self.depth = self.bridge.imgmsg_to_cv2(data)

    def update_pc(self, data):
        self.pc = list(pc2.read_points(data, skip_nans=False))

    def save_caminfo(self, data):
        self.cam_info = data

    def take_action(self, data):
        x, y = data.data.split('(')[1].split(')')[0].split(',')
        register_index = int(x) + (int(y) * 640)
        pc_point = list(self.pc[register_index])

        print('Pixel: (%s, %s)' % (x, y))
        print('PC point: ' + str(pc_point))

        if pc_point[2] == 0:
            print('No depth reading')
            print(pc_depth)
            return

        if self.execute:
            pc_point = np.array([pc_point[0], pc_point[1], pc_point[2], 1.])
            transformed = np.dot(self.cm, pc_point)[:3]
            print(transformed)

            theta = 0.0
            grasp = np.concatenate([transformed, [theta]], axis=0)
            grasp[2] -= Z_OFFSET
            grasp[:2] *= CONTROL_NOISE_COEFFICIENT

            print('Grasp: ' + str(grasp))
            success, actual_grasp = self.execute_grasp(grasp)
            self.widowx.move_to_neutral()

        elif self.calibrate:
            user_in = raw_input('Keep recorded point? [y/n] ')

            if user_in == 'y':
                pose = self.widowx.get_current_pose().pose.position
                print(pose.x, pose.y, pose.z)

                self.robot_coordinates.append((pose.x, pose.y, pose.z))
                self.camera_coordinates.append(pc_point[:3])
                print('Saved')

            elif user_in == 'n':
                print('Not saved')

    def get_pose(self):
        pose = self.widowx.get_current_pose().pose
        pose_list = [pose.position.x,
                     pose.position.y,
                     pose.position.z,
                     pose.orientation.w,
                     pose.orientation.x,
                     pose.orientation.y,
                     pose.orientation.z]
        return pose_list

    def execute_grasp(self, grasp):
        try:
            x, y, z, theta = grasp
            print('Attempting grasp: (%.4f, %.4f, %.4f, %.4f)'
                  % (x, y, z, theta))

            assert self.widowx.orient_to_target(
                x, y), 'Failed to orient to target'

            assert self.widowx.move_to_grasp(x, y, z, theta), \
                'Failed to reach target'

            self.widowx.close_gripper()

            pose = self.get_pose()
            joints = self.widowx.get_joint_values()

            actual_grasp = pose[:3] + [joints[4]]

            print('Actual grasp: (%.4f, %.4f, %.4f, %.4f)' % (
                actual_grasp[0], actual_grasp[1], actual_grasp[2], actual_grasp[3]))

            reached = self.widowx.move_to_vertical(z - GRASP_LIFT_CLEARANCE)
            offset = 0.0
            while not reached and offset < GRASP_LIFT_CLEARANCE:
                offset += .005
                reached = self.widowx.move_to_vertical(
                    z - GRASP_LIFT_CLEARANCE + offset)

            assert self.widowx.move_to_drop(), 'Failed to move to drop'

            success, err = self.widowx.eval_grasp()

            return success, actual_grasp

        except AssertionError as e:
            print('Error executing grasp -- returning...')
            traceback.print_exc(e)
            return False, None


def main():
    parser = argparse.ArgumentParser(
        description="Executes user-specified grasps from a GUI window")
    parser.add_argument('--debug', action="store_true", default=False,
                        help="Prevents grasp from being executed (for debugging purposes)")
    args = parser.parse_args()

    rospy.init_node("human_executor")

    executor = HumanExecutor(execute=(not args.debug), calibrate=False)
    print('Run commander_human.py to issue commands from the GUI window')

    rospy.sleep(2)
    rospy.spin()

if __name__ == '__main__':
    main()
