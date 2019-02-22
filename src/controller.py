#!/usr/bin/env python
#
# Custom controller that uses MoveIt for ef control for widowX arm at RAIL
# Author: Dian Chen (dianchen@berkeley.edu)

import rospy
from geometry_msgs.msg import (
    PointStamped,
    Point,
    Quaternion,
    PoseStamped,
    Pose,
)
from moveit_commander import *
from moveit_msgs.srv import *
from arbotix_msgs.srv import *

from moveit_commander.exception import MoveItCommanderException

import numpy as np
import pickle
import traceback

from config import *


class WidowX:

    def __init__(self, boundaries=True):
        self.scene = PlanningSceneInterface()
        self.commander = MoveGroupCommander("widowx_arm")
        self.gripper = MoveGroupCommander("widowx_gripper")

        self.commander.set_end_effector_link('gripper_rail_link')

        rospy.sleep(2)

        if boundaries:
            self.add_bounds()

    def add_bounds(self):
        floor = PoseStamped()
        floor.header.frame_id = self.commander.get_planning_frame()
        floor.pose.position.x = 0
        floor.pose.position.y = 0
        floor.pose.position.z = .5
        self.scene.add_box('floor', floor, (1., 1., .001))

        leftWall = PoseStamped()
        leftWall.header.frame_id = self.commander.get_planning_frame()
        leftWall.pose.position.x = .16
        leftWall.pose.position.y = 0
        leftWall.pose.position.z = .475
        self.scene.add_box('leftWall', leftWall, (.001, .35, .08))

        rightWall = PoseStamped()
        rightWall.header.frame_id = self.commander.get_planning_frame()
        rightWall.pose.position.x = -.16
        rightWall.pose.position.y = 0
        rightWall.pose.position.z = .475
        self.scene.add_box('rightWall', rightWall, (.001, .35, .08))

        frontWall = PoseStamped()
        frontWall.header.frame_id = self.commander.get_planning_frame()
        frontWall.pose.position.x = 0
        frontWall.pose.position.y = -.14
        frontWall.pose.position.z = .475
        self.scene.add_box('frontWall', frontWall, (.35, .001, .08))

        backWall = PoseStamped()
        backWall.header.frame_id = self.commander.get_planning_frame()
        backWall.pose.position.x = 0
        backWall.pose.position.y = .14
        backWall.pose.position.z = .475
        self.scene.add_box('backWall', backWall, (.35, .001, .08))

    def remove_bounds(self):
        for obj in self.scene.get_objects().keys():
            self.scene.remove_world_object(obj)

    def open_gripper(self, drop=False):
        plan = self.gripper.plan(GRIPPER_DROP if drop else GRIPPER_OPEN)
        return self.gripper.execute(plan, wait=True)

    def close_gripper(self):
        plan = self.gripper.plan(GRIPPER_CLOSED)
        return self.gripper.execute(plan, wait=True)

    def eval_grasp(self, threshold=.001):
        current = np.array(self.gripper.get_current_joint_values())
        target = np.array(GRIPPER_CLOSED)
        error = current[0] - target[0]
        return error > threshold, error

    def orient_to_target(self, x=None, y=None, angle=None):
        current = self.get_joint_values()
        if x or y:
            angle = np.arctan2(y, x)
        if angle >= 3.1:
            angle = 3.1
        elif angle <= -3.1:
            angle = -3.1
        current[0] = angle
        # current[3] = -1.57
        plan = self.commander.plan(current)
        return self.commander.execute(plan, wait=True)

    def wrist_rotate(self, angle):
        rotated_values = self.commander.get_current_joint_values()
        rotated_values[4] = angle - rotated_values[0]
        if rotated_values[4] > np.pi / 2:
            rotated_values[4] -= np.pi
        elif rotated_values[4] < -(np.pi / 2):
            rotated_values[4] += np.pi
        plan = self.commander.plan(rotated_values)
        return self.commander.execute(plan, wait=True)

    def get_joint_values(self):
        return self.commander.get_current_joint_values()

    def get_current_pose(self):
        return self.commander.get_current_pose()

    def move_to_neutral(self):
        print('Moving to neutral...')
        plan = self.commander.plan(NEUTRAL_VALUES)
        return self.commander.execute(plan, wait=True)

    def move_to_drop(self, angle=None):
        drop_positions = DROPPING_VALUES[:]
        if angle:
            drop_positions[0] = angle
        plan = self.commander.plan(drop_positions)
        return self.commander.execute(plan, wait=True)

    def move_to_empty(self):
        plan = self.commander.plan(EMPTY_VALUES)
        return self.commander.execute(plan, wait=True)

    def move_to_reset(self):
        print('Moving to reset...')
        plan = self.commander.plan(RESET_VALUES)
        return self.commander.execute(plan, wait=True)

    def orient_to_pregrasp(self, x, y):
        angle = np.arctan2(y, x)
        return self.move_to_drop(angle)

    def move_to_grasp(self, x, y, z, angle):
        current_p = self.commander.get_current_pose().pose
        p1 = Pose(position=Point(x=x, y=y, z=z), orientation=DOWN_ORIENTATION)
        plan, f = self.commander.compute_cartesian_path(
            [current_p, p1], 0.001, 0.0)

        joint_goal = list(plan.joint_trajectory.points[-1].positions)

        first_servo = joint_goal[0]

        joint_goal[4] = (angle - first_servo) % np.pi
        if joint_goal[4] > np.pi / 2:
            joint_goal[4] -= np.pi
        elif joint_goal[4] < -(np.pi / 2):
            joint_goal[4] += np.pi

        try:
            plan = self.commander.plan(joint_goal)
        except MoveItCommanderException as e:
            print('Exception while planning')
            traceback.print_exc(e)
            return False

        return self.commander.execute(plan, wait=True)

    def move_to_vertical(self, z, force_orientation=True, shift_factor=1.0):
        current_p = self.commander.get_current_pose().pose
        current_angle = self.get_joint_values()[4]
        orientation = current_p.orientation if force_orientation else None
        p1 = Pose(position=Point(x=current_p.position.x * shift_factor,
                                 y=current_p.position.y * shift_factor, z=z), orientation=orientation)
        waypoints = [current_p, p1]
        plan, f = self.commander.compute_cartesian_path(waypoints, 0.001, 0.0)

        if not force_orientation:
            return self.commander.execute(plan, wait=True)
        else:
            if len(plan.joint_trajectory.points) > 0:
                joint_goal = list(plan.joint_trajectory.points[-1].positions)
            else:
                return False

            joint_goal[4] = current_angle

            plan = self.commander.plan(joint_goal)
            return self.commander.execute(plan, wait=True)

    def move_to_target(self, target):
        plan = self.commander.plan(target)
        return self.commander.execute(plan, wait=True)

    def sweep_arena(self):
        self.remove_bounds()
        
        self.move_to_drop(.8)
        plan = self.commander.plan(TL_CORNER[0])
        self.commander.execute(plan, wait=True)

        plan = self.commander.plan(TL_CORNER[1])
        self.commander.execute(plan, wait=True)

        plan = self.commander.plan(L_SWEEP[0])
        self.commander.execute(plan, wait=True)

        plan = self.commander.plan(L_SWEEP[1])
        self.commander.execute(plan, wait=True)

        self.move_to_drop(-.8)
        plan = self.commander.plan(BL_CORNER[0])
        self.commander.execute(plan, wait=True)

        plan = self.commander.plan(BL_CORNER[1])
        self.commander.execute(plan, wait=True)

        self.move_to_drop(-2.45)
        plan = self.commander.plan(BR_CORNER[0])
        self.commander.execute(plan, wait=True)
        plan = self.commander.plan(BR_CORNER[1])
        self.commander.execute(plan, wait=True)

        self.move_to_drop(2.3)
        plan = self.commander.plan(TR_CORNER[0])
        self.commander.execute(plan, wait=True)
        plan = self.commander.plan(TR_CORNER[1])
        self.commander.execute(plan, wait=True)

        self.add_bounds()

    def discard_object(self):
        plan = self.commander.plan(PREDISCARD_VALUES)
        self.commander.execute(plan, wait=True)
        plan = self.commander.plan(DISCARD_VALUES)
        self.commander.execute(plan, wait=True)
        self.open_gripper(drop=True)
        plan = self.commander.plan(PREDISCARD_VALUES)
        self.commander.execute(plan, wait=True)
        self.move_to_neutral()


def main():
    rospy.init_node("widowx_custom_controller")
    widowx = WidowX()

    print('For debugging purposes')
    import pdb; pdb.set_trace()
    pass

if __name__ == '__main__':
    main()
