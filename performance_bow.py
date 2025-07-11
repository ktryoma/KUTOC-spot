# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Command the robot to go to an offset position using a trajectory command."""

import os
import logging
import math
import sys
import time
import argparse

import bosdyn.geometry
import bosdyn.client
import bosdyn.client.util
from bosdyn.api import basic_command_pb2
from bosdyn.api import geometry_pb2 as geo
from bosdyn import geometry
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client import math_helpers
from bosdyn.api import image_pb2, trajectory_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.util import seconds_to_duration
from bosdyn.client.frame_helpers import (GROUND_PLANE_FRAME_NAME, VISION_FRAME_NAME, 
                                         GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME,
                                         BODY_FRAME_NAME,
                                         get_a_tform_b,
                                         get_vision_tform_body,
                                         get_se2_a_tform_b)
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient, CommandFailedError, CommandTimedOutError,
                                         block_for_trajectory_cmd, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient

_LOGGER = logging.getLogger(__name__)

os.environ['BOSDYN_CLIENT_USERNAME'] = "user"
os.environ['BOSDYN_CLIENT_PASSWORD'] = "vvza0en6qb6m"

def main():
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args()
    bosdyn.client.util.setup_logging(options.verbose)
    
    print("start program")

    # Create robot object.
    sdk = bosdyn.client.create_standard_sdk('RobotCommandMaster')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)
    print("authenticated")
    
    # time_sync
    robot.time_sync.wait_for_sync()

    # Check that an estop is connected with the robot so that the robot commands can be executed.
    assert not robot.is_estopped(), 'Robot is estopped. Please use an external E-Stop client, ' \
                                    'such as the estop SDK example, to configure E-Stop.'

    # Create the lease client.
    lease_client = robot.ensure_client(LeaseClient.default_service_name)

    # Setup clients for the robot state and robot command services.
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    
    lease = lease_client.take()
    lease_keep_alive = LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True)
    
    resp = robot.power_on()
    
    try:
        blocking_stand(robot_command_client)
    except CommandFailedError as exc:
        print(f'Error ({exc}) occurred while trying to stand. Check robot surroundings.')
        return False
    except CommandTimedOutError as exc:
        print(f'Stand command timed out: {exc}')
        return False
    print('Robot powered on and standing.')
    
    frame = ODOM_FRAME_NAME
    while True:
        print("continue? y/n")
        if input() == "n":
            break
        try:
            # spot_bow_pose(robot_command_client, robot_state_client)
            bow(robot_command_client, robot_state_client)
        except Exception as e:
            print(f'Error occurred: {e}')
        
        
def bow(robot_command_client, robot_state_client):
    
    end_time = 2.0 
    
    footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.4, roll=0.0, pitch=0.0)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    robot_command_client.robot_command(cmd)
    time.sleep(1.0)
    
    footprint_R_body = geometry.EulerZXY(yaw=0.4, roll=0.0, pitch=-0.3)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
    time.sleep(end_time)
    
    footprint_R_body = geometry.EulerZXY(yaw=0.4, roll=0.0, pitch=0.3)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
    time.sleep(end_time)
    
    footprint_R_body = geometry.EulerZXY(yaw=-0.4, roll=0.0, pitch=0.0)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
    time.sleep(1.0)
    
    footprint_R_body = geometry.EulerZXY(yaw=-0.4, roll=0.0, pitch=-0.3)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
    time.sleep(end_time)
    
    footprint_R_body = geometry.EulerZXY(yaw=-0.4, roll=0.0, pitch=0.3)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
    time.sleep(end_time)
    
    footprint_R_body = geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=0.0)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
    time.sleep(1.0)
    
    footprint_R_body = geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=-0.3)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
    time.sleep(end_time)
    
    footprint_R_body = geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=0.3)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + 6.0)
    time.sleep(6.0)
    
    footprint_R_body = geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=0.0)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    robot_command_client.robot_command(lease=None, command=cmd, end_time_secs=time.time() + end_time)
    time.sleep(1.0)
    
        
        
def spot_bow_pose(robot_command_client, robot_state_client):
    
    walk_degree = 20
    bow_degree = 20
    end_time = 3.0
    
    mobility_params = create_mobility_params()
    robot_state = robot_state_client.get_robot_state()
    
    # リファクタリング: 角度コマンドの繰り返しをループにまとめる
    angles = [walk_degree, -walk_degree, -walk_degree, -walk_degree]
    for ang in angles:
        tag_cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(
            0, 0, math.radians(ang), robot_state.kinematic_state.transforms_snapshot, mobility_params)
        robot_command_client.robot_command(lease=None, command=tag_cmd,
                                           end_time_secs=time.time() + end_time)
        time.sleep(3.0)
    
    # Specify a trajectory to shift the body forward followed by looking down, then return to nominal.
    # Define times (in seconds) for each point in the trajectory.
    
    odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                        ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
    t1 = 0.5
    t2 = 2.5
    t2_1 = 7.0
    t3 = 9.0
    
    
    w = math.cos(math.radians(bow_degree/2))
    y = math.sin(math.radians(bow_degree/2))

    # Specify the poses as transformations to the cached flat_body pose.
    flat_body_T_pose1 = math_helpers.SE3Pose(x=0, y=0, z=0, rot=math_helpers.Quat())
    flat_body_T_pose2 = math_helpers.SE3Pose(
        x=0.0, y=0, z=0, rot=math_helpers.Quat(w=w, x=0, y=y, z=0))
    flat_body_T_pose2_1 = math_helpers.SE3Pose(
        x=0.0, y=0, z=0, rot=math_helpers.Quat(w=w, x=0, y=y, z=0))
    flat_body_T_pose3 = math_helpers.SE3Pose(x=0.0, y=0, z=0, rot=math_helpers.Quat())

    # Build the points in the trajectory.
    traj_point1 = trajectory_pb2.SE3TrajectoryPoint(
        pose=(odom_T_flat_body * flat_body_T_pose1).to_proto(),
        time_since_reference=seconds_to_duration(t1))
    traj_point2 = trajectory_pb2.SE3TrajectoryPoint(
        pose=(odom_T_flat_body * flat_body_T_pose2).to_proto(),
        time_since_reference=seconds_to_duration(t2))
    traj_point2_1 = trajectory_pb2.SE3TrajectoryPoint(
        pose=(odom_T_flat_body * flat_body_T_pose2_1).to_proto(),
        time_since_reference=seconds_to_duration(t2_1))
    traj_point3 = trajectory_pb2.SE3TrajectoryPoint(
        pose=(odom_T_flat_body * flat_body_T_pose3).to_proto(),
        time_since_reference=seconds_to_duration(t3))

    # Build the trajectory proto by combining the points.
    traj = trajectory_pb2.SE3Trajectory(points=[traj_point1, traj_point2, traj_point2_1, traj_point3])

    # Build a custom mobility params to specify absolute body control.
    body_control = spot_command_pb2.BodyControlParams(
        body_pose=spot_command_pb2.BodyControlParams.BodyPose(root_frame_name=ODOM_FRAME_NAME,
                                                                base_offset_rt_root=traj))
    blocking_stand(robot_command_client, timeout_sec=12,
                    params=spot_command_pb2.MobilityParams(body_control=body_control))
    
    time.sleep(8.0)

def create_mobility_params(vel_desired=0.5, ang_desired=0.25):
    speed_limit = geo.SE2VelocityLimit(
        max_vel=geo.SE2Velocity(
            linear=geo.Vec2(x=vel_desired, y=vel_desired),
            angular=ang_desired))
    body_control = set_default_body_control()  # 既存の set_default_body_control を利用
    return spot_command_pb2.MobilityParams(vel_limit=speed_limit, obstacle_params=None,
                                             body_control=body_control,
                                             locomotion_hint=spot_command_pb2.HINT_TROT)

def set_default_body_control():
    """Set default body control params to current body position"""
    footprint_R_body = geometry.EulerZXY()
    position = geo.Vec3(x=0.0, y=0.0, z=0.0)
    rotation = footprint_R_body.to_quaternion()
    pose = geo.SE3Pose(position=position, rotation=rotation)
    point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
    traj = trajectory_pb2.SE3Trajectory(points=[point])
    return spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)

if __name__ == '__main__':
    if not main():
        sys.exit(1)
