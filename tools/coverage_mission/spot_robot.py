"""Thin Boston Dynamics SDK wrapper for the coverage mission.

Localization comes from Spot's own kinematic state (the EAP feeds the
robot's onboard perception): poses are read in the `vision` or `odom`
frame from the frame-tree snapshot, and goals are sent as SE2
trajectory commands in that same frame. Spot's onboard obstacle
avoidance handles the local path.
"""
import math
import os
import time


class SpotRobot:

    def __init__(self, ip, username=None, password=None, frame='vision',
                 walk_speed_mps=0.6, power_on=False, stand=True, log=print):
        self.ip = ip
        self.username = username or os.environ.get('BOSDYN_CLIENT_USERNAME')
        self.password = password or os.environ.get('BOSDYN_CLIENT_PASSWORD')
        self.frame = frame
        self.walk_speed = walk_speed_mps
        self.power_on = power_on
        self.stand = stand
        self.log = log
        self.robot = None
        self.command_client = None
        self.state_client = None
        self.lease_keepalive = None

    def frame_name(self):
        from bosdyn.client.frame_helpers import (
            ODOM_FRAME_NAME,
            VISION_FRAME_NAME,
        )
        if self.frame == 'odom':
            return ODOM_FRAME_NAME
        return VISION_FRAME_NAME

    def connect(self):
        import bosdyn.client
        from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
        from bosdyn.client.robot_command import (
            RobotCommandClient,
            blocking_stand,
        )
        from bosdyn.client.robot_state import RobotStateClient

        if not self.ip:
            raise RuntimeError('Spot IP is not configured')
        if not self.username or not self.password:
            raise RuntimeError(
                'Spot credentials missing: pass --spot-user/--spot-pass or '
                'set BOSDYN_CLIENT_USERNAME/BOSDYN_CLIENT_PASSWORD'
            )

        sdk = bosdyn.client.create_standard_sdk('CoverageMission')
        self.robot = sdk.create_robot(self.ip)
        self.robot.authenticate(self.username, self.password)
        self.robot.time_sync.wait_for_sync()

        if self.robot.is_estopped():
            raise RuntimeError(
                'Spot is estopped. Configure an external E-Stop endpoint '
                '(tablet or estop_gui) before running the mission.'
            )

        lease_client = self.robot.ensure_client(
            LeaseClient.default_service_name
        )
        self.lease_keepalive = LeaseKeepAlive(
            lease_client,
            must_acquire=True,
            return_at_exit=True,
        )
        self.command_client = self.robot.ensure_client(
            RobotCommandClient.default_service_name
        )
        self.state_client = self.robot.ensure_client(
            RobotStateClient.default_service_name
        )

        if self.power_on and not self.robot.is_powered_on():
            self.log('Powering on Spot motors...')
            self.robot.power_on(timeout_sec=25)
        if not self.robot.is_powered_on():
            raise RuntimeError(
                'Spot motors are off. Power on from the tablet or rerun '
                'with --power-on.'
            )
        if self.stand:
            self.log('Commanding Spot to stand...')
            blocking_stand(self.command_client, timeout_sec=15)
        self.log(f'Connected to Spot at {self.ip} (frame: {self.frame})')

    def pose(self):
        """Robot (x, y, yaw) in the mission world frame (vision/odom)."""
        from bosdyn.client.frame_helpers import (
            BODY_FRAME_NAME,
            get_se2_a_tform_b,
        )
        state = self.state_client.get_robot_state()
        se2 = get_se2_a_tform_b(
            state.kinematic_state.transforms_snapshot,
            self.frame_name(),
            BODY_FRAME_NAME,
        )
        if se2 is None:
            raise RuntimeError(
                f'No {self.frame} frame in the Spot frame tree snapshot'
            )
        return se2.x, se2.y, se2.angle

    def goto(self, x, y, yaw, timeout_sec=None):
        """Walk to an SE2 goal in the mission world frame; True on arrival."""
        from bosdyn.client.robot_command import (
            RobotCommandBuilder,
            block_for_trajectory_cmd,
        )

        start = self.pose()
        distance = math.hypot(x - start[0], y - start[1])
        travel_time = distance / max(0.1, self.walk_speed)
        if timeout_sec is None:
            timeout_sec = max(30.0, travel_time * 2.0 + 15.0)

        command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=x,
            goal_y=y,
            goal_heading=yaw,
            frame_name=self.frame_name(),
        )
        command_id = self.command_client.robot_command(
            command,
            end_time_secs=time.time() + timeout_sec,
        )
        self.log(
            f'Walking {distance:.1f}m to x={x:.2f}, y={y:.2f}, '
            f'yaw={yaw:.2f} ({self.frame} frame, timeout {timeout_sec:.0f}s)'
        )
        block_for_trajectory_cmd(
            self.command_client,
            command_id,
            timeout_sec=timeout_sec,
        )
        return True

    def arrival_error(self, x, y, yaw):
        pose = self.pose()
        distance = math.hypot(x - pose[0], y - pose[1])
        yaw_error = abs(math.atan2(
            math.sin(yaw - pose[2]),
            math.cos(yaw - pose[2]),
        ))
        return distance, yaw_error

    def sit(self):
        from bosdyn.client.robot_command import blocking_sit
        if self.command_client is not None:
            self.log('Commanding Spot to sit...')
            blocking_sit(self.command_client, timeout_sec=15)

    def shutdown(self):
        try:
            self.sit()
        except Exception as error:
            self.log(f'Sit on shutdown failed: {error}')
        if self.lease_keepalive is not None:
            self.lease_keepalive.shutdown()
            self.lease_keepalive = None


class DryRunRobot:
    """Stands in for Spot: teleports to goals so the loop can be tested
    without hardware (and without bosdyn-client installed)."""

    def __init__(self, log=print):
        self.log = log
        self.frame = 'dry_run'
        self.current = (0.0, 0.0, 0.0)

    def connect(self):
        self.log('Dry run: not connecting to Spot; robot teleports to goals')

    def pose(self):
        return self.current

    def goto(self, x, y, yaw, timeout_sec=None):
        self.log(f'Dry run: teleporting to x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        self.current = (x, y, yaw)
        return True

    def arrival_error(self, x, y, yaw):
        return 0.0, 0.0

    def sit(self):
        pass

    def shutdown(self):
        pass
