"""Temporary action client: measured position -> min, then repeat min/max sweeps."""

import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .panel_config import (
    load_joints,
    load_parameters,
    measured_positions,
    resolve_panel_config,
)


def segment(start, end, duration, rate):
    """Sample a quintic position profile with zero endpoint speed/acceleration."""
    finite = all(math.isfinite(value) for value in (start, end, duration, rate))
    if not finite or duration <= 0 or rate <= 0:
        raise ValueError('Finite positions, positive duration and rate are required')

    count = math.ceil(duration * rate)
    if count > 200000:
        raise ValueError('Too many trajectory points; reduce duration or rate')

    for i in range(count + 1):
        t = min(i / rate, duration)
        u = t / duration
        d = end - start

        position = start + d * (10 * u**3 - 15 * u**4 + 6 * u**5)
        velocity = d / duration * (30 * u**2 - 60 * u**3 + 30 * u**4)
        acceleration = d / duration**2 * (60 * u - 180 * u**2 + 120 * u**3)

        yield t, position, velocity, acceleration


def make_trajectory(names, actual, joint, targets, rate):
    index = names.index(joint)
    trajectory = JointTrajectory(joint_names=names)
    offset = 0.0
    start = actual[index]

    for target, duration in targets:
        samples = segment(start, target, duration, rate)

        for i, (t, position, velocity, acceleration) in enumerate(samples):
            if trajectory.points and i == 0:
                continue

            point = JointTrajectoryPoint()
            point.positions = list(actual)
            point.velocities = [0.0] * len(names)
            point.accelerations = [0.0] * len(names)

            point.positions[index] = position
            point.velocities[index] = velocity
            point.accelerations[index] = acceleration

            stamp = round((offset + t) * 1e9)
            seconds, nanoseconds = divmod(stamp, 1_000_000_000)
            point.time_from_start.sec = seconds
            point.time_from_start.nanosec = nanoseconds
            trajectory.points.append(point)

        offset += duration
        start = target

    return trajectory


class TrajectoryTest(Node):
    def __init__(self, args):
        super().__init__('test_joint_trajectory', namespace=args.namespace)

        config = resolve_panel_config(
            args.joint_limits,
            os.environ.get('EC_CFG', ''),
            os.environ.get('ROS2_CFG', ''),
        )
        self.joints = load_joints(config, self.get_namespace())
        self.parameters = load_parameters(config, self.get_namespace())
        self.names = [name for name, _, _ in self.joints]

        if args.joint not in self.names:
            raise ValueError(f'Joint {args.joint!r} not in config: {self.names}')

        self.args = args
        self.actual = None
        self.received = 0.0
        self.goal_handle = None
        self.pending_goal = None

        self.subscription = self.create_subscription(
            JointState,
            'rx/joint_states',
            self.receive,
            qos_profile_sensor_data,
        )
        self.client = ActionClient(self, FollowJointTrajectory, args.action)
        self.check_segment(args.minimum, args.maximum, args.duration)

    def receive(self, message):
        try:
            values = measured_positions(message.name, message.position, self.joints)
        except (ValueError, KeyError, TypeError):
            return

        self.actual = values
        self.received = time.monotonic()

    def fresh(self):
        return self.actual is not None and time.monotonic() - self.received < 0.5

    def snapshot(self):
        end = time.monotonic() + self.args.timeout

        # Consume fresh traffic, including state updates after the previous result.
        received_before = self.received
        while self.received <= received_before or not self.fresh():
            if time.monotonic() >= end:
                raise RuntimeError('No fresh complete joint feedback')
            rclpy.spin_once(self, timeout_sec=0.05)

        for value, (name, lo, hi) in zip(self.actual, self.joints):
            if not lo <= value <= hi:
                raise ValueError(f'{name}: measured position outside configured limits')

        return list(self.actual)

    def check_segment(self, start, target, duration):
        i = self.names.index(self.args.joint)
        _, lo, hi = self.joints[i]

        if not lo <= start <= hi or not lo <= target <= hi:
            raise ValueError(f'Target or measured position outside [{lo}, {hi}]')

        distance = abs(target - start)
        peaks = {
            'max_velocity': 1.875 * distance / duration,
            'max_acceleration': (10 / math.sqrt(3)) * distance / duration**2,
        }

        for key, peak in peaks.items():
            limits = self.parameters.get(key, [])
            if (
                len(limits) != len(self.joints)
                or not math.isfinite(limits[i])
                or limits[i] <= 0
            ):
                raise ValueError(f'Invalid {key} in joint configuration')

            if peak > limits[i] + 1e-9:
                raise ValueError(
                    f'{key} exceeded ({peak:.4g} > {limits[i]:.4g}); increase duration'
                )

    def wait(self, future, timeout=None):
        end = time.monotonic() + timeout if timeout else None

        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if end and time.monotonic() >= end:
                raise RuntimeError('Action response timeout; execution state may be unknown')

        return future.result()

    def execute(self, targets):
        actual = self.snapshot()
        start = actual[self.names.index(self.args.joint)]

        for target, duration in targets:
            self.check_segment(start, target, duration)
            start = target

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = make_trajectory(
            self.names,
            actual,
            self.args.joint,
            targets,
            self.args.rate,
        )

        self.pending_goal = self.client.send_goal_async(goal)
        self.goal_handle = self.wait(self.pending_goal, self.args.timeout)
        self.pending_goal = None

        if not self.goal_handle.accepted:
            self.goal_handle = None
            raise RuntimeError(
                'Trajectory rejected; check controller mode, feedback and limits'
            )

        execution_timeout = sum(duration for _, duration in targets) + self.args.timeout
        result = self.wait(self.goal_handle.get_result_async(), execution_timeout)
        self.goal_handle = None

        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError(
                f'Trajectory failed: status={result.status}, '
                f'code={result.result.error_code}, {result.result.error_string}'
            )

    def cancel(self):
        try:
            if self.pending_goal and not self.goal_handle:
                self.goal_handle = self.wait(self.pending_goal, self.args.timeout)

            if self.goal_handle and self.goal_handle.accepted:
                result = self.wait(self.goal_handle.cancel_goal_async(), self.args.timeout)
                if result.goals_canceling:
                    self.wait(self.goal_handle.get_result_async(), self.args.timeout)
                    self.get_logger().info('Trajectory cancelled. No motor STOP sent.')
                else:
                    self.get_logger().warning(
                        'Cancellation not accepted; verify controller state.'
                    )
        except Exception as error:
            self.get_logger().error(f'Cancellation not confirmed: {error}')


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint', required=True)
    parser.add_argument('--min', dest='minimum', type=float, required=True)
    parser.add_argument('--max', dest='maximum', type=float, required=True)
    parser.add_argument('--duration', type=float, required=True, help='Seconds per min/max leg')
    parser.add_argument(
        '--approach-duration',
        type=float,
        help='Seconds from initial position to min; defaults to duration',
    )
    parser.add_argument('--rate', type=float, default=20.0, help='Trajectory points per second')
    parser.add_argument(
        '--repeat',
        type=int,
        default=1,
        help='Number of min -> max -> min cycles; 0 means infinite',
    )
    parser.add_argument('--joint-limits', default='')
    parser.add_argument('--namespace', default='')
    parser.add_argument('--action', default='advrf_motors_controller/follow_joint_trajectory')
    parser.add_argument('--timeout', type=float, default=5.0)

    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if args.approach_duration is None:
        args.approach_duration = args.duration

    numeric_values = [
        args.minimum,
        args.maximum,
        args.duration,
        args.approach_duration,
        args.rate,
        args.timeout,
    ]
    if (
        not all(math.isfinite(value) for value in numeric_values)
        or min(args.duration, args.approach_duration, args.rate, args.timeout) <= 0
        or args.rate > 1000
        or args.minimum >= args.maximum
        or args.repeat < 0
    ):
        parser.error(
            'Require min < max, positive durations/rate/timeout, '
            'rate <= 1000 and repeat >= 0'
        )

    return args


def run_sequence(node, args):
    if not node.client.wait_for_server(timeout_sec=args.timeout):
        raise RuntimeError(
            'Trajectory action unavailable; start the trajectory controller first'
        )

    repetitions = args.repeat or 'infinite'
    node.get_logger().info(
        f'Approach current -> {args.minimum}; then {repetitions} min/max/min cycles. '
        'No motor START is sent.'
    )
    node.execute([(args.minimum, args.approach_duration)])

    cycle = 0
    while args.repeat == 0 or cycle < args.repeat:
        node.execute([
            (args.maximum, args.duration),
            (args.minimum, args.duration),
        ])
        cycle += 1
        node.get_logger().info(f'Cycle {cycle} complete')


def main():
    args = parse_arguments()
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 0

    try:
        node = TrajectoryTest(args)
        run_sequence(node, args)

    except KeyboardInterrupt:
        code = 130
        if node:
            node.cancel()

    except Exception as error:
        print(f'Trajectory test failed: {error}', file=sys.stderr)
        code = 1
        if node:
            node.cancel()

    finally:
        if node:
            node.client.destroy()
            node.destroy_node()
        rclpy.shutdown()

    return code
