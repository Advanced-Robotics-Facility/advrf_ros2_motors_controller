"""GoTo -> trajectory action -> simulated motors, on an isolated DDS domain."""
import os
os.environ['ROS_DOMAIN_ID'] = '189'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from advrf_interfaces.action import GoToJointPositions
from advrf_interfaces.msg import MotorTxPdoVector
from sensor_msgs.msg import JointState


class GoToTest(unittest.TestCase):
    def test_execution_and_cancellation(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            os.environ['ROS_LOG_DIR'] = str(root)
            mapping = root / 'robot_id_map.yaml'
            mapping.write_text('joint_map: {7: a, 2: b}\n')
            config = root / 'config.yaml'
            config.write_text(f'''/**:
  ros__parameters:
    robot_id_map: {mapping}
    joints: [a, b]
    position_lower: [-2.0, -2.0]
    position_upper: [2.0, 2.0]
    max_velocity: [1.0, 1.0]
    max_acceleration: [2.0, 2.0]
    feedback_timeout: 0.2
    path_tolerance: 0.2
    goal_tolerance: 0.005
    goal_time_tolerance: 0.2
    response_timeout: 0.5
''')
            processes = []
            logs = []
            rclpy.init()
            node = rclpy.create_node('go_to_test')
            enabled = [True]
            moving = [False]
            follow = [True]
            positions = {7: 0.0, 2: 0.35}
            commands = []
            publisher = node.create_publisher(JointState, 'rx/joint_states', 10)

            def state():
                if enabled[0]:
                    message = JointState()
                    message.name = ['b', 'a']
                    message.position = [positions[2], positions[7]]
                    message.velocity = [0.1 if moving[0] else 0.0, 0.0]
                    publisher.publish(message)

            def command(message):
                commands.append(message)
                if follow[0]:
                    for motor in message.data:
                        positions[motor.ecat_id] = motor.pos_ref

            node.create_timer(0.01, state)
            node.create_subscription(MotorTxPdoVector, 'tx/motors', command, 10)
            client = ActionClient(node, GoToJointPositions, 'go_to_joint_positions')

            def spin(duration):
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.01)

            def wait(future, timeout=6):
                end = time.monotonic() + timeout
                while not future.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.01)
                self.assertTrue(future.done(), 'Action timed out')
                return future.result()

            def goal(target, scaling=1.0, names=None):
                request = GoToJointPositions.Goal()
                request.joint_names = ['a'] if names is None else names
                request.positions = [float(target)]
                request.velocity_scaling = float(scaling)
                return request

            try:
                for executable in sys.argv[1:3]:
                    log = open(root / (Path(executable).name + '.log'), 'w+')
                    logs.append(log)
                    processes.append(subprocess.Popen(
                        [executable, '--ros-args', '--params-file', str(config)],
                        stdout=log, stderr=log,
                    ))
                self.assertTrue(client.wait_for_server(timeout_sec=10))
                spin(0.5)
                feedback = []
                handle = wait(client.send_goal_async(goal(0.2), feedback_callback=feedback.append))
                self.assertTrue(handle.accepted)
                result = wait(handle.get_result_async())
                self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
                self.assertEqual(result.result.error_code, 0)
                self.assertAlmostEqual(positions[7], 0.2, delta=0.005)
                self.assertAlmostEqual(positions[2], 0.35, delta=0.005)
                self.assertTrue(feedback)
                self.assertTrue(any(0 < motor.pos_ref < 0.19 for msg in commands
                                    for motor in msg.data if motor.ecat_id == 7))
                self.assertFalse(wait(client.send_goal_async(goal(9))).accepted)
                self.assertFalse(wait(client.send_goal_async(goal(0.1, 2))).accepted)
                self.assertFalse(wait(client.send_goal_async(goal(0.1, names=['unknown']))).accepted)

                handle = wait(client.send_goal_async(goal(1.0, 0.25)))
                self.assertTrue(handle.accepted)
                self.assertFalse(wait(client.send_goal_async(goal(0.1))).accepted)
                spin(0.1)
                self.assertTrue(wait(handle.cancel_goal_async()).goals_canceling)
                self.assertEqual(wait(handle.get_result_async()).status, GoalStatus.STATUS_CANCELED)

                moving[0] = True
                spin(0.1)
                self.assertFalse(wait(client.send_goal_async(goal(0.1))).accepted)
                moving[0] = False
                spin(0.1)
                handle = wait(client.send_goal_async(goal(0.5)))
                self.assertTrue(handle.accepted)
                enabled[0] = False
                result = wait(handle.get_result_async())
                self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
                self.assertEqual(result.result.error_code, -4)
                self.assertFalse(wait(client.send_goal_async(goal(0.1))).accepted)
                enabled[0] = True
                spin(0.2)
                follow[0] = False
                handle = wait(client.send_goal_async(goal(0.6)))
                self.assertTrue(handle.accepted)
                self.assertEqual(wait(handle.get_result_async()).status, GoalStatus.STATUS_ABORTED)

                follow[0] = True
                spin(0.1)
                handle = wait(client.send_goal_async(goal(0.8)))
                self.assertTrue(handle.accepted)
                spin(0.1)
                processes[1].terminate()
                processes[1].wait(timeout=5)
                result = wait(handle.get_result_async())
                self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
                self.assertIn('timeout', result.result.message)
                self.assertFalse(wait(client.send_goal_async(goal(0.1))).accepted)
            finally:
                for process in processes:
                    process.terminate()
                    process.wait(timeout=5)
                client.destroy()
                node.destroy_node()
                rclpy.shutdown()
                for log in logs:
                    log.seek(0)
                    print(log.read())
                    log.close()


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
