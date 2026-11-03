"""Integration test against a simulated motor transport, never a physical robot."""
import os
os.environ['ROS_DOMAIN_ID'] = '187'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectoryPoint
from advrf_ros2_interfaces.msg import MotorTxPdoVector


class ActionTest(unittest.TestCase):
    def test_execution(self):
        with tempfile.TemporaryDirectory() as tmp:
            os.environ['ROS_LOG_DIR'] = tmp
            mapping = Path(tmp) / 'middleware.yaml'
            mapping.write_text('joint_map: {7: a, 2: b, 4: imu}\n')
            config = Path(tmp) / 'controller.yaml'
            config.write_text(f'''/**:
  ros__parameters:
    middleware_config: {mapping}
    joints: [a, b]
    position_lower: [-2.0, -2.0]
    position_upper: [2.0, 2.0]
    max_velocity: [10.0, 10.0]
    max_acceleration: [100.0, 100.0]
    feedback_timeout: 0.15
    path_tolerance: 0.5
    goal_tolerance: 0.005
    goal_time_tolerance: 0.15
''')
            log = open(Path(tmp) / 'node.log', 'w+')
            rclpy.init()
            node = rclpy.create_node('controller_test')
            process = subprocess.Popen([sys.argv[1], '--ros-args', '--params-file', str(config)], stdout=log, stderr=log)
            output_process = subprocess.Popen([sys.argv[2], '--ros-args', '--params-file', str(config)], stdout=log, stderr=log)
            pub = node.create_publisher(JointState, 'rx/joint_states', 10)
            positions = {7: 0.0, 2: 0.0}
            received = []
            upstream_commands = []
            upstream_sub = node.create_subscription(JointState, 'joint_commands', upstream_commands.append, 10)
            feedback = []
            mode = node.create_client(SetBool, 'set_trajectory_enabled')
            follow = [True]
            enabled = [True]

            def command(msg):
                received.append(msg)
                if follow[0]:
                    for motor in msg.data:
                        positions[motor.ecat_id] = motor.pos_ref

            sub = node.create_subscription(MotorTxPdoVector, 'tx/motors', command, 10)

            def state():
                if enabled[0]:
                    msg = JointState()
                    msg.name = ['b', 'a']  # Deliberately reversed feedback order.
                    msg.position = [positions[2], positions[7]]
                    pub.publish(msg)

            timer = node.create_timer(0.01, state)
            client = ActionClient(node, FollowJointTrajectory, 'advrf_motors_controller/follow_joint_trajectory')

            def spin(seconds):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.01)

            def wait(future, timeout=4):
                end = time.monotonic() + timeout
                while not future.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.01)
                self.assertTrue(future.done(), 'Action timed out')
                return future.result()

            def goal(target, duration=1):
                g = FollowJointTrajectory.Goal()
                g.trajectory.joint_names = ['b', 'a']
                p = JointTrajectoryPoint()
                p.positions = [target, target / 2]
                p.velocities = [0.0, 0.0]
                p.accelerations = [0.0, 0.0]
                p.time_from_start.sec = duration
                g.trajectory.points = [p]
                return g

            def partial_goal(target, duration=1):
                g = FollowJointTrajectory.Goal()
                g.trajectory.joint_names = ['a']
                p = JointTrajectoryPoint()
                p.positions = [target]
                p.velocities = [0.0]
                p.accelerations = [0.0]
                p.time_from_start.sec = duration
                g.trajectory.points = [p]
                return g

            try:
                self.assertTrue(client.wait_for_server(timeout_sec=10))
                spin(0.3)
                handle = wait(client.send_goal_async(goal(0.2), feedback_callback=lambda f: feedback.append(f)))
                self.assertTrue(handle.accepted)
                result = wait(handle.get_result_async())
                self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
                self.assertEqual(result.result.error_code, 0)
                self.assertTrue(feedback)
                self.assertTrue(any(abs(p.pos_ref - 0.2) < 0.005 for m in received for p in m.data if p.ecat_id == 2))
                self.assertTrue(any(0 < p.pos_ref < 0.19 for m in received for p in m.data if p.ecat_id == 2))
                count = len(received)
                handle = wait(client.send_goal_async(partial_goal(0.15)))
                self.assertTrue(handle.accepted)
                result = wait(handle.get_result_async())
                self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
                partial_commands = received[count:]
                self.assertTrue(any(abs(p.pos_ref - 0.15) < 0.005
                                    for m in partial_commands for p in m.data if p.ecat_id == 7))
                self.assertTrue(all(abs(p.pos_ref - 0.2) < 0.005
                                    for m in partial_commands for p in m.data if p.ecat_id == 2))
                self.assertTrue(mode.wait_for_service(timeout_sec=3))
                endpoints = node.get_publishers_info_by_topic('tx/motors')
                self.assertEqual([endpoint.node_name for endpoint in endpoints],
                                 ['converter_tx_motor_node'])
                self.assertTrue(wait(mode.call_async(SetBool.Request(data=False))).success)
                spin(0.1)
                count = len(upstream_commands)
                spin(0.15)
                self.assertEqual(len(upstream_commands), count, 'Disabled trajectory hold published')
                self.assertFalse(wait(client.send_goal_async(goal(0.2))).accepted)
                self.assertTrue(wait(mode.call_async(SetBool.Request(data=True))).success)
                spin(0.15)
                self.assertEqual(len(upstream_commands), count, 'Old hold resumed after mode change')
                self.assertFalse(wait(client.send_goal_async(goal(9))).accepted)
                old = goal(0.2)
                old.trajectory.header.stamp.sec = node.get_clock().now().seconds_nanoseconds()[0] - 1
                self.assertFalse(wait(client.send_goal_async(old)).accepted)
                unknown = goal(0.2)
                unknown.trajectory.joint_names[0] = 'unknown'
                self.assertFalse(wait(client.send_goal_async(unknown)).accepted)
                h = wait(client.send_goal_async(goal(0.3, 3)))
                self.assertTrue(h.accepted)
                self.assertFalse(wait(mode.call_async(SetBool.Request(data=False))).success,
                                 'Mode changed while a trajectory was active')
                self.assertFalse(wait(client.send_goal_async(goal(0.1))).accepted)
                self.assertTrue(wait(h.cancel_goal_async()).goals_canceling)
                self.assertEqual(wait(h.get_result_async()).status, GoalStatus.STATUS_CANCELED)
                h = wait(client.send_goal_async(goal(0.3, 3)))
                enabled[0] = False
                result = wait(h.get_result_async())
                self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
                self.assertEqual(result.result.error_code, -4)
                self.assertFalse(wait(client.send_goal_async(goal(0.1))).accepted)
                enabled[0] = True
                spin(0.2)
                follow[0] = False
                h = wait(client.send_goal_async(goal(0.25)))
                result = wait(h.get_result_async())
                self.assertEqual(result.result.error_code, -5)
                h = wait(client.send_goal_async(goal(1.0)))
                result = wait(h.get_result_async())
                self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
                self.assertEqual(result.result.error_code, -4)
            finally:
                client.destroy()
                node.destroy_node()
                rclpy.shutdown()
                process.terminate()
                process.wait(timeout=5)
                output_process.terminate()
                output_process.wait(timeout=5)
                log.seek(0)
                print(log.read())
                log.close()


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
