"""Exercise the direct controller on an isolated DDS domain with no hardware."""
import os
os.environ['ROS_DOMAIN_ID'] = '188'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
import rclpy
from sensor_msgs.msg import JointState
from advrf_ros2_interfaces.msg import MotorTxPdo
from advrf_ros2_interfaces.srv import SetJointGains
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter


class JointCommandTest(unittest.TestCase):
    def test_commands(self):
        with tempfile.TemporaryDirectory() as tmp:
            os.environ['ROS_LOG_DIR'] = tmp
            config = Path(tmp) / 'controller.yaml'
            config.write_text(f'''/**:
  ros__parameters:
    motor_ids: [7, 2]
    joints: [a, b]
    position_lower: [-2.0, -2.0]
    position_upper: [2.0, 2.0]
    max_velocity: [1.0, 1.0]
    max_acceleration: [5.0, 5.0]
    gains: [1.0, 2.0, 3.0, 4.0, 5.0]
    joint_gains: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
''')
            rclpy.init()
            node = rclpy.create_node('joint_command_test', namespace='/advrf/test_robot')
            pub = node.create_publisher(JointState, 'joint_commands', 10)
            other_pub = node.create_publisher(JointState, 'joint_commands', 10)
            received = []
            sub = node.create_subscription(MotorTxPdo, 'tx/motors', received.append, 10)
            with open(Path(tmp) / 'node.log', 'w+') as log:
                process = subprocess.Popen([sys.argv[1], '--ros-args', '--params-file', str(config), '-r', '__ns:=/advrf/test_robot'], stdout=log, stderr=log)

                def spin(seconds):
                    end = time.monotonic() + seconds
                    while time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.01)

                def send(names, positions, velocities=None, effort=None):
                    msg = JointState()
                    msg.name = names
                    msg.position = positions
                    msg.velocity = velocities or []
                    msg.effort = effort or []
                    pub.publish(msg)
                    spin(0.12)

                try:
                    end = time.monotonic() + 10
                    while pub.get_subscription_count() == 0 and time.monotonic() < end:
                        spin(0.05)
                    self.assertGreater(pub.get_subscription_count(), 0)
                    spin(0.3)
                    self.assertFalse(received, 'No publication before the first valid command')
                    send(['b', 'a'], [0.2, -0.4], [0.1, -0.2])
                    self.assertTrue(received)
                    self.assertGreater(received[-1].header.stamp.sec, 0)
                    self.assertTrue(all(len(getattr(received[-1], field)) == len(received[-1].ecat_id)
                                        for field in ("pos_ref", "vel_ref", "tor_ref", "cur_ref",
                                                      "gain_0", "gain_1", "gain_2", "gain_3", "gain_4",
                                                      "fault_ack", "ts", "op_idx_aux", "aux")))
                    self.assertEqual(list(received[-1].ecat_id), [2, 7])
                    self.assertAlmostEqual(received[-1].pos_ref[1], -0.4)
                    self.assertAlmostEqual(received[-1].vel_ref[0], 0.1)
                    self.assertEqual(received[-1].gain_4[0], 10.0)
                    self.assertEqual(received[-1].gain_0[1], 1.0)
                    send(['a'], [0.3])
                    self.assertEqual(len(received[-1].ecat_id), 1)
                    self.assertEqual(received[-1].ecat_id[0], 7)
                    self.assertEqual(received[-1].vel_ref[0], 0.0)
                    send(['a'], [], [0.4])
                    self.assertAlmostEqual(received[-1].vel_ref[0], 0.4)
                    self.assertEqual(received[-1].pos_ref[0], 0.0)
                    self.assertEqual(received[-1].ecat_id[0], 7)
                    # A different upstream producer uses the same public input.
                    count = len(received)
                    other_pub.publish(JointState(name=['b', 'a'], position=[0.6, -0.2], velocity=[0.1, 0.0]))
                    spin(0.12)
                    self.assertGreater(len(received), count)
                    self.assertEqual(list(received[-1].ecat_id), [2, 7])
                    self.assertAlmostEqual(received[-1].pos_ref[0], 0.6)
                    self.assertEqual(received[-1].gain_4[0], 10.0)
                    self.assertEqual(node.count_publishers('tx/motors'), 1)
                    client = node.create_client(SetJointGains, '/advrf/test_robot/set_joint_gains')
                    self.assertTrue(client.wait_for_service(timeout_sec=5.0))

                    def set_gains(joint, values):
                        future = client.call_async(SetJointGains.Request(joint_name=joint, gains=values))
                        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                        self.assertTrue(future.done())
                        return future.result()

                    count = len(received)
                    self.assertTrue(set_gains('a', [11., 12., 13., 14., 15.]).success)
                    spin(0.1)
                    self.assertGreater(len(received), count, 'Last command must be repeated')
                    self.assertAlmostEqual(received[-1].pos_ref[1], -0.2)
                    self.assertEqual(received[-1].gain_0[1], 11.)
                    for joint, values in [
                        ('imu', [0.] * 5), ('unknown', [0.] * 5),
                        ('a', [float('nan')] * 5), ('a', [float('inf')] * 5),
                        ('a', [1e100] * 5),
                    ]:
                        response = set_gains(joint, values)
                        self.assertFalse(response.success)
                        self.assertTrue(response.message)
                    send(['b', 'a'], [0.2, -0.4])
                    self.assertEqual(received[-1].gain_4[0], 10.0)
                    self.assertEqual(
                        [getattr(received[-1], f'gain_{i}')[1] for i in range(5)],
                        [11., 12., 13., 14., 15.])
                    parameters = node.create_client(SetParameters, '/advrf/test_robot/converter_tx_motor_node/set_parameters')
                    self.assertTrue(parameters.wait_for_service(timeout_sec=5.0))

                    def set_mask(values):
                        request = SetParameters.Request(parameters=[
                            Parameter('gain_mask', value=values).to_parameter_msg()])
                        future = parameters.call_async(request)
                        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                        self.assertTrue(future.done())
                        return future.result().results[0]

                    self.assertFalse(set_mask([True, False]).successful)
                    self.assertTrue(set_mask([True, False, True, False, False]).successful)
                    self.assertTrue(set_gains('a', [21., 22., 23., 24., 25.]).success)
                    send(['a', 'b'], [0.1, 0.2])
                    self.assertEqual(
                        [getattr(received[-1], f'gain_{i}')[0] for i in range(5)],
                        [21., 12., 23., 14., 15.])
                    self.assertEqual(received[-1].gain_4[1], 10.)
                    self.assertTrue(set_mask([False] * 5).successful)
                    self.assertTrue(set_gains('a', [0.] * 5).success)
                    send(['a'], [0.1])
                    self.assertEqual(received[-1].gain_0[0], 21.)
                    held = received[-1]
                    for args in [
                        ([], []), (['a'], []), (['a', 'a'], [0.0, 0.0]),
                        (['a', 'imu'], [0.0, 0.0]), (['a'], [float('nan')]),
                        (['a'], [], [2.0]), (['a'], [], [float('nan')]),
                        (['a', 'b'], [], [0.1]), (['a'], [3.0]), (['a'], [0.0], [2.0]),
                        (['a'], [0.0], [0.0, 0.0]), (['a'], [0.0], [], [1.0]),
                    ]:
                        send(*args)
                        self.assertNotEqual(received[-1].header.stamp, held.header.stamp)
                        held.header = received[-1].header
                        self.assertEqual(received[-1], held, f'Invalid batch replaced last command: {args}')
                    count = len(received)
                    spin(0.7)
                    self.assertGreater(len(received), count + 20, 'Must keep publishing beyond bridge timeout')
                    held.header = received[-1].header
                    self.assertEqual(received[-1], held)
                finally:
                    process.terminate()
                    process.wait(timeout=5)
                    node.destroy_node()
                    rclpy.shutdown()
                    log.seek(0)
                    print(log.read())


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
