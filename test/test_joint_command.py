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
from std_srvs.srv import SetBool
from advrf_interfaces.msg import MotorTxPdoVector


class JointCommandTest(unittest.TestCase):
    def test_commands(self):
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
    max_velocity: [1.0, 1.0]
    max_acceleration: [5.0, 5.0]
    gains: [1.0, 2.0, 3.0, 4.0, 5.0]
    joint_gains: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
''')
            rclpy.init()
            node = rclpy.create_node('joint_command_test')
            pub = node.create_publisher(JointState, 'joint_commands', 10)
            received = []
            mode = node.create_client(SetBool, 'set_direct_enabled')
            sub = node.create_subscription(MotorTxPdoVector, 'tx/motors', received.append, 10)
            with open(Path(tmp) / 'node.log', 'w+') as log:
                process = subprocess.Popen([sys.argv[1], '--ros-args', '--params-file', str(config)], stdout=log, stderr=log)

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
                    send(['b', 'a'], [0.2, -0.4], [0.1, -0.2])
                    self.assertTrue(received)
                    self.assertEqual([p.ecat_id for p in received[-1].data], [2, 7])
                    self.assertAlmostEqual(received[-1].data[1].pos_ref, -0.4)
                    self.assertAlmostEqual(received[-1].data[0].vel_ref, 0.1)
                    self.assertEqual(received[-1].data[0].gain_4, 10.0)
                    self.assertEqual(received[-1].data[1].gain_0, 1.0)
                    send(['a'], [0.3])
                    self.assertEqual(len(received[-1].data), 1)
                    self.assertEqual(received[-1].data[0].ecat_id, 7)
                    self.assertEqual(received[-1].data[0].vel_ref, 0.0)
                    self.assertTrue(mode.wait_for_service(timeout_sec=3))
                    future = mode.call_async(SetBool.Request(data=False))
                    rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                    self.assertTrue(future.result().success)
                    spin(0.1)
                    count = len(received)
                    send(['a'], [0.4])
                    self.assertEqual(len(received), count, 'Disabled direct controller published')
                    future = mode.call_async(SetBool.Request(data=True))
                    rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                    self.assertTrue(future.result().success)
                    send(['a'], [0.4])
                    self.assertGreater(len(received), count)
                    count = len(received)
                    for args in [
                        ([], []), (['a'], []), (['a', 'a'], [0.0, 0.0]),
                        (['a', 'imu'], [0.0, 0.0]), (['a'], [float('nan')]),
                        (['a'], [3.0]), (['a'], [0.0], [2.0]),
                        (['a'], [0.0], [0.0, 0.0]), (['a'], [0.0], [], [1.0]),
                    ]:
                        send(*args)
                        self.assertEqual(len(received), count, f'Invalid batch published: {args}')
                    spin(0.2)
                    self.assertEqual(len(received), count, 'Unexpected cyclic republication')
                finally:
                    process.terminate()
                    process.wait(timeout=5)
                    node.destroy_node()
                    rclpy.shutdown()
                    log.seek(0)
                    print(log.read())


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
