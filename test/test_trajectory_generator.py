"""Generate and execute test sweeps using an ideal, isolated motor transport."""
import os
os.environ['ROS_DOMAIN_ID'] = '193'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from advrf_interfaces.msg import MotorTxPdoVector
from sensor_msgs.msg import JointState
from advrf_ros2_motors_controller.test_trajectory import make_trajectory


class GeneratorTest(unittest.TestCase):
    def test_sweep_and_repeat(self):
        trajectory = make_trajectory(['a', 'b'], [0.3, 0.6], 'a', [(-0.1, 1.), (0.1, 1.)], 20.)
        self.assertEqual(len(trajectory.points), 41)
        self.assertAlmostEqual(trajectory.points[0].positions[0], 0.3)
        self.assertAlmostEqual(trajectory.points[20].positions[0], -0.1)
        self.assertAlmostEqual(trajectory.points[-1].positions[0], 0.1)
        self.assertTrue(all(p.positions[1] == 0.6 for p in trajectory.points))
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            os.environ['ROS_LOG_DIR'] = temporary
            mapping = root / 'map.yaml'
            mapping.write_text('joint_map: {1: a, 2: b}')
            config = root / 'joint.yaml'
            config.write_text(f'''/**:
  ros__parameters:
    robot_id_map: {mapping}
    joints: [a, b]
    position_lower: [-1.0, -1.0]
    position_upper: [1.0, 1.0]
    max_velocity: [2.0, 2.0]
    max_acceleration: [10.0, 10.0]
    feedback_timeout: 0.5
    command_durability: volatile
''')
            rclpy.init()
            fake = rclpy.create_node('generator_fake_motors', namespace='/sweep_test')
            positions = [0.3, 0.6]
            received = []
            publisher = fake.create_publisher(JointState, 'rx/joint_states', 10)
            def command(message):
                received.append(message)
                for motor in message.data:
                    positions[motor.ecat_id - 1] = motor.pos_ref
            fake.create_subscription(MotorTxPdoVector, 'tx/motors', command, 10)
            fake.create_timer(0.01, lambda: publisher.publish(JointState(name=['b', 'a'], position=list(reversed(positions)))))
            executor = SingleThreadedExecutor()
            executor.add_node(fake)
            thread = threading.Thread(target=executor.spin)
            thread.start()
            with open(root / 'controller.log', 'w+') as log:
                controller = subprocess.Popen([sys.argv[1], '--ros-args', '--params-file', str(config),
                    '-r', '__ns:=/sweep_test'], stdout=log, stderr=log)
                client = None
                base = [sys.argv[2], '--namespace', '/sweep_test', '--joint-limits', str(config),
                        '--joint', 'a', '--min', '-0.1', '--max', '0.1', '--duration', '1']
                try:
                    result = subprocess.run(base + ['--repeat', '2'], capture_output=True, text=True, timeout=12)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assertIn('Cycle 2 complete', result.stdout + result.stderr)
                    self.assertNotIn('Cycle 3 complete', result.stdout + result.stderr)
                    self.assertAlmostEqual(positions[0], -0.1, places=3)
                    self.assertTrue(received)
                    self.assertTrue(all(abs(m.pos_ref - 0.6) < 1e-6 for msg in received for m in msg.data if m.ecat_id == 2))
                    client = subprocess.Popen(base + ['--repeat', '0'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                    time.sleep(1.5)
                    self.assertIsNone(client.poll(), 'Infinite repeat ended unexpectedly')
                    client.send_signal(signal.SIGINT)
                    output, _ = client.communicate(timeout=5)
                    self.assertEqual(client.returncode, 130, output)
                    self.assertIn('cancelled', output)
                finally:
                    if client and client.poll() is None:
                        client.kill()
                        client.wait()
                    controller.terminate()
                    controller.wait(timeout=5)
                    executor.shutdown(timeout_sec=2)
                    thread.join(timeout=2)
                    fake.destroy_node()
                    rclpy.shutdown()


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
