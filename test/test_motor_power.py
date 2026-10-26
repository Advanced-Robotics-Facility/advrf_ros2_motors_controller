"""Motor power tests against a fake replCmd, with no physical motor commands."""
import os
os.environ['ROS_DOMAIN_ID'] = '190'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from advrf_ros2_interfaces.srv import ReplCmd, MotorPowerRequest
from std_srvs.srv import Trigger


class MotorPowerTest(unittest.TestCase):
    def test_mock_service(self):
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
    motor_service_timeout: 0.3
''')
            rclpy.init()
            node = rclpy.create_node('motor_power_test', namespace='/power_test')
            requests = []
            reject = [None]
            delay = [False]

            def repl(request, response):
                command = request.request.ctrl_cmd
                requests.append(request.request)
                if delay[0]:
                    time.sleep(0.5)
                response.success = command.board_id != reject[0]
                response.msg = 'mock ACK' if response.success else 'mock NACK'
                return response

            server = node.create_service(ReplCmd, 'replCmd', repl)
            start = node.create_client(Trigger, 'start_motors')
            stop = node.create_client(Trigger, 'stop_motors')
            single = node.create_client(MotorPowerRequest, 'motor_power_request')
            parameters = AsyncParameterClient(node, '/power_test/motor_power_node')
            process = None

            def spin(duration):
                end = time.monotonic() + duration
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.01)

            def wait(future, timeout=4):
                end = time.monotonic() + timeout
                while not future.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.01)
                self.assertTrue(future.done(), 'Service timed out')
                return future.result()

            def launch(auto_start, start_gains=None, mode="impedance", start_joint_gains=None):
                nonlocal process
                process = subprocess.Popen([
                    sys.argv[1], '--ros-args', '--params-file', str(config),
                    '-r', '__ns:=/power_test', '-p', f'control_mode:={mode}',
                    '-p', f'auto_start_motors:={str(auto_start).lower()}',
                    '-p', f'motor_service_timeout:={3.0 if auto_start else 0.3}',
                ] + (['-p', f'start_gains:={start_gains}'] if start_gains is not None else [])
                  + (['-p', f'start_joint_gains:={start_joint_gains}'] if start_joint_gains is not None else []),
                    stdout=log, stderr=log)
                self.assertTrue(start.wait_for_service(timeout_sec=5))
                self.assertTrue(stop.wait_for_service(timeout_sec=5))
                spin(0.2)

            with open(root / 'power.log', 'w+') as log:
                try:
                    launch(False)
                    self.assertEqual(requests, [])
                    self.assertTrue(wait(start.call_async(Trigger.Request())).success)
                    self.assertEqual([r.ctrl_cmd.board_id for r in requests], [7, 2])
                    for request in requests:
                        self.assertEqual(request.type, 2)
                        self.assertEqual(request.ctrl_cmd.type, 11)
                        self.assertEqual(request.ctrl_cmd.value, 212.0)
                        self.assertEqual(request.ctrl_cmd.gains.type, 0)
                        self.assertEqual(request.ctrl_cmd.gains.pos_kp, 0.0)
                        self.assertEqual(request.ctrl_cmd.gains.tor_kp, 0.0)

                    self.assertTrue(single.wait_for_service(timeout_sec=5))
                    requests.clear()
                    on = MotorPowerRequest.Request(action=MotorPowerRequest.Request.ACTION_POWER_ON, joint='a')
                    result = wait(single.call_async(on))
                    self.assertTrue(result.success)
                    self.assertFalse(result.outcome_unknown)
                    self.assertEqual([r.ctrl_cmd.board_id for r in requests], [7])
                    off = MotorPowerRequest.Request(action=MotorPowerRequest.Request.ACTION_POWER_OFF, joint='b')
                    self.assertTrue(wait(single.call_async(off)).success)
                    self.assertEqual(requests[-1].ctrl_cmd.type, 12)
                    requests.clear()
                    all_on = MotorPowerRequest.Request(action=MotorPowerRequest.Request.ACTION_POWER_ON, joint='all')
                    self.assertTrue(wait(single.call_async(all_on)).success)
                    self.assertEqual([r.ctrl_cmd.board_id for r in requests], [7, 2])
                    requests.clear()
                    all_off = MotorPowerRequest.Request(action=MotorPowerRequest.Request.ACTION_POWER_OFF, joint='all')
                    self.assertTrue(wait(single.call_async(all_off)).success)
                    self.assertEqual([r.ctrl_cmd.board_id for r in requests], [7, 2])
                    self.assertTrue(all(r.ctrl_cmd.type == 12 for r in requests))
                    count = len(requests)
                    for invalid in [MotorPowerRequest.Request(action=99, joint='a'),
                                    MotorPowerRequest.Request(action=1, joint='missing')]:
                        result = wait(single.call_async(invalid))
                        self.assertFalse(result.success)
                        self.assertFalse(result.outcome_unknown)
                    self.assertEqual(len(requests), count)
                    delay[0] = True
                    result = wait(single.call_async(on))
                    self.assertFalse(result.success)
                    self.assertTrue(result.outcome_unknown)
                    delay[0] = False
                    spin(0.1)

                    reject[0] = 7
                    requests.clear()
                    self.assertFalse(wait(stop.call_async(Trigger.Request())).success)
                    self.assertEqual([r.ctrl_cmd.board_id for r in requests], [7, 2])
                    self.assertTrue(all(r.ctrl_cmd.type == 12 for r in requests))

                    requests.clear()
                    self.assertFalse(wait(start.call_async(Trigger.Request())).success)
                    self.assertEqual([r.ctrl_cmd.board_id for r in requests], [7])
                    reject[0] = None
                    self.assertTrue(parameters.wait_for_services(timeout_sec=5))
                    results = wait(parameters.set_parameters([
                        Parameter('control_mode', value='position'),
                    ]))
                    self.assertTrue(results.results[0].successful)
                    requests.clear()
                    self.assertTrue(wait(start.call_async(Trigger.Request())).success)
                    self.assertEqual([r.ctrl_cmd.value for r in requests], [59.0, 59.0])

                    # A late reply must be reported as unknown and must not retry START.
                    delay[0] = True
                    requests.clear()
                    result = wait(start.call_async(Trigger.Request()))
                    self.assertFalse(result.success)
                    self.assertIn('outcome unknown', result.message)
                    self.assertEqual(len(requests), 1)
                    delay[0] = False
                    spin(0.1)
                    self.assertTrue(wait(stop.call_async(Trigger.Request())).success)

                    # Only explicit opt-in triggers START at node startup.
                    process.terminate()
                    process.wait(timeout=5)
                    requests.clear()
                    launch(True)
                    deadline = time.monotonic() + 4
                    while len(requests) < 2 and time.monotonic() < deadline:
                        spin(0.05)
                    self.assertEqual([r.ctrl_cmd.board_id for r in requests], [7, 2])
                    self.assertTrue(all(r.ctrl_cmd.type == 11 for r in requests))

                    # Explicit START gains reproduce the validated ReplCmd payload.
                    process.terminate()
                    process.wait(timeout=5)
                    requests.clear()
                    launch(False, [0.1, 0.0, 0.0005, 0.0, 0.0], mode='position')
                    self.assertEqual(requests, [])
                    self.assertTrue(wait(start.call_async(Trigger.Request())).success)
                    for request in requests:
                        self.assertEqual(request.type, 2)
                        self.assertEqual(request.ctrl_cmd.type, 11)
                        self.assertEqual(request.ctrl_cmd.value, 59.0)
                        gains = request.ctrl_cmd.gains
                        self.assertEqual(gains.type, 59)
                        self.assertEqual([gains.pos_kp, gains.pos_kd, gains.tor_kp,
                                          gains.tor_ki, gains.tor_kd],
                                         [0.1, 0.0, 0.0005, 0.0, 0.0])
                    requests.clear()
                    self.assertTrue(wait(stop.call_async(Trigger.Request())).success)
                    self.assertTrue(all(r.ctrl_cmd.type == 12 and r.ctrl_cmd.gains.type == 0
                                        for r in requests))
                    process.terminate()
                    process.wait(timeout=5)
                    requests.clear()
                    launch(False, mode='position', start_joint_gains=[1., 2., 3., 4., 5., 6., 7., 8., 9., 10.])
                    self.assertTrue(wait(start.call_async(Trigger.Request())).success)
                    self.assertEqual([r.ctrl_cmd.board_id for r in requests], [7, 2])
                    self.assertEqual([r.ctrl_cmd.gains.pos_kp for r in requests], [1., 6.])
                    self.assertEqual([r.ctrl_cmd.gains.tor_kd for r in requests], [5., 10.])
                    self.assertTrue(all(r.ctrl_cmd.gains.type == 59 for r in requests))
                    # Profile gains are mode-specific; changing mode requires a restart/profile reload.
                    result = wait(parameters.set_parameters([
                        Parameter('control_mode', value='impedance')]))
                    self.assertFalse(result.results[0].successful)
                finally:
                    if process is not None:
                        process.terminate()
                        process.wait(timeout=5)
                    node.destroy_node()
                    rclpy.shutdown()
                    log.seek(0)
                    print(log.read())


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]])
