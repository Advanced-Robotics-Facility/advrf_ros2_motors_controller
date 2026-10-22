"""Offscreen UI + simulated ROS endpoints. Never connects to hardware.

Run with ROS_DOMAIN_ID=218 ROS_LOCALHOST_ONLY=1 QT_QPA_PLATFORM=offscreen.
"""
import os
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from advrf_interfaces.action import GoToJointPositions
from python_qt_binding.QtCore import QObject
from python_qt_binding.QtWidgets import QApplication
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool
from advrf_ros2_motors_controller.motor_panel import MotorPanel
from advrf_ros2_motors_controller.panel_config import measured_positions


class PanelTest(unittest.TestCase):
    def test_direct_only_mode(self):
        panel = Mock()
        panel.mode.currentIndex.return_value = 1
        trajectory = Mock()
        trajectory.service_is_ready.return_value = False
        panel.mode_clients = [trajectory]
        panel.endpoint.return_value = '/bench'
        panel.node.get_node_names_and_namespaces.return_value = [('controller', '/bench')]
        with patch('rclpy.action.get_action_server_names_and_types_by_node', return_value=[]) as graph:
            self.assertTrue(MotorPanel.mode_services_available(panel))
            graph.return_value = [('/bench/follow', ['control_msgs/action/FollowJointTrajectory'])]
            self.assertFalse(MotorPanel.mode_services_available(panel))
            trajectory.service_is_ready.return_value = True
            self.assertTrue(MotorPanel.mode_services_available(panel))
            panel.command_publisher.get_subscription_count.return_value = 0
            self.assertFalse(MotorPanel.mode_services_available(panel))

    def test_panel_with_fake_endpoints(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '218')
        app = QApplication.instance() or QApplication([])
        rclpy.init()
        ui = rclpy.create_node('panel_test_ui')
        fake = rclpy.create_node('panel_test_backend')
        namespace = '/panel_test'
        config = Path(__file__).resolve().parents[1] / 'config/joint_limits.yaml'
        calls, goals, commands, switches = [], [], [], []
        mode_state = [True]
        def mode_callback(index):
            def callback(request, response):
                mode_state[index] = request.data
                switches.append((index, request.data))
                response.success = True
                response.message = 'Simulated mode configured'
                return response
            return callback
        fake.create_service(SetBool, namespace + '/set_trajectory_enabled', mode_callback(0))
        direct_subscription = fake.create_subscription(JointState, namespace + '/joint_commands', commands.append, 10)
        def power(command):
            def callback(request, response):
                calls.append(command)
                response.success = True
                response.message = command + ' simulated'
                return response
            return callback
        fake.create_service(Trigger, namespace + '/start_motors', power('start'))
        fake.create_service(Trigger, namespace + '/stop_motors', power('stop'))
        def execute(handle):
            goals.append(handle.request)
            for _ in range(30):
                if handle.is_cancel_requested:
                    handle.canceled()
                    return GoToJointPositions.Result(error_code=0, message='Cancelled')
                time.sleep(0.02)
            handle.succeed()
            return GoToJointPositions.Result(error_code=0, message='Reached')
        server = ActionServer(fake, GoToJointPositions, namespace + '/go_to_joint_positions',
            execute_callback=execute, cancel_callback=lambda goal: CancelResponse.ACCEPT)
        publisher = fake.create_publisher(JointState, namespace + '/rx/joint_states', 10)
        class Context(QObject):
            node = ui
            def argv(self):
                return ['--joint-limits', str(config), '--namespace', namespace]
            def add_widget(self, widget):
                widget.show()
        context = Context()
        panel = MotorPanel(context)
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(ui)
        executor.add_node(fake)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        def wait(predicate, timeout=5):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                app.processEvents()
                if predicate():
                    return
                time.sleep(0.01)
            self.fail('Timed out waiting for UI/ROS state: ' + panel.log.toPlainText())
        names = [name for name, _, _ in panel.joints]
        message = JointState(name=list(reversed(names)), position=[0.2] * len(names))
        timer = fake.create_timer(0.05, lambda: publisher.publish(message))
        try:
            self.assertFalse(panel.go.isEnabled())
            wait(lambda: panel.go.isEnabled() and panel.start.isEnabled())
            self.assertTrue(all(spin.value() == 0.2 for _, spin, _ in panel.rows))
            self.assertEqual(calls, [])
            self.assertEqual(goals, [])
            panel.rows[0][1].setValue(0.3)
            app.processEvents()
            self.assertEqual(goals, [])  # Editing targets never commands motion.
            panel.start.click()
            wait(lambda: calls == ['start'] and not panel.power_busy)
            panel.go.click()
            wait(lambda: not panel.busy)
            self.assertEqual(goals[0].positions[0], 0.3)
            self.assertAlmostEqual(goals[0].velocity_scaling, 0.1)
            panel.sync.click()
            self.assertAlmostEqual(panel.rows[0][1].value(), 0.2)
            panel.go.click()
            wait(lambda: panel.handle is not None)
            panel.stop.click()
            wait(lambda: not panel.busy and not panel.power_busy and not panel.stopping)
            self.assertEqual(calls, ['start', 'stop'])
            self.assertIn('Cancelled', panel.log.toPlainText())
            # Continuous mode only commands checked joints, with a velocity cap.
            panel.mode.setCurrentIndex(1)
            wait(lambda: panel.mode_ready and panel.stream.isEnabled())
            self.assertEqual(mode_state, [False])
            self.assertEqual(switches[-1:], [(0, False)])
            for check in panel.checks[1:]:
                check.setChecked(False)
            panel.rows[0][1].setValue(0.8)
            self.assertEqual(commands, [])
            panel.stream.click()
            self.assertFalse(panel.mode.isEnabled())
            wait(lambda: len(commands) >= 4)
            for command in commands:
                self.assertEqual(command.name, [names[0]])
                self.assertGreater(command.position[0], 0.2)
                self.assertLess(command.position[0], 0.4)
            for previous, current in zip(commands, commands[1:]):
                elapsed = ((current.header.stamp.sec - previous.header.stamp.sec)
                    + (current.header.stamp.nanosec - previous.header.stamp.nanosec) * 1e-9)
                self.assertLessEqual(current.position[0] - previous.position[0], 0.1 * elapsed + 0.002)
            panel.stream.click()
            self.assertFalse(panel.streaming)
            self.assertEqual(calls, ['start', 'stop'])  # Stopping stream does not send motor STOP.
            # Returning to GoTo enables trajectory only after direct streaming stops.
            panel.mode.setCurrentIndex(0)
            wait(lambda: panel.mode_ready and panel.go.isEnabled())
            self.assertEqual(switches[-1:], [(0, True)])
            panel.rows[0][1].setValue(0.25)
            panel.go.click()
            wait(lambda: not panel.busy)
            self.assertEqual(goals[-1].joint_names, [names[0]])
            self.assertEqual(list(goals[-1].positions), [0.25])
            # Feedback loss stops streaming and it never restarts automatically.
            panel.mode.setCurrentIndex(1)
            wait(lambda: panel.mode_ready and panel.stream.isEnabled())
            panel.stream.click()
            wait(lambda: panel.streaming)
            fake.destroy_timer(timer)
            wait(lambda: not panel.streaming)
            self.assertFalse(panel.streaming)
            self.assertFalse(panel.start.isEnabled())
            self.assertTrue(panel.stop.isEnabled())
            with self.assertRaises(ValueError):
                measured_positions(['a', 'a'], [0., 0.], [('a', -1, 1)])
            self.assertIn('feedback missing', panel.motion_status.text())
            # Reconnect the single joint-command consumer before returning to GoTo.
            fake.destroy_subscription(direct_subscription)
            wait(lambda: panel.command_publisher.get_subscription_count() == 0)
            timer = fake.create_timer(0.05, lambda: publisher.publish(message))
            direct_subscription = fake.create_subscription(JointState, namespace + '/joint_commands', commands.append, 10)
            panel.mode.setCurrentIndex(0)
            wait(lambda: panel.mode_ready and panel.go.isEnabled())
            self.assertEqual(panel.motion_status.text(), 'GoTo: ready')
            count = len(goals)
            panel.go.click()
            wait(lambda: not panel.busy)
            self.assertEqual(len(goals), count + 1)
            fake.destroy_timer(timer)
            panel.widget.resize(1100, 600)
            panel.widget.grab().save('/tmp/advrf_motor_panel.png')
        finally:
            panel.shutdown_plugin()
            panel.widget.close()
            executor.shutdown(timeout_sec=3)
            thread.join(timeout=3)
            server.destroy()
            ui.destroy_node()
            fake.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
