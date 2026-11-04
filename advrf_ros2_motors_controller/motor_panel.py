"""Local rqt frontend. Explicit GoTo or continuous joint commands through exclusive controllers."""
import math
import argparse
import os
import time
import yaml

from advrf_ros2_interfaces.action import GoToJointPositions
from python_qt_binding.QtCore import QObject, Qt, QTimer, Signal
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QDoubleSpinBox, QSlider, QGridLayout, QFileDialog, QPlainTextEdit, QComboBox, QCheckBox,
)
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.validate_namespace import validate_namespace
from rqt_gui_py.plugin import Plugin
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool

from .panel_config import load_joints, measured_positions, resolve_panel_config, load_velocities


class Dispatch(QObject):
    run = Signal(object)
    state = Signal(object)


class MotorPanel(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('AdvrfMotors')
        self.node = context.node
        self.closed = False
        self.joints = []
        self.rows = []
        self.clients = []
        self.mode_clients = []
        self.command_publisher = None
        self.streaming = False
        self.mode_pending = False
        self.mode_ready = False
        self.mode_attempted = False
        self.checks = []
        self.subscription = None
        self.action = None
        self.handle = None
        self.busy = False
        self.power_busy = False
        self.stopping = False
        self.cancel_requested = False
        self.received = 0.0
        self.values = None
        self.initialized = False
        self.dispatch = Dispatch()
        self.dispatch.run.connect(lambda fn: fn())
        self.dispatch.state.connect(self.receive)
        parser = argparse.ArgumentParser()
        parser.add_argument('--parameter-node', default='')
        parser.add_argument('--joint-limits', default='')
        parser.add_argument('--namespace', default=None)
        parser.add_argument('--ecat-namespace', default=None)
        args = parser.parse_args(context.argv())
        from .launch_config import resolve_namespace
        args.namespace = resolve_namespace(args.namespace, os.environ.get('EC_CFG', ''))
        self.parameter_node = args.parameter_node
        self.remote_limits = None
        self.parameter_future = None
        self.parameter_client = None
        self.ecat_namespace_override = args.ecat_namespace
        self.widget = QWidget()
        self.widget.setObjectName('AdvrfMotorPanel')
        self.widget.setWindowTitle('ADVRF · Joint control')
        layout = QVBoxLayout(self.widget)
        line = QHBoxLayout()
        self.config = QLineEdit(args.parameter_node or args.joint_limits)
        self.config.setReadOnly(bool(args.parameter_node))
        browse = QPushButton('Browse…')
        browse.clicked.connect(self.browse)
        browse.setEnabled(not bool(args.parameter_node))
        line.addWidget(QLabel('Joint config'))
        line.addWidget(self.config)
        line.addWidget(browse)
        layout.addLayout(line)
        line = QHBoxLayout()
        self.namespace = QLineEdit(args.namespace)
        self.connect_button = QPushButton('Load / connect')
        self.connect_button.clicked.connect(self.connect_robot)
        line.addWidget(QLabel('Robot namespace'))
        line.addWidget(self.namespace)
        line.addWidget(self.connect_button)
        layout.addLayout(line)
        self.status = QLabel('Waiting for configuration')
        layout.addWidget(self.status)
        self.connections = QLabel('Backend: not connected')
        self.connections.setWordWrap(True)
        layout.addWidget(self.connections)
        self.motion_status = QLabel('GoTo: waiting for connection')
        self.motion_status.setWordWrap(True)
        layout.addWidget(self.motion_status)
        line = QHBoxLayout()
        self.mode = QComboBox()
        self.mode.addItems(['GoTo', 'Continuous joint commands'])
        self.mode.currentIndexChanged.connect(self.mode_changed)
        self.apply_mode_button = QPushButton('Apply mode')
        self.apply_mode_button.clicked.connect(self.apply_mode)
        self.stream = QPushButton('Start continuous commands')
        self.stream.clicked.connect(self.toggle_stream)
        line.addWidget(QLabel('Command mode'))
        line.addWidget(self.mode)
        line.addWidget(self.apply_mode_button)
        line.addWidget(self.stream)
        layout.addLayout(line)
        self.grid = QGridLayout()
        layout.addLayout(self.grid)
        line = QHBoxLayout()
        self.velocity = QDoubleSpinBox()
        self.acceleration = QDoubleSpinBox()
        for spin in (self.velocity, self.acceleration):
            spin.setRange(0.01, 1.0)
            spin.setSingleStep(0.05)
            spin.setValue(0.1)
        line.addWidget(QLabel('Velocity scaling'))
        line.addWidget(self.velocity)
        line.addWidget(QLabel('Acceleration scaling'))
        line.addWidget(self.acceleration)
        layout.addLayout(line)
        line = QHBoxLayout()
        self.sync = QPushButton('Sync targets to feedback')
        self.go = QPushButton('Go to targets')
        self.cancel = QPushButton('Cancel motion')
        self.start = QPushButton('START motors')
        self.stop = QPushButton('STOP motors')
        for button, callback in ((self.sync, self.synchronize), (self.go, self.send_goal),
                                 (self.cancel, self.cancel_motion),
                                 (self.start, lambda: self.power(True)),
                                 (self.stop, self.stop_motors)):
            button.clicked.connect(callback)
            line.addWidget(button)
        layout.addLayout(line)
        layout.addWidget(QLabel('GoTo: prepare targets, then click Go. Continuous: start streaming, then move sliders.\n'
                                'STOP is a software middleware request, not an emergency stop.'))
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200)
        layout.addWidget(self.log)
        self.timer = QTimer(self.widget)
        
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.stream_timer = QTimer(self.widget)
        self.stream_timer.setInterval(20)
        self.stream_timer.timeout.connect(self.publish_continuous)
        context.add_widget(self.widget)
        if not args.joint_limits and not args.parameter_node:
            try:
                self.config.setText(resolve_panel_config('',
                    os.environ.get('EC_CFG', ''), os.environ.get('ROS2_CFG', '')))
            except (ValueError, OSError, KeyError, yaml.YAMLError) as error:
                self.note(str(error))
        if self.config.text():
            self.connect_robot()
        self.refresh()

    def note(self, text):
        self.log.appendPlainText(str(text))

    def browse(self):
        path, _ = QFileDialog.getOpenFileName(self.widget, 'Joint configuration', '', 'YAML (*.yaml *.yml)')
        if path:
            self.config.setText(path)

    def endpoint(self, name):
        return '/' + '/'.join(filter(None, [self.robot_namespace.strip('/'), name]))

    def fetch_parameters(self):
        from rcl_interfaces.srv import GetParameters
        if self.parameter_future is not None:
            return
        if self.parameter_client is None:
            self.parameter_client = self.node.create_client(
                GetParameters, self.parameter_node.rstrip('/') + '/get_parameters')
        if not self.parameter_client.service_is_ready():
            self.note('Waiting for controller parameters…')
            QTimer.singleShot(1000, lambda: self.connect_robot() if not self.closed else None)
            return
        request = GetParameters.Request()
        request.names = ['joints', 'position_lower', 'position_upper', 'max_velocity']
        self.parameter_future = self.parameter_client.call_async(request)
        self.parameter_future.add_done_callback(
            lambda future: self.dispatch.run.emit(lambda: self.parameters_received(future)))

    def parameters_received(self, future):
        self.parameter_future = None
        if self.closed:
            return
        try:
            values = future.result().values
            if len(values) != 4 or values[0].type != 9 or any(v.type != 8 for v in values[1:]):
                raise ValueError('Controller returned missing or incorrectly typed limits')
            names = list(values[0].string_array_value)
            lower, upper, velocities = [list(v.double_array_value) for v in values[1:]]
            if (not names or len(set(names)) != len(names) or
                    any(len(v) != len(names) for v in (lower, upper, velocities)) or
                    any(not math.isfinite(x) for v in (lower, upper, velocities) for x in v) or
                    any(lo > hi for lo, hi in zip(lower, upper)) or min(velocities) <= 0):
                raise ValueError('Invalid controller limits')
            self.remote_limits = (names, lower, upper, velocities)
            self.connect_robot()
        except Exception as error:
            self.note(f'Cannot read controller parameters: {error}')

    def connect_robot(self):
        if self.busy or self.power_busy or self.streaming or self.mode_pending:
            return
        try:
            namespace = '/' + self.namespace.text().strip().strip('/')
            validate_namespace(namespace)
            if getattr(self, 'parameter_node', ''):
                if self.remote_limits is None:
                    self.fetch_parameters()
                    return
                names, lower, upper, velocities = self.remote_limits
                joints = list(zip(names, lower, upper))
            else:
                joints = load_joints(self.config.text(), namespace)
                velocities = load_velocities(self.config.text(), namespace, joints)
        except (ValueError, OSError, TypeError, AttributeError, yaml.YAMLError) as error:
            self.note(f'Configuration rejected : {error}')
            return
        self.disconnect_robot()
        self.robot_namespace = namespace
        self.joints = joints
        self.max_velocities = velocities
        self.mode_ready = False
        self.mode_attempted = False
        self.mode.blockSignals(True)
        self.mode.setCurrentIndex(0)
        self.mode.blockSignals(False)
        self.values = None
        self.received = 0.0
        self.initialized = False
        while self.grid.count():
            item = self.grid.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self.rows = []
        self.checks = []
        for col, label in enumerate(('Joint', 'Measured', 'Target', 'Target position', 'Limits', 'Command')):
            self.grid.addWidget(QLabel(label), 0, col)
        for row, (name, lo, hi) in enumerate(joints, 1):
            actual = QLabel('—')
            spin = QDoubleSpinBox()
            spin.setDecimals(6)
            spin.setRange(lo, hi)
            spin.setSingleStep(0.01)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 10000)
            def slide(value, s=spin, low=lo, high=hi):
                s.setValue(low + (high - low) * value / 10000)
            def change(value, s=slider, low=lo, high=hi):
                s.blockSignals(True)
                s.setValue(round(10000 * (value - low) / (high - low)) if high > low else 0)
                s.blockSignals(False)
            slider.valueChanged.connect(slide)
            spin.valueChanged.connect(change)
            change(spin.value())
            for col, widget in enumerate((QLabel(name), actual, spin, slider, QLabel(f'{lo:g} … {hi:g}'))):
                self.grid.addWidget(widget, row, col)
            check = QCheckBox()
            check.setChecked(True)
            check.stateChanged.connect(self.refresh)
            self.grid.addWidget(check, row, 5)
            self.checks.append(check)
            self.rows.append((actual, spin, slider))
        # Capture a generation token so queued messages from an old connection are ignored.
        token = self.token = object()
        def state_callback(message):
            self.dispatch.state.emit((token, message, time.monotonic()))
        from .launch_config import resolve_ecat_namespace
        transport_namespace = resolve_ecat_namespace(
            self.ecat_namespace_override, os.environ.get('EC_CFG', ''), self.robot_namespace)
        feedback_topic = transport_namespace.rstrip('/') + '/rx/joint_states'
        self.subscription = self.node.create_subscription(
            JointState, feedback_topic, state_callback, qos_profile_sensor_data)
        self.action = ActionClient(self.node, GoToJointPositions, self.endpoint('go_to_joint_positions'))
        self.clients = [self.node.create_client(Trigger, self.endpoint(name))
                        for name in ('start_motors', 'stop_motors')]
        self.command_publisher = self.node.create_publisher(JointState, self.endpoint('controllers/joint_commands'), 1)
        self.mode_clients = [self.node.create_client(SetBool, self.endpoint(name))
                             for name in ('set_trajectory_enabled',)]
        self.note(f'Connected to {self.endpoint("")} ; waiting for feedback. '
                  'The first valid motor command starts the motors.')
        self.refresh()

    def receive(self, packet):
        token, message, received = packet
        if self.closed or token is not self.token:
            return
        try:
            values = measured_positions(message.name, message.position, self.joints)
        except (KeyError, ValueError, TypeError):
            return
        self.values, self.received = values, received
        for value, (label, _, _) in zip(values, self.rows):
            label.setText(f'{value:.6f}')
        if not self.initialized and self.fresh() and self.within_limits():
            self.synchronize()
        self.refresh()

    def fresh(self):
        return self.values is not None and time.monotonic() - self.received < 0.5

    def within_limits(self):
        return self.values is not None and all(lo <= v <= hi for v, (_, lo, hi) in zip(self.values, self.joints))

    def synchronize(self):
        if self.busy or self.streaming or not self.fresh() or not self.within_limits():
            return
        for value, (_, spin, _) in zip(self.values, self.rows):
            spin.setValue(value)
        self.initialized = True
        self.note('Targets synchronized to measured positions. No command sent.')

    def selected_indices(self):
        return [i for i, check in enumerate(self.checks) if check.isChecked()]

    def mode_changed(self, *_):
        self.mode_ready = False
        self.mode_attempted = False
        self.refresh()

    def mode_services_available(self):
        if not self.mode_clients:
            return False
        if not self.command_publisher.get_subscription_count():
            return False
        if self.mode_clients[0].service_is_ready():
            return True
        if self.mode.currentIndex() == 0:
            return False
        from rclpy.action import get_action_server_names_and_types_by_node
        prefix = self.endpoint('') + '/'
        return not any(name.startswith(prefix) and any(kind in (
            'control_msgs/action/FollowJointTrajectory',
            'advrf_ros2_interfaces/action/GoToJointPositions') for kind in kinds)
            for node_name, node_namespace in self.node.get_node_names_and_namespaces()
            for name, kinds in get_action_server_names_and_types_by_node(
                self.node, node_name, node_namespace))

    def apply_mode(self):
        if (self.busy or self.streaming or self.power_busy or self.mode_pending
                or not self.mode_clients):
            return
        self.mode_ready = False
        self.mode_attempted = True
        if not self.mode_services_available():
            self.note('Joint converter or trajectory mode service unavailable. Start the required backends.')
            self.refresh()
            return
        self.mode_pending = True
        trajectory_mode = self.mode.currentIndex() == 0
        if self.mode_clients[0].service_is_ready():
            self.note('Enabling trajectory generation…' if trajectory_mode else
                      'Disabling trajectory generation before direct commands…')
            self.later(self.mode_clients[0].call_async(SetBool.Request(data=trajectory_mode)),
                       self.mode_enabled)
        else:
            from types import SimpleNamespace
            self.mode_enabled(SimpleNamespace(success=True,
                message='Direct mode ready; no trajectory server connected.'))
        self.refresh()

    def mode_enabled(self, response):
        self.mode_pending = False
        self.mode_ready = response.success
        self.note(response.message)
        if response.success and self.fresh() and self.within_limits():
            self.synchronize()

    def toggle_stream(self):
        if self.streaming:
            self.stop_stream()
            return
        if not self.stream.isEnabled() or not self.fresh() or not self.within_limits():
            return
        self.stream_indices = self.selected_indices()
        self.references = list(self.values)
        self.last_stream = time.monotonic()
        self.streaming = True
        self.stream_timer.start()
        self.note('Continuous commands started at 20 Hz for selected joints; velocity-limited position references.')
        self.refresh()

    def stop_stream(self, reason='Continuous commands stopped. No motor STOP sent.'):
        if self.streaming:
            self.streaming = False
            self.stream_timer.stop()
            self.note(reason)
        self.refresh()

    def publish_continuous(self):
        if not self.streaming:
            return
        now = time.monotonic()
        dt = now - self.last_stream
        if not self.fresh() or not self.within_limits() or dt > 0.25:
            self.stop_stream('Continuous commands stopped: stale/invalid feedback or delayed UI timer. Restart explicitly.')
            return
        if not self.command_publisher.get_subscription_count():
            self.stop_stream('Continuous commands stopped: no joint_commands subscriber.')
            return
        self.last_stream = now
        for i in self.stream_indices:
            target = self.rows[i][1].value()
            step = self.max_velocities[i] * self.velocity.value() * dt
            self.references[i] += max(-step, min(step, target - self.references[i]))
        message = JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = [self.joints[i][0] for i in self.stream_indices]
        message.position = [self.references[i] for i in self.stream_indices]
        self.command_publisher.publish(message)

    def refresh(self, *_):
        connected = self.action is not None
        fresh = self.fresh()
        ready = connected and fresh and self.within_limits() and self.initialized
        if self.streaming and (not fresh or not self.within_limits()):
            self.streaming = False
            self.stream_timer.stop()
            self.note('Continuous commands stopped: feedback missing, stale or outside limits. Restart explicitly.')
        idle = not self.busy and not self.power_busy and not self.stopping and not self.mode_pending
        available = self.mode_services_available()
        if not available:
            self.mode_ready = False
        if connected and idle and not self.streaming and available and not self.mode_attempted:
            self.apply_mode()
            return
        selected = bool(self.selected_indices())
        direct = self.mode.currentIndex() == 1
        self.go.setEnabled(bool(ready and idle and selected and self.mode_ready and not direct
                                and self.action.server_is_ready()))
        self.sync.setEnabled(bool(ready and idle and not self.streaming))
        self.start.setEnabled(bool(ready and idle and not self.streaming and self.clients[0].service_is_ready()))
        self.stop.setEnabled(bool(connected and not self.power_busy and not self.stopping
                                  and self.clients[1].service_is_ready()))
        self.cancel.setEnabled(self.busy and not self.cancel_requested)
        self.stream.setText('Stop continuous commands' if self.streaming else 'Start continuous commands')
        self.stream.setEnabled(bool(self.streaming or (ready and idle and selected and direct
            and self.mode_ready and self.command_publisher.get_subscription_count())))
        self.mode.setEnabled(idle and not self.streaming)
        self.apply_mode_button.setEnabled(bool(connected and idle and not self.streaming))
        self.connect_button.setEnabled(idle and not self.streaming)
        self.config.setEnabled(idle and not self.streaming)
        self.namespace.setEnabled(idle and not self.streaming)
        self.acceleration.setEnabled(not direct and idle)
        for check in self.checks:
            check.setEnabled(idle and not self.streaming)
        for i, (_, spin, slider) in enumerate(self.rows):
            editable = bool(ready and idle and self.checks[i].isChecked())
            spin.setEnabled(editable)
            slider.setEnabled(editable)
        backend = {
            'GoTo action': bool(connected and self.action.server_is_ready()),
            'Direct input': bool(self.command_publisher and self.command_publisher.get_subscription_count()),
            'Selected mode service(s)': available,
            'START service': bool(self.clients and self.clients[0].service_is_ready()),
            'STOP service': bool(self.clients and self.clients[1].service_is_ready()),
        }
        self.connections.setText('Backend · ' + ' | '.join(
            f'{name}: {"ready" if found else "missing"}' for name, found in backend.items()))
        blockers = []
        if not fresh:
            blockers.append('feedback missing or older than 0.5 s')
        elif not self.within_limits():
            blockers.append('measured positions outside configured limits')
        if not self.initialized:
            blockers.append('targets not synchronized')
        if not selected:
            blockers.append('no joints checked')
        if direct:
            blockers.append('Continuous mode selected')
        if not self.mode_ready:
            blockers.append('command mode not confirmed; check mode services / Apply mode')
        if not connected or not self.action.server_is_ready():
            blockers.append('GoTo action server missing')
        if not idle:
            blockers.append('motion, mode switch or power operation in progress')
        reason = 'GoTo: ready' if not blockers else 'GoTo unavailable: ' + '; '.join(blockers)
        self.motion_status.setText(reason)
        self.go.setToolTip(reason)
        self.stream.setToolTip('Requires fresh feedback, selected joints, direct mode and a joint_commands subscriber.')
        self.start.setToolTip('Calls start_motors on the running power node; does not launch a process.')
        self.stop.setToolTip('Calls stop_motors on the running power node; does not terminate processes.')
        state = 'Feedback valid' if fresh else 'Feedback missing or stale'
        if fresh and not self.within_limits():
            state = 'Measured position outside limits: commands disabled'
        self.status.setText(state + (' · motion in progress' if self.busy else '')
                            + (' · streaming joint_commands' if self.streaming else '')
                            + (' · switching mode' if self.mode_pending else '')
                            + (' · power request pending' if self.power_busy else ''))

    def later(self, future, callback):
        def done(result):
            if not self.closed:
                self.dispatch.run.emit(lambda: self.complete(result, callback))
        future.add_done_callback(done)

    def complete(self, future, callback):
        if self.closed:
            return
        try:
            callback(future.result())
        except Exception as error:
            # Do not assume an in-flight command failed or clear its ownership.
            self.note(f'ROS error, verify robot state : {error}')
        self.refresh()

    def send_goal(self):
        if not self.go.isEnabled() or not self.fresh():
            return
        goal = GoToJointPositions.Goal()
        selected = self.selected_indices()
        goal.joint_names = [self.joints[i][0] for i in selected]
        goal.positions = [self.rows[i][1].value() for i in selected]
        goal.velocity_scaling = self.velocity.value()
        goal.acceleration_scaling = self.acceleration.value()
        self.busy = True
        self.cancel_requested = False
        self.refresh()
        self.note('Sending GoTo…')
        self.later(self.action.send_goal_async(goal), self.goal_response)

    def goal_response(self, handle):
        if not handle.accepted:
            self.busy = False
            self.note('GoTo rejected.')
            self.after_motion()
            return
        self.handle = handle
        self.note('GoTo accepted.')
        self.later(handle.get_result_async(), self.goal_result)
        if self.cancel_requested:
            self.request_cancel()

    def goal_result(self, response):
        self.busy = False
        self.handle = None
        self.note(f'GoTo finished : status={response.status}, code={response.result.error_code} · '
                  f'{response.result.message}')
        self.after_motion()

    def cancel_motion(self):
        if self.busy:
            self.cancel_requested = True
            if self.handle:
                self.request_cancel()
            self.refresh()

    def request_cancel(self):
        self.later(self.handle.cancel_goal_async(), self.cancel_response)

    def cancel_response(self, response):
        self.note('Cancellation accepted; waiting for completion.' if response.goals_canceling
                  else 'Cancellation rejected; waiting for motion result.')
        if not response.goals_canceling:
            self.cancel_requested = False

    def stop_motors(self):
        if not self.stop.isEnabled():
            return
        self.stop_stream()
        self.stopping = True
        if self.busy:
            self.note('STOP requested: cancelling motion before stopping motors.')
            self.cancel_motion()
        else:
            self.power(False)
        self.refresh()

    def after_motion(self):
        if self.stopping:
            self.power(False)

    def power(self, start):
        if self.power_busy or (start and not self.start.isEnabled()):
            return
        client = self.clients[0 if start else 1]
        if not client.service_is_ready():
            self.note('Motor service unavailable; motor state unknown.')
            self.stopping = False
            return
        self.power_busy = True
        self.note(('START' if start else 'STOP') + ' requested…')
        self.refresh()
        self.later(client.call_async(Trigger.Request()), self.power_result)

    def power_result(self, response):
        self.power_busy = False
        self.stopping = False
        self.note(f'Power : success={response.success} · {response.message}')

    def disconnect_robot(self):
        for client in self.mode_clients:
            self.node.destroy_client(client)
        self.mode_clients = []
        if self.command_publisher:
            self.node.destroy_publisher(self.command_publisher)
            self.command_publisher = None
        if self.subscription:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None
        if self.action:
            self.action.destroy()
            self.action = None
        for client in self.clients:
            self.node.destroy_client(client)
        self.clients = []

    def shutdown_plugin(self):
        self.closed = True
        self.timer.stop()
        self.stream_timer.stop()
        self.streaming = False
        if self.parameter_client is not None:
            if self.parameter_future is not None:
                self.parameter_future.cancel()
            self.node.destroy_client(self.parameter_client)
            self.parameter_client = None
        # Closing a panel is not a motor power command.
        if self.handle and self.busy:
            self.handle.cancel_goal_async()
        self.disconnect_robot()
