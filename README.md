# advrf_ros2_motors_controller
ros version: jazzy

## Quick Use

```bash
ros2 launch advrf_ros2_bringup controllers.launch.py --show-args
```

For the configured Miranda stack:

```bash
ros2 launch advrf_ros2_miranda_bringup miranda.launch.py
```

## Edit Config

- `robot_config`: motor IDs, profiles, gains and limits.
- `controller_config`: joint list and trajectory action mapping.
- `controllers_config`: node parameters, including update/publish rates, feedback timeouts and tolerances.
- Use explicit YAML file paths when launching the shared controller stack directly.

## Nodes Details

| Node | Purpose |
| --- | --- |
| `joint_trajectory_controller_node` | Executes `control_msgs/action/FollowJointTrajectory` using joint feedback |
| `joint_goal_manager_node` | Converts `advrf_ros2_interfaces/action/GoToJointPositions` goals into trajectories |
| `converter_tx_motor_node` | Converts joint commands into EtherCAT motor command messages |

- Feedback: `rx/joint_states` (`sensor_msgs/msg/JointState`).
- Intermediate commands: `joint_commands` (`sensor_msgs/msg/JointState`).
- Motor commands: `tx/motors` (`advrf_ros2_interfaces/msg/MotorTxPdo`).
- Gain updates: `advrf_ros2_interfaces/srv/SetJointGains`.
- The package also provides an rqt motor panel. Topic and action namespaces are configured by the launch files.

## Dependencies

- `rclcpp`
- `rclcpp_action`
- `control_msgs`
- `trajectory_msgs`
- `sensor_msgs`
- `std_srvs`
- `builtin_interfaces`
- `advrf_ros2_interfaces`
- `rclpy`
- `rcl_interfaces`
- `rqt_gui`
- `rqt_gui_py`
- `python_qt_binding`
- `ament_index_python`

## TroubleShooting

| Symptom | Check |
| --- | --- |
| Goal rejected | Joint names, limits, feedback availability and goal values |
| Trajectory aborts | Feedback timeout and path/goal tolerances |
| No motor command | Controller enabled, joint commands and converter configuration |
| Panel cannot find action | Namespace and running goal manager |
