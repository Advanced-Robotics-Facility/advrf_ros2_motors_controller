# advrf_ros2_motors_controller

ROS 2 controllers for the ADVRF middleware.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to advrf_ros2_motors_controller
source install/setup.bash
```

## Configuration

```bash
export EC_CFG=/path/to/robot/ec_cfg.yaml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

`EC_CFG` provides the joint configuration, motor ID map and motor-profile folder.
Profiles are selected by their `control_mode`, independently of filenames.
Use the same ROS domain as the middleware.

Without `EC_CFG`, pass `joint_config`, `robot_id_map` and, when needed,
`motor_profiles_dir` explicitly to the controller launch.

## Launch

Start the master and middleware separately. Choose one motion controller:

```bash
# Go to joint positions; includes the trajectory controller.
ros2 launch advrf_ros2_motors_controller go_to.launch.py namespace:=/robot

# Execute trajectories from a client or MoveIt.
ros2 launch advrf_ros2_motors_controller trajectory.launch.py namespace:=/robot

# Receive direct joint position commands.
ros2 launch advrf_ros2_motors_controller joint.launch.py namespace:=/robot
```

Motor power services can run alongside the selected controller:

```bash
ros2 launch advrf_ros2_motors_controller motors.launch.py \
  namespace:=/robot control_mode:=position
```

No motors start automatically.

## Panel

```bash
ros2 launch advrf_ros2_motors_controller panel.launch.py namespace:=/robot
```

The panel connects to running nodes; it does not start controllers.

- **START / STOP motors**: request motor activation or shutdown.
- **Sync targets to feedback**: copy measured positions into the targets.
- **Go to targets**: move the checked joints.
- **Continuous joint commands**: stream selected joint targets at 20 Hz.

For GoTo/direct switching, the robot bringup must start both controllers with
`command_enabled:=false` for the direct controller. The panel disables the other
controller before enabling the selected one. Do not run both standalone motion
launches with their default enabled settings.

## Interfaces

Names are relative to the robot namespace.

| Interface | Purpose |
| --- | --- |
| `rx/joint_states` | Measured joint positions |
| `joint_commands` | Direct commands (`JointState`) |
| `go_to_joint_positions` | GoTo action |
| `advrf_motors_controller/follow_joint_trajectory` | Trajectory action |
| `start_motors`, `stop_motors` | Motor power services |
| `tx/motors` | Commands sent to the middleware |

Closing a node or the panel does not send motor STOP.
