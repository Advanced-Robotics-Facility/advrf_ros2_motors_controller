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
Controller files are read from `ros2/config/controller/` (the older `ros2/config/`
layout is also supported). Use the same ROS domain as the middleware.

Without `EC_CFG`, pass `joint_limits`, `robot_id_map` and, when needed,
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

## Debug launcher

```bash
ros2 launch advrf_ros2_motors_controller debug.launch.py namespace:=/advrf/bench
```

Starts the panel, motor START/STOP services, GoTo, trajectory and direct joint
controllers in one namespace. Both motion controllers start disabled; the panel
activates the selected mode. Motors start only when START is requested in the panel.
Configuration comes from `EC_CFG` / `ROS2_CFG`, with optional `joint_limits`,
`robot_id_map`, `trajectory_config`, `control_mode`, `motor_profile`,
`motor_profiles_dir` and `start_gains` launch arguments.
Start the master and middleware separately. Stop existing standalone panel,
motors, GoTo, trajectory and joint launches before using this combined launcher.

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

## Test trajectory generator

With the trajectory controller running and motors already enabled:

```bash
ros2 run advrf_ros2_motors_controller test_joint_trajectory \
  --namespace /robot --joint joint_a --min -0.2 --max 0.2 \
  --duration 5 --repeat 3
```

Reads the initial feedback, approaches min, then repeats **min → max → min**.
`--duration` is the time for each direction. `--repeat 0` loops indefinitely.
Points are spaced at 20 Hz; each trajectory is sent once through the existing
FollowJointTrajectory action. Other joints hold their measured positions.

Configuration comes from EC_CFG / ROS2_CFG, or `--joint-limits /path/to/joint_limits.yaml`.
Optional: `--approach-duration 5`, `--rate 20`. Durations that exceed configured
velocity/acceleration limits are rejected. Ctrl-C requests trajectory cancellation;
it does not send motor STOP. This client does not START motors or switch controllers.

Joint configuration uses `joint_limits.yaml` in `ROS2_CFG` (or
`EC_CFG` → `robot.robot_ros2_folder/config`). The explicit launch argument is
`joint_limits:=/path/to/joint_limits.yaml`; Python clients use `--joint-limits`.
MoveIt joint-limit mappings are converted to controller arrays in URDF joint order.
Position bounds come from `min_position`/`max_position`, falling back to the single
plain `.urdf` file in `ros2/urdf` (variant names containing a dot are excluded).
Velocity and acceleration must be finite, positive and enabled; acceleration must
be supplied in the limits file. All joints listed in that file are included.
ROS parameter YAML with the existing arrays is also supported.
`trajectory_config.yaml` is discovered in `ros2/config`, including when the config
path points to `config/controller`. Explicit trajectory overrides remain supported.
