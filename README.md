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
Controller files are read from `ros2/config/`; the older `config/controller/`
layout is also supported. Use the same ROS domain as the middleware.

Without `EC_CFG`, pass `joint_limits`, `robot_id_map` and, when needed,
`motor_profiles_dir` explicitly to the controller launch.

## Launch

Start the master and middleware separately. Each motion launcher starts the joint-to-motor output it needs:

```bash
# Go to joint positions; includes trajectory generation and joint output.
ros2 launch advrf_ros2_motors_controller go_to.launch.py namespace:=/robot

# Execute trajectories; includes the shared joint output.
ros2 launch advrf_ros2_motors_controller trajectory.launch.py namespace:=/robot

# Direct joint position and/or velocity commands.
ros2 launch advrf_ros2_motors_controller joint.launch.py namespace:=/robot
```

Motor power services can run alongside the selected controller:

```bash
ros2 launch advrf_ros2_motors_controller motors.launch.py \
  namespace:=/robot control_mode:=position
```

No motors start automatically.

`motor_power_node` uses the generic `ReplCmdManager` to send asynchronous REPL
requests. The manager tracks concurrent requests, waits for service availability,
and completes each callback on reply or timeout without retrying. Use it from a
single-threaded executor; the callback bool indicates receipt of a response,
while `response->success` indicates a command ACK.

Power-specific builders (`ReplCmdBuilder::build_motor_start` and
`build_motor_stop`) construct the payloads. The node sequences them for
`start_motors` / `stop_motors`, or selects one joint (e.g. `joint: "a"`) or all joints (`joint: "all"`)
for `motor_power_request` using the
configured control mode and START gains. START stops on the first failure;
STOP attempts every selected motor. `all` is a sequential list of individual
requests, with the next request sent after the previous completes. A missing reply is reported as an unknown outcome.


## Debug launcher

```bash
ros2 launch advrf_ros2_motors_controller debug.launch.py namespace:=/advrf/bench
```

Starts the panel, motor START/STOP services, GoTo, trajectory and direct joint
controllers in one namespace. Trajectory generation starts disabled; the panel enables it for GoTo or keeps
it disabled while publishing continuous commands. Motors start only when START is requested in the panel.
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

For GoTo/direct switching, use `debug.launch.py` or `go_to.launch.py` with the
panel. They start one joint output, shared by trajectory and direct control.
The panel disables trajectory generation before publishing continuous commands.
It stops continuous publication before re-enabling trajectory generation for GoTo.
`set_trajectory_enabled` gates trajectory generation and rejects changes while a
goal is active. There is no source-selection service on the joint converter.

If a joint converter is already running, pass `start_joint_controller:=false` to
`trajectory.launch.py` or `go_to.launch.py`. Run one converter per namespace.

## Command pipeline

```text
GoTo → FollowJointTrajectory → trajectory controller ─┐
                                                    ├→ joint_commands → joint converter → tx/motors → middleware
Panel / external JointState producer ────────────────┘
```

`joint_command_controller_node` has one public `JointState` input, `joint_commands`,
and is the only publisher of `tx/motors`. It validates references, maps joint names
to motor IDs and applies motor gains. It neither selects sources nor generates motion.
Any upstream producer can feed this converter using the same message contract.

GoTo produces trajectories for the trajectory controller, which samples and publishes
`JointState` references on `joint_commands`. The generators require joint limits,
not motor IDs or gains. Trajectory overshoot and acceleration checks remain in the
generator. Commands use volatile durability by default.

The trajectory action accepts any nonempty, unique subset of the configured
joints. This allows one action server to execute MoveIt arm-only and
gripper-only trajectories. Joints omitted from a goal retain their measured or
last commanded position when the controller returns to its hold state. Only one
trajectory goal can execute at a time.

Upstream producers must coordinate: external clients must stop trajectory generation
before direct publication, and stop direct publication before resuming trajectories.
The panel handles this coordination for its own commands. The converter accepts valid
messages from every producer and does not arbitrate between concurrent publishers.

## Interfaces

Names are relative to the robot namespace.

| Interface | Purpose |
| --- | --- |
| `rx/joint_states` | Measured joint positions |
| `joint_commands` | Shared joint references (`JointState`), from any upstream producer |
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
`EC_CFG` → `ros2.folder/config`). The explicit launch argument is
`joint_limits:=/path/to/joint_limits.yaml`; Python clients use `--joint-limits`.
MoveIt joint-limit mappings are converted to controller arrays in URDF joint order.
Position bounds come from `min_position`/`max_position`, falling back to the single
plain `.urdf` file in `ros2/urdf` (variant names containing a dot are excluded).
Velocity and acceleration must be finite, positive and enabled; acceleration must
be supplied in the limits file. All joints listed in that file are included.
ROS parameter YAML with the existing arrays is also supported.
`trajectory_config.yaml` is discovered in `ros2/config`, including when the config
path points to `config/controller`. Explicit trajectory overrides remain supported.

## Robot defaults and overrides

The EC configuration supplies ROS 2 defaults alongside hardware paths:

```yaml
robot:
  robot_id_map_path: '${PWD}/robot_id_map/robot_id_map.yaml'
  robot_control_path: '${PWD}/robot_control/bench_control.yaml'
ros2:
  namespace: /advrf/bench
  folder: '${PWD}/ros2/'
```

`${PWD}` is resolved against the EC configuration file's directory.
All motor-controller launches accept `ec_cfg` (default `$EC_CFG`), `ros2_cfg`
(default `$ROS2_CFG`) and `namespace` (default `ros2.namespace` in that EC file).
Explicit `namespace:=/` selects the root namespace. Explicit file paths override
configuration discovery; `ros2_cfg` overrides the EC file's `ros2.folder/config`.
The legacy `robot.robot_ros2_folder` remains supported when `ros2.folder` is absent.

```bash
# With EC_CFG already selected, namespace is automatic.
ros2 launch advrf_ros2_motors_controller debug.launch.py
# No export required; all included backends and the panel share this selection.
ros2 launch advrf_ros2_motors_controller debug.launch.py ec_cfg:=/path/to/ec_cfg.yaml
# Override the robot's default namespace for this invocation.
ros2 launch advrf_ros2_motors_controller joint.launch.py namespace:=/advrf/test
```

The workspace helper `./launch_debug.sh` sources the installed workspace and
forwards the same arguments, e.g. `./launch_debug.sh ec_cfg:=/path/to/ec_cfg.yaml`.

Direct `joint_commands` messages accept positions, velocities, or both, with one
value per named joint in each supplied array. Empty commands, mismatched sizes,
unknown/duplicate joints, effort and nonfinite/out-of-range references are rejected.
Velocity-only commands are forwarded as `vel_ref` with `pos_ref=0` in the fixed
motor PDO layout. Start the motor in velocity mode with its matching profile;
these messages do not change the motor's control mode. Trajectories still require
positions. The direct controller does not send a zero velocity on command timeout.

## Separate EtherCAT and ROS2 namespaces

`ros2.namespace` in EC_CFG selects the high-level ROS2 namespace (for example
`/advrf/bench`). `ecat_dds.namespace` selects the transport namespace (for example
`/advrf/bench/ecat`). Controller launches remap rx/joint_states, rx/motors,
tx/motors and replCmd to that transport namespace. Joint commands, actions,
controller switching and public power services stay in the ROS2 namespace.
The RQT panel and test_joint_trajectory resolve the same feedback namespace.

All launches accept `ecat_namespace:=/advrf/bench/ecat`; the trajectory script
accepts `--ecat-namespace /advrf/bench/ecat`. Existing EC configurations without
a separate ecat_dds.namespace keep their original topic names.

The canonical DDS section in EC_CFG is `ecat_dds`. The former `dds_bridge`
section is accepted only as a fallback. If both exist, `ecat_dds` takes priority;
keep only the new section in new configuration files.

`ros2.motor_tx_profile` can select a dedicated DDS command gain-slot profile.
It is used by command-converter launchers when no explicit motor_profile or
motor_profiles_dir is passed. Motor power START keeps the original client
profile, avoiding application of wire-slot gains to the START command.
