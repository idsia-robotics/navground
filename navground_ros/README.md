The navground_ros package
=========================

Topics
----

### Subscribers

- `stop`: `std_msgs::Empty`, stops the navigation.
- `target_pose`: `geometry_msgs::PoseStamped`, sets the target
- `target_point`: `geometry_msgs::PointStamped`, sets the target (ignoring orientation)
- `obstacles`: `navground_msgs::Obstacles`, updates the list of obstacles
- `odometry`: `nav_msgs::Odometry`, updates the robot odometry

In particular `navground_msgs::Obstacles` contains a list of obstacles, each one specified as a vertical cylinder (`radius` and `height`), inflated by a  margin `socialMargin`, with the top face at `topPosition` and moving at `velocity`.
```
float32 radius
float32 socialMargin
geometry_msgs/PointStamped topPosition
geometry_msgs/Vector3Stamped velocity
float32 height
```

`topPosition` and `velocity` must be in a frame connected (in the tf tree) with the robot odometry frame.


### Publishers

- `cmd_vel`: `geometry_msgs::Twist`, the command to avoid the obstacle while moving towards the target
- [optional] `cmd_vel_stamped`:  `geometry_msgs::TwistStamped`, the command to avoid the obstacle while moving towards the target (stamped version in the odometry frame)
- [optional] `visualization_marker/target`: `visualization_msgs::Marker`, the target
- [optional] `visualization_marker/obstacles`: `visualization_msgs::Marker`, the obstacles
- [optional] `visualization_marker/collision`: `visualization_msgs::Marker`, the collision line (HL)
- [optional] `visualization_marker/desired_velocity`: `visualization_msgs::Marker`, the desired velocity

### Actions

- `go_to_target`: `navground_msgs/GoToTarget`, blocking action to reach a target (either a pose or a point). It provides the distance to target as a feedback


Parameters
----

All parameters are exposed as arguments in `navigation.launch` and most of them are configurable at runtime through `dynamic reconfigure`.

### Robot model and kinematics:
  - `~type`: `str`, one of 'TWO_WHEELED' or 'HOLONOMIC'
  - `~radius`: `float > 0`
  - `~axis_length`: `float > 0`, only for TWO_WHEELED
  - `~maximal_speed`: `float > 0`, the robot maximal linear speed
  - [optional] '~maximal_angular_speed": `float > 0`, the robot maximal angular speed
  - [optional]`~maximal_rotation_speed`: 'float > 0', when `~maximal_angular_speed` is undefined, this is used to compute the robot maximal angular speed. If both are undefined, it will use `~maximal_speed`

### Navigation towards a target:
  - `~rate`: `float > 0`, the controller rate
  - `~publish_cmd_stamped`: `bool`, whenever to publish a TwistStamped instead od a Twist.
  - `~point_toward_target`: `bool`, whenever the robot should point towards the target point, only for HOLONOMIC robots
  - `~tol_angle`: `float > 0`, the target angle tolerance
  - `~tol_distance`: `float > 0`, the target distance tolerance

### Navigation Behavior
  - `~behavior`: `str`, one of "HL" (Human-like), "ORCA", or "HRVO"
  - `~optimal_speed`: `float > 0`, the optimal (maximal) speed when there are no obstacles)
  - '~minimal_speed": `float >= 0`, a low-cutoff speed (speeds below are set to zero)
  - `~optimal_angular_speed`: 'float > 0', the optimal (maximal) angular speed
  - `~horizon`: `float > 0`, the planning horizon
  - `~safety_margin`: `float >= 0`, the safety margin to add to the robot radius
  - `~tau`: `float >= 0`, the relaxation-time for speed
  - `~rotation_tau`: `float >= 0`, the relaxation-time for angular speed

### Human-like navigation behavior

  - `~eta`: `float >= 0`, the (minimal) time to keep away from an obstacle.
  - `~aperture`: `float >= 0`, the range of steering angles.
  - `~resolution`: `int > 0`, the number of directions to into which aperture is subdivided.

### ORCA-HRVO navigation behaviors

  - `~time_horizon`: `float > 0`, the planning time-horizon

### 3D navigation (only for HOLONOMIC robots)

  - `~optimal_vertical_speed`: `float > 0`, the optimal (maximal) vertical speed
  - `~tau_z`: `float > 0`, the vertical speed relaxation-time

### Visualization
  - `~drawing`: `bool`, whnever to publish `rviz` markers messages to display the internal state of the navigation algorithm.


Launch
----

```bash
ros2 navground_ros navigation.launch
```
