![](../images/banner.png)

## Overview

The *stretch_nav2* package provides the standard ROS 2 navigation stack (Nav2) with its launch files. This package utilizes slam_toolbox and Nav2 to drive Stretch around a mapped space. Running this code will require the robot to be untethered. We recommend stowing the arm while running navigation on the robot.

## Quickstart

The first step is to map the space that the robot will navigate in. The `offline_mapping.launch.py` will enable you to do this. First, run:

```bash
ros2 launch stretch_nav2 offline_mapping.launch.py
```

Rviz will show the robot and the map that is being constructed. With the terminal open, use the joystick (see instructions below for using a keyboard) to teleoperate the robot around. Avoid sharp turns and revisit previously visited spots to form loop closures. In Rviz, once you see a map that has reconstructed the space well enough, open a new terminal and run the following commands to save the map to the `stretch_user/` directory.

```bash
mkdir ${HELLO_FLEET_PATH}/maps
ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/<map_name>
```

**NOTE**: The `<map_name>` does not include an extension. The map_saver node will save two files as `<map_name>.pgm` and `<map_name>.yaml`.

**Tip**: For a quick sanity check, you can inspect the saved map using a pre-installed tool called Eye of Gnome (eog) by running the following command:

```bash
eog ${HELLO_FLEET_PATH}/maps/<map_name>.pgm
```

Next, with `<map_name>.yaml`, we can navigate the robot around the mapped space. Run:

```bash
ros2 launch stretch_nav2 navigation.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

A new RViz window should pop up with a `Startup` button in a menu at the bottom left of the window. Press the `Startup` button to kick-start all navigation related lifecycle nodes. Rviz will show the robot in the previously mapped space, however, it's likely that the robot's location on the map does not match the robot's location in the real space. To correct this, from the top bar of Rviz, use `2D Pose Estimate` to lay an arrow down roughly where the robot is located in the real space. This gives an initial estimate of the robot's location to AMCL, the localization package. AMCL will better localize the robot once we pass the robot a `2D Nav Goal`.

In the top bar of Rviz, use `2D Nav Goal` to lay down an arrow where you'd like the robot to navigate. In the terminal, you'll see Nav2 go through the planning phases and then navigate the robot to the goal. If planning fails, the robot will begin a recovery behavior - spinning around 180 degrees in place or backing up.

**Tip**: If navigation fails or the robot becomes unresponsive to subsequent goals through RViz, you can still teleoperate the robot using the Xbox controller.

### Navigation with Pause/Resume Control

If you need the ability to pause and resume Nav2 velocity commands (e.g., for safety intervention or hybrid manual/autonomous control), use the `navigation_with_mux.launch.py` launch file:

```bash
ros2 launch stretch_nav2 navigation_with_mux.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

This launch file includes a `cmd_vel_mux` node that sits between Nav2 and the robot driver. You can pause/resume Nav2 commands using a service:

```bash
# Pause Nav2 commands (cancels the current goal, saves it for later)
ros2 service call /cmd_vel_mux/enable std_srvs/srv/SetBool "{data: false}"

# Resume Nav2 commands (resends the saved goal)
ros2 service call /cmd_vel_mux/enable std_srvs/srv/SetBool "{data: true}"
```

When you pause navigation:
- The current Nav2 goal is cancelled (preventing "failed to make progress" errors)
- The goal pose is saved
- When you resume, the saved goal is automatically resent to Nav2

You can also start with Nav2 commands disabled:

```bash
ros2 launch stretch_nav2 navigation_with_mux.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml nav2_enabled:=false
```

When Nav2 commands are paused, you can still teleoperate the robot using the joystick controller.

### Room Navigator

The `room_navigator.launch.py` provides a simple way to navigate to predefined room locations using services. It includes the pause/resume functionality from `navigation_with_mux.launch.py`.

```bash
ros2 launch stretch_nav2 room_navigator.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

This launches navigation with two services for predefined locations:

```bash
# Navigate to the kitchen
ros2 service call /go_to_kitchen std_srvs/srv/Trigger

# Navigate to the bedroom
ros2 service call /go_to_bedroom std_srvs/srv/Trigger
```

The services block until navigation completes and return success/failure. You can also pause/resume navigation:

```bash
# Pause navigation
ros2 service call /cmd_vel_mux/enable std_srvs/srv/SetBool "{data: false}"

# Resume navigation (resends the saved goal)
ros2 service call /cmd_vel_mux/enable std_srvs/srv/SetBool "{data: true}"
```

Room locations can be customized by editing the `rooms` dictionary in `stretch_nav2/room_navigator.py`.

### Running in Simulation

To run room navigation in the MuJoCo simulator with driver assistance, use the combined launch file:

```bash
ros2 launch stretch_nav2 sim_room_navigation.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

This single command launches:
- MuJoCo simulation environment
- Multi-goal control (da_sim)
- Driver assistance with shared control (da_core)
- Navigation with cmd_vel mux
- Room navigator

Optional arguments:
- `use_rviz:=true/false` (default: true)
- `use_mujoco_viewer:=true/false` (default: true)
- `shared:=true/false` (default: true) - Enable shared control
- `robocasa_layout:=<layout>` (default: Random)
- `robocasa_style:=<style>` (default: Random)

#### Manual Launch (Alternative)

If you prefer to launch components separately, open three terminals and run:

**Terminal 1 - Launch the simulator:**
```bash
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py use_mujoco_viewer:=true use_rviz:=false mode:=navigation
```

**Terminal 2 - Launch navigation with the mux:**
```bash
ros2 launch stretch_nav2 navigation_with_mux.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml use_sim_time:=true use_rviz:=true teleop_type:=none
```

**Terminal 3 - Launch the room navigator:**
```bash
ros2 launch stretch_nav2 room_navigator.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml use_sim_time:=true use_rviz:=false
```

Then use service calls to start navigation and the `/nav_control` topic to control movement:

```bash
# Navigate to the kitchen
ros2 service call /go_to_kitchen std_srvs/srv/Trigger

# Navigate to the bedroom
ros2 service call /go_to_bedroom std_srvs/srv/Trigger
```

**Note**: The `cmd_vel_mux` only processes commands when the robot is in `room_navigation` mode. The simulator is launched with `mode:=room_navigation` above. On a real robot, switch modes with:

```bash
ros2 service call /switch_to_room_navigation_mode std_srvs/srv/Trigger
```

The `/nav_control` topic accepts a `Float32MultiArray` with 4 values: `[forward, left, right, back]`:
- **forward** (0.0-1.0): Scale Nav2's velocity along the planned path
- **left** (0.0-1.0): Add left turning while moving
- **right** (0.0-1.0): Add right turning while moving
- **back** (0.0-1.0): Override with backward movement (ignores Nav2)
- **All zeros**: Stop and cancel the Nav2 goal

**Important**: Use `--rate` to publish continuously (the mux times out after 0.5s without messages):

```bash
# Move forward along Nav2's planned path at full speed (10 Hz)
ros2 topic pub --rate 10 /nav_control std_msgs/msg/Float32MultiArray '{data: [1.0, 0.0, 0.0, 0.0]}'

# Move forward at half speed
ros2 topic pub --rate 10 /nav_control std_msgs/msg/Float32MultiArray '{data: [0.5, 0.0, 0.0, 0.0]}'

# Turn left while moving forward
ros2 topic pub --rate 10 /nav_control std_msgs/msg/Float32MultiArray '{data: [0.5, 0.5, 0.0, 0.0]}'

# Turn right while moving forward
ros2 topic pub --rate 10 /nav_control std_msgs/msg/Float32MultiArray '{data: [0.5, 0.0, 0.5, 0.0]}'

# Back up (ignores Nav2 path)
ros2 topic pub --rate 10 /nav_control std_msgs/msg/Float32MultiArray '{data: [0.0, 0.0, 0.0, 1.0]}'

# Stop and cancel Nav2 goal (single publish is fine for stop)
ros2 topic pub /nav_control std_msgs/msg/Float32MultiArray '{data: [0.0, 0.0, 0.0, 0.0]}'
```

When you stop publishing (or send all zeros), the Nav2 goal is cancelled. When you start publishing non-zero values again, the saved goal is automatically resent to Nav2.

### Teleop using a Joystick Controller

The launch files expose the launch argument "teleop_type". By default, this argument is set to "joystick", which launches joystick teleop in the terminal with the xbox controller that ships with Stretch RE1. The xbox controller utilizes a dead man's switch safety feature to avoid unintended movement of the robot. This is the switch located on the front left side of the controller marked "LB". Keep this switch pressed and translate or rotate the base using the joystick located on the right side of the xbox controller.

If the xbox controller is not available, the following commands will launch mapping and navigation, respectively, with keyboard teleop:

```bash
ros2 launch stretch_nav2 offline_mapping.launch.py teleop_type:=keyboard
```
or
```bash
ros2 launch stretch_nav2 navigation.launch.py teleop_type:=keyboard map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

## License

For license information, please see the LICENSE files.
