# AutoFleet-Edge

## Demo

https://github.com/user-attachments/assets/27201d6c-e82b-4532-8220-4e403787ac76

A ROS 2 learning project that drives a simulated differential-drive vehicle to an `(x, y)` target using position feedback.

The current demo uses **ROS 2 Humble**, **Gazebo Fortress**, and **Python**, running in Docker on an **Apple silicon Mac**. It controls the blue vehicle in the upstream `ros_gz_sim_demos` world.

## What works

- Choose `loading_bay` or `drop_off` by name, or supply explicit coordinates.
- Read the vehicle's position and orientation from odometry.
- Turn toward a target, move forward, and slow down near the destination.
- Send stop commands when the vehicle is within **0.10 m** of the target.
- Send stop commands on stale feedback, bridge loss, timeout, or Ctrl+C.
- Run 16 offline tests for target selection, control calculations, and stopping decisions.

This is a single-vehicle coordinate-control exercise. Mapping, obstacle avoidance, natural-language instructions, and fleet coordination are future work.

## How the controller works

1. Subscribe to `/model/vehicle_blue/odometry` (`nav_msgs/msg/Odometry`).
2. Calculate the distance and heading error to the requested target.
3. Publish forward and turning speeds to `/model/vehicle_blue/cmd_vel` (`geometry_msgs/msg/Twist`).
4. Repeat using new position feedback until the target is reached or a stopping condition occurs.

Target coordinates use the incoming odometry frame, observed as `vehicle_blue/odom`. They are not coordinates in a mapped environment. The controller does not control the final heading.

| Setting | Value |
| --- | --- |
| Arrival tolerance | 0.10 m |
| Maximum forward speed | 0.15 m/s |
| Maximum turn rate | 0.60 rad/s |
| Feedback freshness limit | 2 wall-clock seconds |
| Startup wait limit | 20 wall-clock seconds |
| Default motion timeout | 90 wall-clock seconds |

## Files

| File | Purpose |
| --- | --- |
| `autofleet_go_to.py` | Coordinate controller and its offline tests. |
| `ros_action_bridge.py` | Earlier experiment that publishes one `PoseStamped` goal to `/goal_pose`. It does not drive this demo. |
| `.devcontainer/` | Earlier development-container scaffold. The Mac workflow below uses a desktop image directly. |

The coordinate controller accepts targets through command-line arguments. It does not subscribe to `/goal_pose`; the earlier publisher uses a different frame (`map`) and has not been integrated with it.

## Named destinations

| Name | x (m) | y (m) |
| --- | --- | --- |
| `loading_bay` | 0.0 | 1.0 |
| `drop_off` | 1.5 | 1.0 |

These names are labels for coordinates in the vehicle's odometry frame. They do not add buildings to the world, create a map, or provide natural-language understanding. The pairs are stored in the `DESTINATIONS` dictionary near the top of `autofleet_go_to.py`.

List them from the repository directory without starting ROS or moving the vehicle:

```bash
python3 autofleet_go_to.py --list-destinations
```

Choose a name with `--destination`, or supply both `--x` and `--y`. Unknown names, incomplete coordinate pairs, and mixing the two target formats are rejected before ROS starts. The existing coordinate command remains available:

```bash
python3 /home/ubuntu/autofleet_go_to.py --x 0.0 --y 1.0
```

## Run on an Apple silicon Mac

Install and start [Docker Desktop for Mac](https://docs.docker.com/desktop/setup/install/mac-install/). Git is needed to clone this repository. Keep commands for the Mac and Ubuntu terminals separate as indicated below.

### 1. Download the project

In a **Mac Terminal**:

```bash
git clone https://github.com/abhi1228-DAM/autofleet-edge.git
cd autofleet-edge
```

### 2. Create the Ubuntu desktop container

For a **new setup**, run in the same Mac Terminal:

```bash
docker run -d \
  --name autofleet-desktop \
  --platform linux/arm64 \
  -p 127.0.0.1:6080:80 \
  --shm-size=1g \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  ghcr.io/tiryoh/ros2-desktop-vnc:humble
```

If this container already exists, use `docker start autofleet-desktop` instead of creating another one. Keep the existing container to retain its installed packages and copied files.

While still in the cloned repository directory, copy the controller into Ubuntu:

```bash
docker cp autofleet_go_to.py autofleet-desktop:/home/ubuntu/autofleet_go_to.py
```

Install the simulation packages once, using the Mac Terminal:

```bash
docker exec --user ubuntu autofleet-desktop sudo apt-get update
docker exec --user ubuntu autofleet-desktop sudo apt-get install -y ros-humble-ros-gz ros-humble-ros-gz-sim-demos
```

Open [http://127.0.0.1:6080](http://127.0.0.1:6080) in your Mac browser to see the Ubuntu desktop. The image's default password is `ubuntu` if requested.

### 3. Start Gazebo

In a Mac Terminal, enter the container:

```bash
docker exec -it --user ubuntu autofleet-desktop bash
```

When the prompt shows `ubuntu@...`, run:

```bash
source /opt/ros/humble/setup.bash
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch ros_gz_sim_demos diff_drive.launch.py rviz:=false
```

Gazebo should appear in the browser desktop with blue and green vehicles. Keep this terminal open and the simulation unpaused. If this demo is already running, use the existing instance.

### 4. Drive to a target

Open a **second Mac Terminal window** and enter the same container:

```bash
docker exec -it --user ubuntu autofleet-desktop bash
```

In this second Ubuntu shell:

```bash
source /opt/ros/humble/setup.bash
python3 /home/ubuntu/autofleet_go_to.py --destination loading_bay
```

Stop any earlier manual velocity publisher before running the controller. Use one blue-vehicle controller at a time.

For a named target, the program first prints `DESTINATION` with its coordinates. It then prints `WAITING`, `TARGET`, motion updates when needed, and finally `ARRIVED` or `STOPPED`. Watch the vehicle as well as the terminal. Ctrl+C requests cancellation and sends repeated zero-velocity commands.

After the first run finishes, try another target in the open area:

```bash
python3 /home/ubuntu/autofleet_go_to.py --destination drop_off
```

### 5. Check position feedback

In a sourced Ubuntu shell:

```bash
ros2 topic echo /model/vehicle_blue/odometry --once --field pose.pose.position
```

## Observed simulation results

On 2026-10-30, two named-destination commands run manually in the simulation reported `ARRIVED`. Both used the incoming `vehicle_blue/odom` frame.

| Destination | Target x, y (m) | Reported arrival x, y (m) | Reported target error |
| --- | --- | --- | --- |
| `loading_bay` | 0.000, 1.000 | -0.001, 0.965 | 0.035 m (3.5 cm) |
| `drop_off` | 1.500, 1.000 | 1.434, 0.997 | 0.066 m (6.6 cm) |

Terminal excerpts from those runs:

```text
DESTINATION loading_bay -> x=0.00, y=1.00 m
TARGET x=0.00, y=1.00 m in frame 'vehicle_blue/odom'
ARRIVED: x=-0.001, y=0.965; target error=0.035 m. Stopping.

DESTINATION drop_off -> x=1.50, y=1.00 m
TARGET x=1.50, y=1.00 m in frame 'vehicle_blue/odom'
ARRIVED: x=1.434, y=0.997; target error=0.066 m. Stopping.
```

Both reported errors were within the configured **10 cm arrival tolerance**. During the `drop_off` run, the controller first commanded a turn with zero forward speed, then forward motion, and reduced forward speed near the target.

These are individual observations based on the controller's odometry feedback. They do not establish repeatability or accuracy against an independent ground-truth measurement. An earlier coordinate-command run (`--x 0.0 --y 1.0`) reported a 0.037 m (3.7 cm) target error.

## Offline tests

From the repository directory, using Python 3.8 or later:

```bash
python3 autofleet_go_to.py --self-test
```

All 16 tests passed during development. They cover turning direction, angle wrapping, arrival, speed limits, quaternion handling, stopping decisions, and convergence in a simple motion model. They also check that named targets reach the controller with the correct coordinates, coordinate commands still work, invalid arguments are rejected before ROS starts, and listing destinations does not start ROS. ROS and Gazebo are not required for these tests; they do not test live message transport or obstacle handling.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| `container ... is not running` | Run `docker start autofleet-desktop` in the Mac Terminal, then relaunch Gazebo before requesting movement. |
| `file 'diff' was not found` | Use `diff_drive.launch.py` with no space inside the filename. |
| `No such file or directory` for the controller | Repeat `docker cp` from the Mac directory containing `autofleet_go_to.py`. |
| `unrecognized arguments` for `--destination` or `--list-destinations` | Copy the updated script into the container; check the actual downloaded filename, which may have a suffix such as `-2`. Keep the container filename `/home/ubuntu/autofleet_go_to.py`. |
| `invalid choice` for a destination | Use `--list-destinations` and copy a name exactly, including its underscore. |
| No fresh odometry or velocity subscriber | Keep the demo launch process running and Gazebo unpaused. |
| `STALE_ODOMETRY` | Check whether Gazebo is paused, stalled, or running too slowly; resume it before rerunning. |
| Simulation time reset | Restart the controller after resetting the simulation. |
| No Gazebo window | Open the browser desktop and check that the launch shell has `DISPLAY=:1`. |

## Limitations and next steps

- The controller has no obstacle detection or path planning. Use the open simulation area.
- Coordinates come from odometry. There is no map localization or Nav2 integration.
- Stop commands require a running process and working transport; forcibly terminating the process or container prevents cleanup.
- Physical robots, natural-language task parsing, and multiple-robot operation have not been validated.

Planned improvements are repeated arrival measurements, sequences of named destinations, and integration with a navigation stack that can plan around obstacles.

## Credits

This is an AI-assisted learning project. The simulation world and ROS-Gazebo bridge are provided by [ROS-Gazebo's differential-drive demo](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_sim_demos). The Ubuntu desktop image is maintained by [Tiryoh](https://github.com/Tiryoh/docker-ros2-desktop-vnc).

The project-specific work is the coordinate controller, its offline checks, and the integration walkthrough. The goal is to understand and demonstrate ROS 2 feedback control before adding more features.
