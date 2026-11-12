# Autonomous-Follow-and-Stop-Robot-in-a-Dynamic-Environment
CET3004 – Robotics, Assignment 2 · University of Sunderland ROS 2 Jazzy · Gazebo Harmonic · Python
A differential-drive robot that uses a single 2D LiDAR to detect a moving target, follow it, stop inside a safety zone, and resume when the target moves away. The system is fully reactive: there is no map, no global planner and no cost map. Everything is decided from the current LiDAR scan.

# 1. Scenario

Mobile robots in shared workspaces (logistics hubs, hospitals, factory floors) are often asked to follow a worker or another robot. The priorities are:

Safety: never close in on a person, and always stop first.
Predictability: humans nearby must be able to anticipate the robot's motion.
Simplicity: behaviour that is easy to verify and explain.

This package is a prototype that addresses those priorities using onboard sensing only.


# 2. Features
Differential-drive robot described in Xacro, driven by ros2_control (DiffDriveController) through cmd_vel.
2D LiDAR defined in the URDF and bridged to ROS 2 as /scan.
Perception node: sector-based nearest-object detection with noise filtering.
Control node: reactive state machine (SEARCH → FOLLOW → STOP) with a proportional speed controller and a safety override.
Gazebo warehouse-style world with static walls, a moving target and an additional dynamic obstacle.
One launch file that starts the world, the robot, the controllers and both nodes.


# 3. System Architecture
 Gazebo Harmonic world
 (walls, moving target, dynamic obstacle)
          │
   gpu_lidar sensor
          │  gz topic
          ▼
    ros_gz_bridge ───────────────► /scan (sensor_msgs/LaserScan)
                                        │
                                        ▼
                               ┌──────────────────┐
                               │  perception_node │
                               └──────────────────┘
                                 │       │        │
                      /target_distance  /target_detected  /safe_to_move
                                 │       │        │
                                 ▼       ▼        ▼
                               ┌──────────────────┐
                               │  follow_stop_node│
                               └──────────────────┘
                                        │
                                     /cmd_vel
                                        ▼
                         diff_drive_controller (ros2_control)
                                        │
                                        ▼
                                  Gazebo robot


# 4. How it works
4.1 Perception
Read the incoming LaserScan and discard invalid values (NaN, inf, and ranges outside range_min/range_max).
Keep only the front sector (default ±30° around the robot's forward axis).
Smooth the data with a short median filter over recent scans to reject single-ray outliers.
Group neighbouring points into clusters using range jumps, which separates the target from walls behind it.
Select the target cluster: the one closest to the previous target position (tracking gate), falling back to the nearest cluster when there is no history.
Publish the distance to the target, whether a target is detected, and whether it is safe to move.

4.2 Transitions:
          target found                 target < safety_distance
SEARCH ───────────────► FOLLOW ─────────────────────────────► STOP
   ▲                       ▲                                    │
   │      target lost      │   target > safety_distance + hyst. │
   └───────────────────────┴────────────────────────────────────┘

4.3 Safety logic
Safety zone (0.8 m): target closer than this → STOP.
Emergency threshold (0.5 m): any object closer than this in the front sector → immediate zero command, regardless of state.
Hysteresis: resuming requires a small margin beyond the safety distance, so the robot does not flicker between STOP and FOLLOW at the boundary.
Safety override: the emergency check runs before the state machine and always wins.
Speed limits: linear and angular velocities are capped, and acceleration is limited for smooth starts and stops.

4.4 Speed controller

v = clamp(Kp · (d − d_safe), 0, v_max)

The robot slows down as it approaches the safety zone and reaches it with near-zero speed. Angular velocity is proportional to the angular offset of the target from the robot's forward axis.

my_diffbot/
├─ package.xml
├─ setup.py
├─ setup.cfg
├─ resource/my_diffbot
├─ my_diffbot/
│  ├─ __init__.py
│  ├─ nodes/
│  │  ├─ perception_node.py      # /scan → /target_distance, /target_detected, /safe_to_move
│  │  └─ follow_stop_node.py     # reactive state machine → /cmd_vel
│  └─ launch/
│     ├─ bringup.launch.py           # world + robot spawn
│     └─ spawn_and_follow.launch.py  # world + robot + controllers + perception + control
├─ urdf/
│  ├─ diffbot.urdf.xacro         # robot, LiDAR, ros2_control
│  └─ diffbot.urdf               # generated from the xacro
├─ config/
│  └─ diff_controllers.yaml      # ros2_control and DiffDriveController parameters
└─ worlds/
   └─ follow_world.sdf           # walls, moving target, dynamic obstacle

# 6. Robot and LiDAR
Base: differential drive with two powered wheels and a caster, defined in Xacro with parameters for dimensions and masses and with inertial properties for each link.
Control: gz_ros2_control hardware interface with joint_state_broadcaster and diff_drive_controller, configured in config/diff_controllers.yaml.
LiDAR: 2D gpu_lidar mounted at the front of the base, horizontal scan, bridged to /scan via ros_gz_bridge.
Known sensor limitations: the sensor sees only one horizontal plane (it is blind above and below it), objects can occlude each other, and noise is added to the scan.

# 7. Simulation world

worlds/follow_world.sdf is a warehouse-style layout:

Static elements: outer walls, aisles and corners.
Moving target: a model that follows a scripted trajectory.
Dynamic obstacle: a second moving model that crosses the robot's path to force perception decisions.

   
# 8. Requirements
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic

sudo apt update
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard \ # it was only be used during initial configuration to make sure the robot was able to move.
  python3-colcon-common-extensions

# 9. Build
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# place the my_diffbot package here
cd ~/ros2_ws
colcon build --packages-select my_diffbot
source install/setup.bash


# 10. Run

Full system (world, robot, controllers, perception and control):

ros2 launch my_diffbot spawn_and_follow.launch.py

World and robot only (for testing with the keyboard):
ros2 launch my_diffbot bringup.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

Useful checks:

ros2 topic echo /scan
ros2 topic echo /target_distance
ros2 topic echo /cmd_vel
ros2 control list_controllers
rqt_graph

# 11. Parameters
Parameter      	     Default	             Description
front_sector_deg	   30	                   Half-width of the front sector in degrees
safety_distance 	   0.8 m	               Distance at which the robot stops
emergency_distance	 0.5 m	               Immediate-stop threshold
hysteresis	         0.1 m	Extra          distance required to resume following
max_linear_speed	   0.5 m/s	             Linear velocity cap
max_angular_speed	   1.0 rad/s	           Angular velocity cap
kp_linear	           0.8	Proportional     gain for speed
kp_angular	         1.5	Proportional     gain for steering

Parameters are set in the launch files and can be overridden at launch time, e.g.:
ros2 launch my_diffbot spawn_and_follow.launch.py safety_distance:=1.0

# 12. Evaluation

The behaviour is assessed in simulation with repeated runs at different target speeds, sudden obstacle insertion and repeated stop–resume cycles. Data is recorded with ros2 bag and analysed for:
minimum distance maintained
time to stop
overshoot of the safety distance
resume delay
collisions

ros2 bag record /scan /target_distance /target_detected /safe_to_move /cmd_vel


#  13 License / academic use

Submitted as individual coursework for CET3004 – Robotics, University of Sunderland.







