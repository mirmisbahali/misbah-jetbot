# File Structure

```txt
misbah-jetbot/
├─ containers/
│  ├─ jetson.Dockerfile           # arm64, Humble, minimal GPU deps; tuned for Nano
│  ├─ host.Dockerfile             # amd64/arm64, Humble + dev tools for RViz/Nav2
│  └─ entrypoint.sh               # common ROS env setup for both images
├─ compose/
│  ├─ docker-compose.yml          # services: ros-jetson, ros-host; shared volumes/net
│  └─ .env                        # COMPOSE_PROJECT_NAME, ROS_DOMAIN_ID, etc.
├─ .devcontainer/
│  └─ devcontainer.json           # optional: open the repo as a container on laptop
├─ ros2_ws/
│  ├─ src/
│  │  ├─ robot_bringup/           # launch files to start the stack (sensors, Nav2)
│  │  │  ├─ launch/
│  │  │  │  ├─ jetson_bringup.launch.py
│  │  │  │  └─ sim_bringup.launch.py
│  │  │  └─ params/
│  │  │     └─ robot.yaml         # nodes, remaps, QoS, namespaces
│  │  ├─ robot_description/       # URDF/Xacro + meshes
│  │  │  ├─ urdf/
│  │  │  ├─ meshes/
│  │  │  └─ ros2_control.yaml     # controllers + joints
│  │  ├─ robot_navigation/        # Nav2 configs & launch
│  │  │  ├─ launch/nav2.launch.py
│  │  │  └─ config/
│  │  │     ├─ nav2_params.yaml
│  │  │     └─ behavior_trees/
│  │  ├─ robot_sensors/           # LiDAR/IMU/camera drivers or wrappers
│  │  ├─ robot_bringup_msgs/      # custom msg/srv if needed
│  │  └─ robot_util/              # shared libs, e.g., TF helpers
│  ├─ install/                    # colcon (generated)
│  ├─ build/                      # colcon (generated)
│  └─ log/                        # colcon (generated)
├─ maps/
│  ├─ demo_world.yaml
│  └─ demo_world.pgm              # or .png
├─ rviz/
│  └─ nav_debug.rviz              # your RViz profile (used by host container)
├─ sim/
│  ├─ gazebo/
│  │  ├─ world.sdf
│  │  └─ plugins/
│  └─ ignition/                   # if you prefer Ignition Gazebo
├─ network/
│  ├─ fastdds.xml                 # DDS transport/QoS tuning (shared by both)
│  └─ cyclonedds.xml              # alt DDS config (optional)
├─ scripts/
│  ├─ build_host.sh               # colcon build in host container
│  ├─ build_jetson.sh             # colcon build in jetson container
│  ├─ run_host_rviz.sh            # RViz bringup (binds X11/Wayland from laptop)
│  ├─ run_jetson_bringup.sh       # starts sensors/nav on Nano
│  └─ sync_time.sh                # (optional) time sync checks
├─ config/
│  ├─ robot.env.example           # env vars used by compose & entrypoints
│  └─ udev/                       # udev rules for sensors (ids, stable names)
├─ docs/
│  ├─ SETUP.md                    # first-time setup (Jetson + laptop)
│  └─ NAV2_TUNING.md              # costmaps, controllers, BT notes
└─ .gitignore
```


# Run Containers
## On Jetson

```bash
docker run --rm -it \
  --network host --ipc host --privileged \
  -e ROS_DOMAIN_ID=42 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --name ros2_jetson $PWD/ros2_ws/:jetson
```

## On Host (Laptop)

```bash
docker run --rm -it \
  --network host --ipc host \
  -e ROS_DOMAIN_ID=42 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --name ros2_host $PWD/ros2_ws:host
```