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
# SSH setup
## 0) Preconditions

* JetBot and laptop on the **same network**.
* You know the JetBot’s username (in my case, `misbah` or whatever you created) and IP.
---

## 1) Create an SSH key on your laptop (ed25519, recommended)

```bash
ssh-keygen -t ed25519 -a 100 -C "laptop-to-jetbot" -f ~/.ssh/id_jetbot
# Choose a passphrase (recommended)
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_jetbot
```
---

## 2) Copy your public key to JetBot

**Easiest way is to use `ssh-copy-id`:**

in `misbah@10.42.0.116` replace `misbah` with your Jetbot's username and `10.42.0.116` with your Jetbot's IP

```bash
ssh-copy-id -i ~/.ssh/id_jetbot.pub misbah@10.42.0.116
```

**Option B (manual, if ssh-copy-id isn’t available):**
You will need to create `~/.ssh/authorized_keys` file in you Jetbot's file system and copy the public key available on host system `~/.ssh/id_jetbot.pub` to the the `authorized_keys` file in Jetbot.

I tried creating a bash command which will do this automatically, just replace the `~/.ssh/id_misbah.pub` with your .pub key location and `misbah@10.42.0.116` with your Jetbot's username and IP. If the below command fails then please create the file manually.

```bash
cat ~/.ssh/id_misbah.pub | ssh misbah@10.42.0.116 'mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys'
```

---

## 3) Test passwordless login
**replace `misbah@10.42.0.116` with your Jetbot's username and IP**

```bash
ssh misbah@10.42.0.116
# You should be logged in without a password prompt (you may be asked about host key the first time).
```

---

## 4) Make it convenient with an SSH config (on your laptop)

Create `~/.ssh/config`:

```sshconfig
Host jetbot
  HostName 10.42.0.116          # or the JetBot's IP
  User misbah                    # change if your JetBot user is different
  IdentityFile ~/.ssh/id_ed25519
```

Now you can do:

```bash
ssh jetbot
```

## 5) VS Code Remote-SSH (quick start)

> [!IMPORTANT]
> The `Connect to Host...` in the Remote-SSH extension is not working with the Jetpack 4.6 version. The specific error I'm getting is `Remote host does not meet the prerequisites for running vs code server`. I solved this issue by downgrading the VS code version to `1.85` on my Laptop.

* Install the **Remote - SSH** extension on your laptop.
* Command Palette → *Remote-SSH: Connect to Host…* → type `jetbot` (uses your `~/.ssh/config`).
* Open your robot repo on JetBot and you’re in.

---

# Dev containers setup
