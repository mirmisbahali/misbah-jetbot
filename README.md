# File Structure

```txt
misbah-jetbot/
├── README.md
├── .gitignore
├── .devcontainer/
│   ├── host/
│   │   ├── Dockerfile               # amd64/arm64, Humble + dev tools
│   │   └── devcontainer.json        # host development container config
│   └── jetbot/
│       ├── Dockerfile               # arm64, Humble, minimal GPU deps; tuned for Nano
│       └── devcontainer.json        # jetbot development container config
├── pico/
│   ├── README.md
│   └── firmware/
│       └── motor_driver/
│           ├── blink.py             # LED blink test
│           ├── main.py              # main motor driver entry point
│           ├── motor_test.py        # manual motor test script
│           └── uart.py              # UART communication helper
└── ros2_ws/
    ├── host_ws/
    │   └── README.md
    └── jetbot_ws/
        └── src/
            ├── motor_controller/    # ROS2 package: motor control
            │   ├── motor_controller/
            │   │   ├── __init__.py
            │   │   ├── simple_jetbot_controller.py  # Twist subscriber, direct GPIO control
            │   │   └── uart_controller.py           # UART-based motor control node
            │   ├── resource/
            │   │   └── motor_controller
            │   ├── test/
            │   │   ├── test_copyright.py
            │   │   ├── test_flake8.py
            │   │   └── test_pep257.py
            │   ├── package.xml
            │   ├── setup.cfg
            │   └── setup.py
            └── pico_bridge/         # ROS2 package: bridge between ROS2 and Pico
                ├── pico_bridge/
                │   ├── __init__.py
                │   └── bridge.py                    # Twist → UART → Pico bridge node
                ├── resource/
                │   └── pico_bridge
                ├── test/
                │   ├── test_copyright.py
                │   ├── test_flake8.py
                │   └── test_pep257.py
                ├── package.xml
                ├── setup.cfg
                └── setup.py
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
> The `Connect to Host...` in the Remote-SSH extension was not working with the Jetpack 4.6 version. The specific error I was getting was `Remote host does not meet the prerequisites for running vs code server`. 
> I resolved this issue by downgrading VS code on my laptop to version `1.85`.

* Install the **Remote - SSH** extension on your laptop.
* Command Palette → *Remote-SSH: Connect to Host…* → type `jetbot` (uses your `~/.ssh/config`).
* Open your robot repo on JetBot and you’re in.

---

# Dev containers setup
