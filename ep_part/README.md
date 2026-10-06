# FutureDrive EP ROS 2 Arduino bridge

This project runs ROS 2 Humble in Docker on the Raspberry Pi and translates
`geometry_msgs/msg/Twist` messages from `/cmd_vel` into the Arduino serial
commands used by the current firmware.

The current Arduino firmware is stored at
`arduino/futuredrive_4wd/futuredrive_4wd.ino`. Treat this file as the firmware
source of truth when changing motor directions, driving speed, or turn speed.

## Command mapping

| `/cmd_vel` request | Arduino command |
| --- | --- |
| Positive `linear.x` | `W` |
| Negative `linear.x` | `S` |
| Positive `angular.z` | `A` |
| Negative `angular.z` | `D` |
| Zero velocity or timeout | `X` |

The bridge also publishes:

- `/arduino/command` (`std_msgs/msg/String`): commands sent to the Uno
- `/arduino/feedback` (`std_msgs/msg/String`): lines printed by the Uno
- `/ultrasonic/range` (`sensor_msgs/msg/Range`): HC-SR04 distance in metres

The current one-character Arduino protocol cannot represent a backward curve.
A reverse command containing angular velocity is therefore sent as `S`.

## Copy the project to the Raspberry Pi

Run this on the Ubuntu PC:

```bash
scp -r /home/hkit/futuredrive_EP hkit4@192.168.0.49:/home/hkit4/
```

## Run on the Raspberry Pi

Close the Arduino IDE Serial Monitor first because only one program can use
`/dev/ttyACM0` at a time.

```bash
cd /home/hkit4/futuredrive_EP
docker compose up --build -d
docker compose logs -f
```

If the Pi only provides the older Compose command, replace `docker compose`
with `docker-compose`.

## Test from the Ubuntu PC

The PC and container must use the same ROS domain:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
```

Forward test:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

Press `Ctrl+C` to stop publishing. The bridge sends `X` after the command
timeout.

Left-curve test:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.5}}"
```

Inspect bridge topics:

```bash
ros2 topic echo /arduino/command
ros2 topic echo /arduino/feedback
```

## WASDX keyboard driving

Run the custom teleop program on the Ubuntu PC:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
python3 /home/hkit/futuredrive_EP/tools/wasd_teleop.py
```

Use `W` forward, `S` backward, `A` left, `D` right, and `X` or Space to
stop. Use `Q` or `Ctrl+C` to exit. Movement stops automatically when a key is
released for 0.6 seconds.
