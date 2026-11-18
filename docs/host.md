# Host software

What runs on the robot's computer, without ROS.

```
   policy / script        operator web page          obot record        any other client
        |                    (obot ui)                    |                    |
        +---------------------+---------------------------+--------------------+
                              |   python/  (obot.client.Robot)
                 commands     v     states
        +-----------------------------------------------+
        |  shared memory  /dev/shm/obot                 |   lock-free; the daemon never waits
        |  command mailbox  <-      -> state ring       |   for a client
        +-----------------------------------------------+
                              ^
                              |   rt/   obot-rtd: one SCHED_FIFO thread on an isolated CPU
                              |   cycle: commands -> limits -> frames out -> statuses in -> state
        +---------------------+---------------------+
        |  SocketCAN (classic, FD)  |  usb_rt (USB) |  built-in simulated actuators
        +---------------------------+---------------+
```

- **`rt/`** is the only code that touches actuators. `obot-rtd` reads a robot description
  (`config/*.toml`), runs the control cycle at a fixed period and owns the robot's mode
  (init, idle, active, fault, estop), the watchdogs and the limits.
- **`python/`** is everything else: the client library, the `obot` command, teleoperation,
  the operator web page, recording.
- **`os/`** builds the robot's operating system image.
- **`config/`** holds one file per robot: `obot3.toml` (Orin, CAN) and `obot1.toml`
  (Jetson Nano, USB).

## Try it without a robot

```sh
sudo apt install cmake ninja-build g++ python3-venv
git submodule update --init
cd rt && cmake --preset release && cmake --build --preset release && ctest --preset release
B=~/.cache/obot-build/release

$B/obot-rtd --config ../config/obot3.toml --sim --no-rt &        # simulated actuators

cd ../python && python3 -m venv .venv && .venv/bin/pip install -e '.[ui,log,dev]'
.venv/bin/obot --config ../config/obot3.toml status
.venv/bin/obot --config ../config/obot3.toml ui --test-camera     # then open http://localhost:8080/
.venv/bin/pytest
```

The same daemon over real CAN sockets, against simulated actuators on virtual CAN:

```sh
sudo modprobe vcan
for i in 0 1 2; do sudo ip link add vcan$i type vcan; sudo ip link set vcan$i mtu 72 up; done
M="--bus arm_a=vcan0 --bus arm_b=vcan1 --bus base=vcan2"
$B/obot-sim-node --config ../config/obot3.toml $M &
$B/obot-rtd --config ../config/obot3.toml $M
```

## The daemon

| | |
|---|---|
| `obot-rtd --config FILE` | run |
| `--check` | validate the configuration and print each CAN bus's load |
| `--sim` | every bus on built-in simulated actuators |
| `--bus NAME=IFACE` | move a bus to another interface |
| `--cpu N`, `--no-rt` | override the control CPU; run without realtime setup |
| `--stats-json FILE`, `--duration SEC` | timing statistics on exit; run for a fixed time |
| `obot-rt-latency --cpu 5 --duration 60` | wake-up latency of a realtime thread on this machine |
| `obot-sim-node --config FILE` | simulated actuators on SocketCAN interfaces |

### Modes

`init` until every actuator has reported, then `idle`. `enable` moves to `active`; a fault
latches `fault` (drives idle) until `clear`. `stop` (emergency stop) latches `estop` from
any mode and is never displaced by another request.

### What the daemon guarantees

- A command is followed only while it is fresh (`command_timeout_ms`). When the client
  goes quiet, position joints brake to a stop and hold, velocity joints ramp to zero,
  torque joints go to damping. This is a warning (`command_stale`), not a fault.
- Position targets are clamped to the joint range and reach the drive through a velocity
  and acceleration limited filter, so a policy at 30 Hz or a typed number cannot step the
  joint. Tracking a target moving at `v` lags by about `v^2 / (2 qdd_max)`.
- Faults: no status from an actuator for `comm_timeout_cycles` cycles, a drive fault, a
  joint beyond its range by `limit_margin`, velocity above 1.5 x `qd_max`, temperature
  above `temperature_max`, a CAN controller error.
- Commands with non-finite numbers or unknown modes are refused as a whole.
- On SIGTERM the drives receive idle commands before the process exits. obot-controller
  drives also fault by themselves when the host stops talking (their host timeout).

### Actuators

| driver | bus | notes |
|---|---|---|
| `obot` | CAN FD, or USB (`interface = "usbrt"`) | obot-controller firmware. Position and impedance use the firmware's STATE controller with gains converted to motor units; velocity uses VELOCITY; torque uses CURRENT. |
| `cubemars_mit` | classic CAN, 1 Mbit/s | CubeMars AK actuators with their stock driver in MIT mode. **Not yet run against hardware.** The value ranges (`cubemars = {...}`) are per model: copy them from the manual. |

### How many joints fit on a CAN bus

Each cycle carries one command and one status per joint, and the daemon refuses to start
if a bus would be more than `max_bus_load` busy in the worst case:

| frames per joint | worst case on the wire | joints per bus at 1 kHz (70 %) |
|---|---|---|
| CubeMars MIT, 2 x 8 bytes classic at 1 Mbit/s | 270 us | 2 |
| obot, 48 + 64 bytes CAN FD at 1 / 5 Mbit/s | 307 us | 2 |

So the 1 kHz configuration in `obot3.toml` uses three buses for six joints. The
alternatives are a 2 ms period, a higher data bit rate, or a firmware change that puts
several joints' commands in one frame.

## The shared memory interface

`rt/include/obot/ipc/layout.h` is the definition; `python/src/obot/layout.py` mirrors it
and a test compares the two. A client maps `/dev/shm/<name>`, checks the header, then:

- reads `RobotState` records from the ring (one per control cycle, the last 4096 kept);
- writes `RobotCommand` to the mailbox under `flock()` on the file.

Anything that can map a file can be a client, including a container started with
`--ipc=host`. Bump `kLayoutVersion` when the structs change; clients refuse a region with
another version.

## Operating the robot

```sh
obot ui                     # cameras, state and gamepad driving at http://<robot>:8080/
obot ui --joystick          # the same, and also accept a controller plugged into the robot
obot teleop                 # only a controller on the robot, no web page
obot status | watch | enable | disable | clear | stop
obot record run.mcap        # every state record; opens in Foxglove, PlotJuggler, Rerun
```

Controller layout (Xbox or any controller the browser reports with the standard mapping):

| | |
|---|---|
| hold **LB** | deadman: nothing moves unless it is held |
| left stick, **RT** | drive and turn; the trigger goes from the configured speed up to the base's limit |
| right stick, D-pad | jog the four arm joints of the selected group; **RB** selects the next group |
| **Start** / **B** | enable / disable |
| **Back** / **Y** | clear faults / emergency stop |

Without a controller the page takes the keyboard: hold Shift as the deadman, W A S D to
drive, arrow keys for the arm, Space to disable.

One operator has control at a time, and the page writes commands only while someone is
operating, so it does not override a policy or script. If the operator's connection drops
the daemon's command timeout stops the robot.

Whoever can reach the port can drive the robot. On a network you do not control use
`obot ui --token SECRET` and open `http://<robot>:8080/?token=SECRET`.

Cameras are listed in the robot configuration:

```toml
[[camera]]
name = "front"
source = "v4l2:/dev/video0"     # or "gst:<pipeline ending in appsink>", or "test"
width = 640
height = 480
fps = 15
```

## Bringing up a real robot

1. Fill in everything marked `VERIFY` in the robot's configuration: ids, directions,
   zero offsets, torque constants, and for CubeMars actuators the frame ranges.
2. `obot-rtd --check --config FILE`.
3. Start with the arm unpowered or supported and low gains. `obot status` must show every
   joint reporting and plausible angles before `obot enable`.
4. Measure the control loop on the target under load:
   `obot-rt-latency --cpu <isolated cpu> --duration 600` while running `stress-ng`.
   The `obot-rtd` log prints the same statistics on exit.

Not done yet: a homing procedure that sets `zero_offset` (the arm has hard stops that
could serve), a gripper joint, gravity compensation.
