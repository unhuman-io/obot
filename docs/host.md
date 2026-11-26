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
                              |   cycle: commands -> limits -> motor commands out -> statuses in -> state
        +---------------------+---------------------+
        |  motor-realtime library: Motor / MotorManager |   all motor communication
        +-----------------------------------------------+
        |  CAN FD (MotorCAN)  |  USB (usb_rt)  |  simulated motors
        +---------------------+----------------+
```

- **`rt/`** is the only code that touches actuators. `obot-rtd` reads a robot description
  (`config/*.toml`), runs the control cycle at a fixed period and owns the robot's mode
  (init, idle, active, fault, estop), the watchdogs and the limits.
- **`python/`** is everything else: the client library, the `obot` command, teleoperation,
  the operator web page, recording.
- **`os/`** builds the robot's operating system image.
- **`config/`** holds one file per robot: `obot3.toml` (Orin, CAN) and `obot1.toml`
  (Jetson Nano, USB), plus `bench_wheel.toml`, a single wheel board on USB for checking
  the daemon against real hardware.

## Try it without a robot

```sh
sudo apt install cmake ninja-build g++ python3-venv
git submodule update --init --recursive
cd rt && cmake --preset release && cmake --build --preset release && ctest --preset release
B=$PWD/build/release

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
| `obot-navd` | keeps the map of the surroundings for the operator page (started by `obot ui`, not by hand) |
| `obot-sim-node --config FILE` | simulated actuators on CAN interfaces (vcan): the firmware's side of the protocol, for testing against the library's CAN code |

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
- Commands with non-finite numbers, unknown modes or a timestamp in the future are refused
  as a whole, except that a stop or disable request they carry is still taken.
- One daemon per shared memory name: a second `obot-rtd` refuses to start.
- On SIGTERM the drives receive idle commands before the process exits. obot-controller
  drives also fault by themselves when the host stops talking (their host timeout).

### Actuators

Every actuator runs the obot-controller firmware and is reached through the
[motor-realtime](https://github.com/unhuman-io/motor-realtime) library, which is built
into `obot-rtd` from the submodule `rt/third_party/motor-realtime`. The daemon contains
no protocol code of its own: it fills in the firmware's `MotorCommand`, hands it to the
library's `Motor` for that actuator, and takes the `MotorStatus` the library read. A
change of transport or wire format is made in motor-realtime, followed by moving the
submodule to that commit.

| bus `interface` | library class | found by |
|---|---|---|
| a CAN FD interface, e.g. `can0` | `MotorCAN` | the joint's `id` on that interface |
| `usbrt` | `Motor` on the `usb_rt` kernel driver | the board's name (`usb_name`, default: the joint's name) |
| `sim` | a simulated `Motor` inside the daemon | - |

What the daemon adds is the joint's view: position and impedance use the firmware's STATE
controller with gains converted to motor units, velocity uses VELOCITY, torque uses
CURRENT, idle is DAMPED (or OPEN).

The realtime loop itself is the library's `MotorThread`, in its fault tolerant mode: each
period it reads every motor's status (waiting until all have answered, at most the receive
deadline), calls the daemon's control cycle, and writes the commands, over CAN as "command,
reply with your status". The daemon configures it for SCHED_FIFO on the isolated CPU. A
motor that is late or gone does not stop the loop; the control cycle sees that its status
is not fresh and faults the robot after `comm_timeout_cycles`. Things to know:

- All motors have to be there when the daemon starts; it exits with a message naming the
  missing one, and systemd starts it again.
- A board that restarts or is unplugged while the daemon runs is a new device to the OS.
  After a second of errors on it the daemon exits, and the restart opens it again.
- USB boards need the `usb_rt` kernel driver 0.8.0 or newer. In 0.7.1, a board that loses
  power while in use leaves the reading thread deadlocked in the kernel, and the daemon
  can then not be stopped without a reboot. `deploy.sh` checks the version.
- The submodule currently points at the branch of
  [motor-realtime #277](https://github.com/unhuman-io/motor-realtime/pull/277); move it to
  `develop` once that is merged.

### How many joints fit on a CAN bus

Each cycle carries one command and one status per joint, and the daemon refuses to start
if a bus would be more than `max_bus_load` busy in the worst case:

| frames per joint | worst case on the wire | joints per bus at 1 kHz (70 %) |
|---|---|---|
| 48 + 64 bytes CAN FD at 1 / 5 Mbit/s | 307 us | 2 |

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
obot viz                    # live plots of every joint and the control loop in the Rerun viewer
obot map-test               # on the robot: it turns and drives by itself, and its map is checked
obot cpu-test               # on the robot: what the software costs in processor time
```

Controller layout (Xbox or any controller the browser reports with the standard mapping):

| | |
|---|---|
| hold **LB** | deadman: nothing moves unless it is held |
| left stick, **RT** | drive and turn; the trigger goes from the configured speed up to the base's limit |
| right stick, D-pad | jog the four arm joints of the selected group; **RB** selects the next group |
| **Start** / **B** | enable / disable |
| **Back** / **Y** | clear faults / emergency stop |

| **X** | switch to Cartesian mode and back: right stick moves the tool forward / back and rolls the wrist, the D-pad raises / lowers and pitches it. Needs `[arm]` in the configuration; moves that would leave the joint limits or the reach are not made. |

Without a controller the page takes the keyboard: hold Shift as the deadman, W A S D to
drive, arrow keys for the arm, Space to disable.

One operator has control at a time and keeps it until they have let go of everything for
a second; the page then stops writing commands, so it does not override a policy or
script, and the daemon holds the arm where it is. Taking over again starts from what the
daemon is holding. Stop, disable, enable and clear-faults buttons work from any page or
controller, whoever is driving. If the operator's connection drops, the daemon's command
timeout stops the robot.

A controller that drops out without disconnecting (a wireless one out of range or out of
battery) keeps reporting its last state until the operating system notices, which can take
seconds. Input that has not changed at all for `controller_timeout_s` (5 s, in `[teleop]`)
while something is held is therefore treated as released; a wired controller or the Xbox
wireless adapter, which report a loss at once, are the better choice. This is a
convenience stop, not a safety function: the robot needs a hardware emergency stop.

Whoever can reach the port can drive the robot. On a network you do not control use
`obot ui --token SECRET` and open `http://<robot>:8080/?token=SECRET`.

The page runs in any browser, on Windows, macOS or Linux; only the robot side needs
Linux. With `[model] urdf = "..."` in the configuration it draws the robot in 3D from its
URDF, posed with the measured joint angles.

Cameras are listed in the robot configuration:

```toml
[[camera]]
name = "front"
source = "v4l2:/dev/video0"     # or "gst:<pipeline ending in a JPEG encoder>", or "test"
width = 640
height = 480
fps = 15
```

A depth camera (Intel RealSense D435) is two entries, `realsense:color` and
`realsense:depth`. They read it through `pyrealsense2` where that is installed. Where it
is not (there is none for the Jetson Nano's Ubuntu 18.04) they use the video devices the
kernel makes of the camera, which needs nothing from Intel, only `v4l2-ctl`
(`sudo apt install v4l-utils`). Depth is shown in false colour, near = warm.

### Plots

The **Plots** section draws any of the robot's signals against time: per joint position,
velocity and torque with their commands, temperature, bus voltage and status age, and for
the robot base speed, odometry and the control loop's own timing. Choose up to eight under
*Signals*; signals with the same unit share a panel, and pointing at a plot reads off all
of them at that instant. The samples come from the daemon's state history, one per control
cycle, at the chosen rate (up to the loop's 1 kHz), not from the page's slower status
updates.

### The 3D view: the robot and what it has seen

With a depth camera the page also draws a map of the surroundings around the robot: every
depth picture is turned into points, placed in the room using the wheel odometry (and the
joint angles, for a camera on the arm), and kept in a grid of 3 cm cells. Cells stay after
the camera has turned away, which is the history; a cell the camera later sees *through*
(the thing was picked up) is taken out again. The orange line is the way the robot drove.

For that the configuration has to say where each camera is:

```toml
[[camera]]
name = "d435"
source = "realsense:color"
xyz = [0.116, 0.0325, 0.143]    # m, in the base: x forward, y left, z up from the floor
rpy = [0.0, 0.0, 0.0]           # rad; rpy[1] > 0 looks down (this one is level)

[[camera]]
name = "d435_depth"
source = "realsense:depth"
xyz = [0.116, 0.0175, 0.143]
rpy = [0.0, 0.0, 0.0]
color = "d435"                  # the camera whose picture colours the points
# link = "wrist_camera_link"    # for a camera on the arm: a link of the robot model
# hfov = 79.6                   # horizontal field of view, degrees (default: D435, 4:3)

[world]                         # optional; these are the defaults
voxel = 0.03
range = 3.5
max_voxels = 250000
correct_odometry = false        # experimental: the cameras correct the wheel odometry, see below
```

**Level to floor** on the page measures height and tilt of a base camera from the floor in
its picture and shows the `xyz` and `rpy` to put in the configuration. Things to know:

- The fields of view are the D435's nominal ones, not the calibration stored in the
  camera, so sizes can be off by a few percent.
- `obot ui --sim-cameras` (with `obot-rtd --sim`) shows all of this without a camera: the
  server draws depth and colour pictures of a small made-up room (`scene:depth`,
  `scene:color`).

### Who keeps the map: obot-navd

The map is kept by `obot-navd`, a C++ program that the page server starts and hands every
depth picture to, with the pose it was taken from. It builds the 3D map, notes where the
floor has been seen to be free and how far ahead the nearest thing is, and answers the
pages (map updates, the map from above) and the navigation (what stands where). It is our
own code, no library: a hash table of voxels and a grid, on the processor. The amount of
data is small (some 19 000 points per picture) and a graphics processor would spend longer
fetching it than computing with it.

The same is implemented in Python (`obot.world`, `obot.ui.worldview`), which is what runs
when `obot-navd` is not installed, and what it is tested against: the same pictures and
poses go to both, and the maps have to come out the same. Planning a way and following it
are still Python (`obot.ui.navigate`); they are the next to move.

### Driving to a place on the map

**Drive to a click**, then a click on the floor in the 3D view or on the **map from
above** under it: the base drives there by itself. The map from above shows what stands
on the floor (dark), the floor seen to be free (light), the margin around things in which
the middle of the base cannot be (shaded), and the robot's footprint to scale. **Look
around** turns it on the spot until the camera has seen all round (one way as far as the
tail has room, then the other way). **Stop**, or touching a controller, ends it at once.

**Map by itself** sends the robot off to map its surroundings without being told where
to go: it looks around, drives to the nearest edge of what it has seen, looks around
there, and so on, until there is no such edge left that it can reach. It stays within the
distance given next to the button (4 m) of where it started, and stops by itself after a
quarter of an hour at the latest. Set the distance so that an open door, the garage door
or the top of a staircase is outside it: the robot does not know that it should not go
there. Edges that are not worth a visit are left out: the unseen inside a thing or right
behind one.

On the map from above, the wheel (or two fingers) zooms about where the pointer is, a drag
moves the map, and a double click shows all of it again.

How it decides:

- It only drives where it has *seen the floor to be free*, and keeps the middle of the
  wheel axis 0.30 m from everything in the map and 0.32 m from floor it has not seen. The
  base is 0.45 m wide at the wheels, so that is 7 cm either side, less a few for how well
  it follows the plan. **A gap has to be about 0.65 m wide to be driven through.** The map
  starts empty, and the camera looks forward only, so at first what it knows is a wedge in
  front of the robot.
- The base is not round: behind the wheels a narrow tail carries the caster, and swings
  out 0.375 m when the base turns on the spot. A plan treats the base as a disc of half
  its width, which is right for driving straight through a gap, and prefers places with
  room to turn. Before every move the base's whole outline is checked against the map for
  the next half second of it: a turn that the tail has no room for is not made. In a
  passage too narrow to turn round in, the robot says "no room to turn here" (it does not
  drive backwards, because the camera looks forwards).
- To a place it has not seen a way to, it plans as if the unseen were free, drives the
  part of that way it has seen, and plans again from there with what the camera saw on
  the way. If there is nothing to drive, it looks around once; then it gives up and says
  why under the view.
- It only ever drives forwards, because that is where the camera looks, and turns on the
  spot. Speeds are 0.25 m/s and 0.7 rad/s at most.
- Whatever the plan says, it does not drive on when the depth camera sees something less
  than 0.22 m in front of the base, when the middle of the depth picture has no readings
  (a depth camera cannot see what is right in front of it), or when no pictures arrive.

What it does not do: it does not see what is lower than 5 cm, glass, or what is above the
camera's view close by (a table top at 30 cm). Its tail swings through 0.375 m when it
turns on the spot, and nothing looks there: only what the map already has is avoided. Keep a hand near a controller.

### What the map remembers, and for how long

There are two maps, and they forget differently.

- The **3D map** (the points in the 3D view) is the detailed one, and the short memory: a
  point goes when the camera has looked through its place a few times, and when the map is
  full (250 000 cells) what has not been looked at for longest goes.
- The **floor plan** (the map from above, also laid on the floor of the 3D view) is the
  simple one, and the long memory. For every 6 cm square of 30 m by 30 m it keeps whether
  the floor has been seen to be free there, and how sure it is that something stands
  there. That goes up quickly with every picture that shows something there and down
  slowly with every picture that shows floor instead: something seen for a few seconds
  takes about 8 seconds of seeing floor in its place to be forgotten, and what is not
  looked at is not forgotten at all. Where the floor plan remembers something that the 3D
  map no longer has, the map from above shows it lighter. Driving keeps clear of both.

Both are saved every two minutes and when the page server stops (`~/.local/state/obot/`),
together with where on the map the robot is and how its wheels stand, which is written
whenever it changes.

When the robot is started again, its odometry starts at zero, and the map is in the
odometry it was made with. What happens then:

- **The wheels stand as they did**: the robot has not driven anywhere, so it is where it
  was, and the map is taken up from there. This is the case after a deploy, after
  `obot-rtd` or the page server were restarted, and after the robot was switched off and
  on without being moved. The map is then checked against what the camera sees: if the
  pictures show things where the map has none (the robot was carried somewhere else), the
  map is put aside and a new one started.
- **The wheels have turned** (the robot was pushed while off): where it is on the old map
  is not known. A new map is started; the old one is kept once, as `...previous`.

The line under the 3D view says which it was. What is not there yet: finding the robot's
place on an old map from what it sees (after it was moved), and joining maps made on
different days.

### The cameras correct the odometry (experimental, off)

`correct_odometry = true` under `[world]` makes the page server lay each depth picture
over a floor plan of what it has seen before and correct the wheels' pose by the
difference. In simulation, with wheels that misreport by 6 % and 12 %, it holds the pose to
a centimetre. **On the one real robot it was tried on, it did worse than the wheels
alone**, which is why it is off: turning on the spot and driving a few metres on a garage
floor, the wheels came back to within 2 mm and 0.1 degree of where they started, while the
correction wandered off by up to 35 cm in between. What made the map smear on that robot
was a wrong `track_width`, not slip. Before trusting it, record a drive and compare.

Two things from those recordings that matter either way:

- `track_width` and `wheel_radius` under `[base]` decide how good the odometry is. Turn
  the robot on the spot: if walls come apart in the map as it turns, `track_width` is off.
- A depth picture reaches the page server about 50 ms after it was taken (Jetson Nano,
  D435 over V4L2), and not always by the same. Where the kernel says when a picture was
  taken (depth over V4L2), that time is used; the picture is still some 27 ms older than
  that. `latency` on a camera overrides either.

### Testing the map on the robot: `obot map-test`

A test to start by hand, on the robot, with the page server running and the robot standing
where it has room to turn and things around it to see (walls, furniture):

```
obot map-test            # asks before it moves the robot; --yes does not
obot map-test --help     # the limits, --no-drive, --turns, --out
```

A map is good if a thing is in the same place in it every time the robot looks. So the
robot makes a map of what is around it four times, each from nothing and by turning once on
the spot, and the maps are laid over each other: how far does one have to be turned and
moved to lie on the other?

| maps | what the robot did between | what it gives |
|---|---|---|
| B on A | three more turns to the left | The base turns more, or less, than the odometry says: `track_width`. The test prints the factor to put it right. |
| C on B | C is made turning right, B turning left | The pictures are older, or newer, than they are taken for: everything is then drawn ahead of where it is, or behind. The camera's `latency`. |
| D on B | a drive to a free place about a metre away, and back | What driving adds: how far the map has moved and turned. |

It also checks that the depth camera delivers and says when its pictures were taken, that
the robot got where it was sent, and how long a map update took. It takes about three
minutes, leaves the robot disabled where it started, and the map it has then is only that
of the last turn. The exit status is 0 (passed), 1 (failed) or 2 (could not be made: no
room to turn, too little to see, a fault). A report and a picture of the four maps over
each other go to `~/.local/state/obot/map-test-<time>/`.

On r1 in a garage, as configured: the base turns 0.01 %, 0.09 % and 0.08 % more than the
odometry says (three runs), the pictures are less than a millisecond off, and after the
drive the map had moved 1 cm and turned 0.2 and 0.7 degrees. With the track width from the
CAD model and the kernel's time taken as it is, which is how it was before this test:
0.87 %, which is 12.6 degrees after four turns, and 23 ms. Both fail. The limits (0.3 %, 15 ms, 12 cm and 3 degrees after the drive)
are from that one robot; the turn of one map against another is good to about 0.15
degrees, which is 0.01 % over four turns and 3 ms.

The same test runs in CI against the simulated robot (`test_navigate.py`), once as it is
and once told that the pictures are 60 ms older than they are, which it has to find.

### What it costs in processor time: `obot cpu-test`

```
obot cpu-test                       # on the robot, page server running; moves nothing
deploy/deploy.sh --cpu-test ...     # the same after a deployment's checks; over a limit fails it
```

It measures how much of one processor the page server, obot-navd, the programs that read
the cameras, and obot-rtd use: for 15 s with nobody on the page, and for 15 s with a page
open as a browser has it (state and four plots over the websocket, every camera's stream,
the 3D map's changes, the map from above), without the browser. Exit status 0, 1 (over a
limit) or 2 (could not be made). `--help` has the limits; the defaults are for a Jetson
Nano with r1's three cameras and are loose, a robot of its own can be held to less
(`--idle-pct 25 --page-pct 70`).

A computer with no cameras can stand in for a robot: `obot ui --replay-cameras room` feeds
the configured cameras (their names, sizes and mounts) with pictures of a made-up room as
they would see it turning on the spot, and `--replay-cameras FILE.npz` with depth pictures
recorded on a robot (`frames`: N x height x width, uint16, millimetres). With obot-rtd on
simulated actuators (`deploy.sh --sim`) and `OBOT_UI_ARGS=--replay-cameras room` in
`/etc/default/obot-ui`, that is a test rig for this: a Jetson Nano set up so gives

| | page server | obot-navd | obot-rtd |
|---|---|---|---|
| nobody on the page | 5 % | 10 % | 2 % |
| a page open, three camera streams | 24 % | 9 % | 2 % |

Where the page server's time goes with a page open: making JPEGs of the camera pictures
(OpenCV, not Python), writing them and the websocket to the network, and the 50 Hz loop
that passes the operator's controller on (6 %). Two things keep it there:

- Pictures are only made into JPEGs while somebody asks for them. Before that was so, the
  first row read 50 %: three cameras' pictures encoded 15 times a second for nobody.
- A depth camera's picture goes to the page grey (nearest brightest) and at most 320
  pixels wide, and the page paints it. Colouring a 640 x 480 picture on the robot cost
  5.5 ms and its JPEG 5.1 ms; grey and half the size it is 1.5 ms in all. Before, the
  second row read 45 %. `/camera/<name>.jpg` and `.mjpg` of a depth camera are therefore
  grey; `depth_colours` in `/api/info` are the colours.

The hardware JPEG encoder of a Jetson (`nvjpegenc`) takes 0.6 ms of processor for a
640 x 480 picture where OpenCV takes 5; fed from Python through a pipe that advantage is
mostly gone (measured: 3 to 5 ms in all). It pays where the camera feeds it without the
picture passing through Python, as the `gst:` source of the CSI camera does.

## Bringing up a real robot

1. Fill in everything marked `VERIFY` in the robot's configuration: ids, zero offsets,
   torque constants. Joint signs are not set there: every joint has direction 1, and which
   way is positive is settled in the motor controller's firmware (phase order and encoder
   direction).
2. `obot-rtd --check --config FILE`.
3. Start with the arm unpowered or supported and low gains. `obot status` must show every
   joint reporting and plausible angles before `obot enable`.
4. Measure the control loop on the target under load:
   `obot-rt-latency --cpu <isolated cpu> --duration 600` while running `stress-ng`.
   The `obot-rtd` log prints the same statistics on exit.

Not done yet: a homing procedure that sets `zero_offset` (the arm has hard stops that
could serve), a gripper joint, gravity compensation.
