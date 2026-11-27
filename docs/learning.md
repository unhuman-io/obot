# Teaching the robot tasks: a plan

Goal: everyday tasks, the first one picking clothes up off the floor. This is a plan, not
something that exists yet; the pieces it stands on (the daemon, the client, teleoperation,
recording) do. Versions and numbers below are from a survey in November 2026 and should be
rechecked when each stage starts.

## How it fits the host software

The learning stack is one more client of the shared memory interface, at 10 to 50 Hz:

- observation: joint positions and velocities, base odometry, camera frames;
- action: arm joint position targets (and gripper), base velocity `(v, omega)`.

The daemon already makes this safe to do from Python: targets are filtered and limited,
and a policy that stalls or crashes makes the robot hold and stop within
`command_timeout_ms`.

[LeRobot](https://github.com/huggingface/lerobot) (0.7.0) is the practical framework: its
dataset format (LeRobotDataset v3: Parquet + MP4), recording and training commands
(`lerobot-record`, `lerobot-train`, `lerobot-rollout`) and policies (ACT, Diffusion,
SmolVLA, pi0.5, ...) are what most low-cost robots use. A robot plugs in as a pip package
named `lerobot_robot_<name>` that subclasses `Robot` (`get_observation`, `send_action`),
and a teleoperation device as `lerobot_teleoperator_<name>`. LeRobot needs Python 3.12 or
newer, so on the Jetson it runs in a container (`--ipc=host` to reach the daemon).

## Stages

1. **LeRobot plugin.** `lerobot_robot_obot`: observation and action from `obot.Robot`,
   cameras through LeRobot's own camera classes. Check with `lerobot-teleoperate`,
   `lerobot-record`, `lerobot-replay` against `obot-rtd --sim`.
2. **A way to demonstrate.** The arm has four joints (no shoulder yaw: the base turns), so
   6-DOF hand trackers (phone, VR, UMI) need lossy retargeting. The best fit is a small
   unpowered replica of the arm with encoders (a leader arm) plus a gamepad stick or pedal
   for the base. The gamepad jogging in `obot.teleop` works today but is slow for
   collecting data.
3. **A baseline without learning.** An open-vocabulary detector (Grounding DINO + SAM 2,
   or YOLO-World) on the base camera finds clothing, the depth image gives a grasp point,
   the base turns to face it, and a scripted reach, pinch and lift follows. Cloth forgives
   imprecise grasps, and this gives a success rate to beat with zero demonstrations.
4. **Imitation learning for the grasp.** 50 to 100 demonstrations of approach, grasp and
   drop; `lerobot-train --policy.type=act` on a desktop GPU. ACT runs on an Orin Nano
   (about 18 ms per inference with TensorRT FP16 in a community benchmark). Improve it by
   correcting it while it runs (`lerobot-rollout --strategy.type=dagger`).
5. **A vision-language-action model, if needed.** Fine-tune SmolVLA (about 9 Hz on an
   Orin Nano with TensorRT) or serve pi0.5 from a desktop GPU. LeRobot's remote policy
   server currently refuses robots whose action includes base velocities, so either split
   the base out of the action or write a thin client.

Simulation: MuJoCo loads the URDF directly and is worth having for checking kinematics,
controllers and teleoperation. Cloth in simulation does not transfer well; collect real
data for the cloth part.

## What is missing on the robot

- A gripper (a placeholder in the CAD model) and its joint in the configuration.
- Camera calibration and the transform from camera to arm.
- Homing, so joint angles mean the same thing every day (demonstrations depend on it).
