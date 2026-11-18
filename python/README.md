# obot (Python)

Operator-side tools for obot robots: the shared memory client, the `obot` command,
gamepad teleoperation, the operator web page and MCAP recording. See `docs/host.md`.

```sh
pip install -e '.[ui,log,dev]'
pytest            # needs obot-rtd built (rt/), or OBOT_RT_BIN_DIR pointing at it
```

```python
from obot import Robot, JointMode

with Robot() as robot:
    robot.enable()
    command = robot.new_command()
    command["joint_mode"][:4] = JointMode.POSITION
    command["q"][0] = 0.3
    while True:                       # keep sending: stale commands are not followed
        robot.send(command)
        state = robot.state()         # numpy record: state["q"], state["mode"], ...
```
