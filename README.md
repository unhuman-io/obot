# Robot

## Goal
Design an in/outdoor mobile manipulating robot capable of autonomous navigation with the ability to pick up small items. Precision/repeatability should be comparable to industrial robotics. Joint torque sensing to maintain human safety, and used to protect the robot for collisions.

Demonstrate best practices for robot prototyping. Design using obtainable parts and manufacturing tolerances. Design to be assembled by a person of average skill.

## Why
Our group currently consists of 4 highly experienced electrical/mechanical/controls engineers that have been working on robotic hardware for 10's of years. We've found that robotic hardware robotic hardware requires either significted experience or significant time and iterations to design, tolerance, and control robotic hardware. We've also noticed that there is significant interest in the areas of non-industrial robotics but many groups have difficulty with implementation of the hardware. Difficulty with hardware slows progress and can lead to misleading results when implementing higher level control and AI. At the same time many groups have an interest in design ownership and are not interested in purchasing a platform over which they have little control. With this project we wish to make our hardware and control experience available at no cost in order to help as much as possible a growth in robotics.


## Design targets
- 20 kg device
- Two wheel plus caster base.
- 5 DOF Manipulating arm with some pulling/lifting force, easily overpowered
  - Pan, pitch, pitch, pitch, roll
  - Joint torque sensing
- Wrist has offset roll inline with pitch to maintain close grip to wrist pitch
- Grip options - spike, actuated spike, vacuum cup, parallel jaw, electrostatic, soft gripper
- Strength is 500 g gripper continuous plus 10N peak 
- Base and wrist cameras

## Engineering
- Base
  - Ninebot mini wheels
    - Kt = .76 Nm/A, 20 A peak
    - High resolution encoder - ic haus or mps multipole
  - Ninebot 36V nominal lithium ion battery, 155 Wh
  - Basket for items
  - Camera
  - Computer - Jetson nano
  - Approximate footprint 600 x 500 WxL
- Arm
  - Harmonic drive based design
  - Possible tmotor or robodrive or Rozum?
  - Integrated module electronics
  - Integrated torque sensor
    - Capacitive for easy assembly
  - Motor encoder
    - High resolution MPS on rotor encoder, enables 12bit+3bit
    - Magnetic encoder to reduce the need for precise gap tolerances and cleanliness
  - Output absolute encoder
    - Magnetic encoder to reduce assembly tolerances
    - Resolution need not be as high - but it’s nice if it is 
  - 0.4 x 0.4 m workspace in front of robot
  - Links 0.3 x 0.3 x 0.1 m upper arm x forearm x grip offset
- Grip
  - Modular
- Software
  - Motor driver software as available here: https://github.com/unhuman-io/bort2
  - Mid level software, custom 2 kHz realtime loop. TBD what libraries to use for dynamics
    - Gravity compensation
    - Cartesian impedance control or
    - Inverse kinematics with joint torque limits
  - Higher level software TBD, currently I’m like at the NVidia jetbot as a starting point.
