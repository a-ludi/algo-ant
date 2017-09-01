My Robot
========

The goal of this projects is to create an autonomous robot being able to solve
certain tasks from sketch.


The Robot
---------

The body of the robot has six legs consisting of three parts each. Each leg is
connected to the body by a cardan joint and the parts are connected by hinges.
A head is located to the front of the body. It is connected by a ball joint
and gives housing to a camera and proximity sensors. Attached to the head wih
hinges are two pincers with rubber-coated endings.


Forces
------

- Servo Stall Torque: 18 kg·mm
- Worst Leg Leverage:
	- 15 mm leverage at servo
	- 180/120 transmission → 22.5 mm
	- Total Effective Leg Length: 240 mm
- Effective Force at Foot: ~0.250 kg


Parts Needed
------------

- MMA7455 Chip Tilt Slant Angle Sensor Accelerometer Module DC 5V / 3,3V
- SunFounder PCA9685 16 Channel 12 Bit PWM Servo Driver for Arduino and Raspberry Pi
- KY66 SG90 Micro Servo Motor
