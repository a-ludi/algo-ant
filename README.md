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
	- 180:120 transmission → 22.5 mm
	- Total Effective Leg Length: 240 mm
- Effective Force at Foot: ~0.250 kg


Parts Needed
------------

- MMA7455 Chip Tilt Slant Angle Sensor Accelerometer Module DC 5V / 3,3V
- SunFounder PCA9685 16 Channel 12 Bit PWM Servo Driver for Arduino and Raspberry Pi
- Servos:
	- KY66 SG90 Micro Servo Motor ([data sheet][servo-sg90])
	- Reely Mini-Servo S0009 Analog-Servo (Getriebe-Material: Metall, Stecksystem: JR, [data sheet][servo-s0009-mg-analog])
- MakerHawk Raspi UPS HAT Board für Raspberry Pi 3 Modell B Pi 2B B + A + und 2500mAh Lithium Akku ([data sheet](makerhawk-raspi-ups-hat-board))
- WINGONEER Lithium-Akku-Erweiterungskarten-Netzteil mit Schalter für Himbeer-Pi 3 Modell B, 2 Modell B und 1 Modell B + Banana Pi 3800mAh 5V / 1.8A ([data sheet][wingoneer-lithium-akku-erweiterungskarten-netzteil])


[makerhawk-raspi-ups-hat-board]: ./docs/makerhawk-raspi-ups-hat-board.jpg
[wingoneer-lithium-akku-erweiterungskarten-netzteil]: ./docs/wingoneer-lithium-akku-erweiterungskarten-netzteil.jpg
[servo-s0009-mg-analog]: ./docs/servo-s0009-mg-analog.pdf
[servo-sg90]: ./docs/servo-sg90.pdf