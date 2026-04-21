# Multi-DoFs Skin Stretch Device control
This device is a combination of a Cable-Driven Parallel Robot (CDPR), an Elevation Control Module (ECM) and a Rocker device.\
CDPR is controlled by 4 DC motors: **POLOLU-4846** + **VNH5019 Motor Driver Carrier**.\
ECM is controlled by 2 Stepper motors **POLOLU-1208** + **A4988 Stepper Motor Driver Carrier** (not yet implemented)\
Rocker is controlled by 1 Servo motor **Dynamixel XL330-T288**\

# Instruction:
- upload Arduino code to the microcontroller (Arduino Due)
- check Serial Port in Stretch.py and robot.py
- run manual_calibrate.py to tension the cables
- - for demo in training, run demoGUI.py
- run main.py for main experiment

# Report and Presentation slides are available in /doc

This project is subject to change
