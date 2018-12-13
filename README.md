![alt text](https://github.com/Andrew-Raz-ACRV/Robotics-Club-Robot-arm/blob/master/QUTRCWallpaper.png width=100)

# Robotics-Club-Robot-arm
This project is part of the 2018 workshop series for the QUT Robotics club. It is based on the unit called Introduction to Robotics which is a subject focussed on the control of a Robotic arm and the application of computer vision processes. For half the project, the focus was on developing our own brand of robotic arms (Armageddon) with a position-based inverse kinematics controller that made the toolpoint of the robot follow a trajectory given by keyboard presses. The second half expanded the work towards an autonomous picking system where the robot uses a camera to see a ball, given some reference points to its frame of reference, and plan a trajectory to pick up the ball and place it into a box.

![alt text](https://github.com/Andrew-Raz-ACRV/Robotics-Club-Robot-arm/blob/master/Robot%20Arm%20pic1.png width=100)

## Getting Started with the Project
This project is based on using the robotic arms designed by the club which involve components:
* Raspberry Pi 3 B+ with a NOOBS sd card
* Raspberry Pi Camera
* Power adapter for the Raspberry Pi
* Sparkfun Raspberry Pi Servo hat
* Hobby King servos
* Laser cut parts developed by the club which is on this Github
a more comprehensive parts list is in this Github as well.

### Prerequisites
The Raspberry Pi project requires a few installations
- Getch is for capturing keyboard presses: https://pypi.org/project/getch/
- smbus for the i2c communication to the servo hat: https://learn.sparkfun.com/tutorials/pi-servo-hat-hookup-guide/all#software---python
- An old version of opencv that can be installed easy and quickly 
- Optional installation with matlibplot

Installations for the Kinematics Half of the Project:
```
sudo apt-get install getch
sudo apt-get install smbus
```

Installations for the Computer Vision half of the project:
```
sudo apt-get install python opencv
sudo apt-get install python matlibplot
```

## Running Robot Arm Tests:

Robot Arm Calibration Test:
In Robot_Arm_control_Workshops, you can test the calibration of the servos to see if they are accurate
```
python calibration.py
```

Computer Vision Threshold Test:
In Computer_vision_Workshops, you can run a script to see the thresholding
```
python test_video_blobdetection.py
```

## Contents on this Github:
This Github holds the following:
* Laser cut file (CorelDraw) of the Robot Arm
* Parts List for Constructing the Robot Arm
* MATLAB simulation of the inverse Kinematics provided with Andrew's Kinematics functions
  Run the file RobotArmSimulator.m
* Workshop Slides with Theory explained in them
* Python scripts of the Workshop tasks
* Python scripts of the Final Demo Scripts


## Final Demo Scripts:

In the Terminal, you can run these commands to run the demos:

The Robot Arm Teleoperation Script in Python 3
In Teleoperation_Demo_Python3:
```
python3 RobotArmRaspberryPi_Complete_Code.py
```

The Robotic Vision Autonomous Picking Script in Python 2
In VisionArm_Demo_Python2:
```
python VisionArm.py
```

## Author

* **Andrew Razjigaev** - *President of the QUT Robotics Club 2018*  

## Acknowledgments
* Student Clubs and Projects (SCAP) Fund 2018
* Krishan Rana - For his Development of the gripper and joints Laser cut pieces.
* Marty - For his assistance with running the Computer Vision task
