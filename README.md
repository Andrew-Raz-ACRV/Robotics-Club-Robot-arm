![alt text](https://github.com/Andrew-Raz-ACRV/Robotics-Club-Robot-arm/blob/master/QUTRCWallpaper.png)

# Robotics-Club-Robot-arm

## Armageddon — a 4-DOF robot arm and the workshop series built around it

A Raspberry Pi robot arm with inverse-kinematic teleoperation and autonomous colour-based pick-and-place, developed for the QUT Robotics Club's 2018 workshop series.

![alt text](https://github.com/Andrew-Raz-ACRV/Robotics-Club-Robot-arm/blob/master/Robot%20Arm%20pic1.png)

The project shipped as two things: a working arm, and a progressive curriculum that took club members from servo control through kinematics to a computer-vision pipeline. It was modelled on QUT's Introduction to Robotics unit, and funded through the Student Clubs and Projects (SCAP) Fund.

**Status:** archived. Built in 2018 and not maintained. The Python 2 scripts and the OpenCV version it targets are both long superseded; treat this as a teaching reference rather than a current codebase.

<!-- GAP: a photograph of the assembled arm belongs here — it is the single most
     useful thing that could be added, and the original README's arm photo is a good
     candidate if re-cropped. Must work on light and dark backgrounds: no white
     studio background. The current header image is a club wallpaper and should not
     survive; it occupies the most valuable space on the page and says nothing. -->

## What it does

**Kinematics half.** A position-based inverse-kinematic controller that drives the tool point along a trajectory from keyboard input, with USB gamepad teleoperation as an alternative. Link lengths in the DH model mirror the laser-cut structure.

**Vision half.** An autonomous picking routine: the arm locates a coloured ball with a single Pi Camera, plans a trajectory from a set of reference points defined in its own frame, picks the ball up and places it in a box.

The camera is **uncalibrated** and the vision pipeline is colour-threshold based, so this is a demonstration of the full sense-plan-act loop rather than an accurate manipulation system.

## Hardware

- Raspberry Pi 3 B+ with a NOOBS SD card
- Raspberry Pi Camera
- SparkFun Pi Servo HAT (PCA9685, over I²C)
- Hobby King servos
- Laser-cut acrylic structure — CorelDraw files and a full parts list are in this repository

Enable I²C and the camera in `raspi-config` before starting.

## Install

```bash
# keyboard capture and I2C
pip install getch
sudo apt-get install python-smbus

# vision half
sudo apt-get install python-opencv python-matplotlib

# gamepad teleoperation
pip install evdev
```

<!-- The original README had `sudo apt-get install getch` and
     `sudo apt-get install python opencv`. Neither works: getch is a PyPI package
     (the original's own link points at PyPI), and there is no apt package named
     "python opencv". Corrected above.

     GAP: these corrections are inferred from the package names, not verified on a
     Pi. Worth one test run before publishing, on the repo whose entire purpose is
     teaching people to follow instructions. -->

## Running

**Teleoperation** (Python 3), in `Teleoperation_Demo_Python3`:

```bash
python3 RobotArmRaspberryPi_Complete_Code.py
```

**Autonomous vision picking** (Python 2), in `VisionArm_Demo_Python2`:

```bash
python VisionArm.py
```

**Gamepad teleoperation** (Python 2):

```bash
ls /dev/input          # find which event the gamepad is; event3 by default
python RobotArm_Gamepad_teleop.py
```

### Checking your setup

Servo calibration, in `Robot Arm control Workshops`:

```bash
python3 calibration.py
```

Vision thresholding, in `Computer_vision_Workshops`:

```bash
python test_video_blobdetection.py
```

## The workshop curriculum

The teaching material is the part of this repository most likely to be useful to someone else, and it is all here:

- **Workshop slides** with the underlying theory
- **Python scripts for each workshop task**, separate from the final demo scripts, so a session can build up incrementally
- **A MATLAB inverse-kinematics simulator** (`RobotArmSimulator.m`) for working through the kinematics away from the hardware
- **Laser-cut files and parts list** for building the arm itself

The progression runs servo control → forward and inverse kinematics → trajectory following → computer vision → autonomous pick-and-place, with each stage runnable on its own.

## Licence

MIT — see [LICENSE](LICENSE).

<!-- RESOLVED 2026-09-07: Andrew has added MIT to the repo. This was the last
     confirmed licence gap on the profile.

     Optional refinement, not a blocker: the workshop slides and curriculum are
     teaching material rather than code, and CC BY 4.0 is the usual companion licence
     for that half. MIT covers everything as it stands and is perfectly serviceable —
     worth doing only if the material gets reused enough to matter. -->


## Credits

Built for the QUT Robotics Club 2018 workshop series, during my term as club president. The mechanical design was a collaboration; I led the final integrated design, and wrote the control software, kinematics, vision pipeline and workshop curriculum, and sourced the funding.

- **Krishan Rana** — CAD and laser-cut design from the end-effector through to the first joint
- **Marty** — debugging assistance on the computer-vision task
- **Student Clubs and Projects (SCAP) Fund, 2018** — project funding
