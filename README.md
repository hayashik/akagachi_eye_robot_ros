# 'Akagachi' - Open source robot eye component
This project is in active and an eye componet for HRI studies.
Akagachi means soulful eyes in archaic Japanese.
This repository contains the source code of ROS, a parametric 3d CAD.
This component is written in Python and ROS package. 
CAD files for Robotic eye system are also available.  
Our vision is to contribute for diverse group interasted in the social cues of eyes.
Hardware details are [here](https://github.com/hayashik/akagachi_eye_robot_hardware).

## Software
### Installing on Ubuntu 16.04, 18.04 and ROS kinetic, melodic
This packages have been developed and checked on ROS kinetic and melodic. The below settings is recommended.
Dockerfile is avairable.

- ROS [Kinetic](http://wiki.ros.org/kinetic) or [Melodic](http://wiki.ros.org/melodic)
  - OS: Ubuntu 16.04 LTS or 18.04 LTS
  - Python 3.7
  - Opencv >= 3
  - [Dlib](http://dlib.net/) 19.18 # now under development

### sketch/sketch.ino
Edit the pin assignments according to your setup, and write this sketch to Arduino.

- Arduino IDE 1.8.10
    - [x] Arduino Nano

### Python scripts
#### scripts/arduino.py
ROS node for communication with Arduino. Listens to /joint_states ROS topic for commands to send to Arduino.

#### scripts/vision_optical_flow.py
ROS node for detection by the optical flow.



## Package setting
place this repository in /src directory of your catkin workspace, and run `catkin_make`.
Set the arduino_port parameter in config/config.yaml to the port of your Arduino.
```bash
roslaunch akagachi_demo manual_control.launch
```
## Producting robot
### CAD data
These CAD data of this repository were designed using `Fusion 360`. You may make any changes to the `.step` files by any 3D CAD software. If you only want to print our drives then the `.stl` files are all you need.

### Akagachi versions
- [x] **1-A** : Simple and compact eye robot. Using brush servos direct drive. 
- [ ] **1-B** : Simple eye robot. Using brushless servos linkage.    # now under development
- [ ] **1-C** : Display eye robot.  # now under development

### Enabled IoT device
- [x] [Tinker Board S](https://www.asus.com/Single-Board-Computer/Tinker-Board/) and Armbian Bionic 4.4y
- [x] Raspberry Pi 3 Model B+ and Ubuntu Mate 18.04

## License
MIT  
When you publicate some researches, please refer a follow paper and the acknowledgements to the [Connected Robotics Inc.](https://connected-robotics.com/en/) can be added.  
Kotaro Hayashi & Yasunori Toshimitsu, "Eyes on You: Field Study of Robot Vendor Using Human-Like Eye Component “Akagachi",” In Proceedings of 25th IEEE International Symposium on Robot and Human Interactive Communication (RO-MAN), pp. 119-124.
