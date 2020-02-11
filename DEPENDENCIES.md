

# Libraries
## General Dependencies
```bash
apt install -y pkg-config git cmake build-essential nasm wget  python3-setuptools libusb-1.0-0-dev  python3-dev python3-pip python3-numpy python3-scipy libglew-dev libglfw3-dev libtbb-dev python3-catkin-pkg-modules python3-rospkg-modules python3-yaml
pip3 install pyserial
pip3 install scikit-image
```
## OpenCV >= 3
```bash
apt update
apt install -y python3-opencv libopencv-dev ros-melodic-cv-bridge ros-melodic-opencv-apps
pip3 install opencv-python
pip3 install opencv-contrib-python
pip3 install imutils
```
# Dlib Under Development
    pip3 install dlib