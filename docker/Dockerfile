FROM osrf/ros:melodic-desktop-full

ENV TZ=Asia/Tokyo
ENV DEBIAN_FRONTEND=noninteractive

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt update -y && apt-get install -yq make cmake gcc g++ unzip wget build-essential gcc libtbb-dev usbutils x11-apps v4l-utils emacs

##General Dependencies
RUN apt install -y pkg-config git cmake build-essential nasm wget python3-setuptools libusb-1.0-0-dev  python3-dev python3-pip python3-numpy python3-scipy libglew-dev libglfw3-dev libtbb-dev \
    python3-catkin-pkg-modules python3-rospkg-modules python3-yaml python-pip

RUN apt update && apt upgrade -y && apt install -y tzdata
RUN apt update && apt install -y --no-install-recommends mesa-utils x11-apps && rm -rf /var/lib/apt/lists/*

# uvc camera dependencies
RUN apt update && apt-get install -y uvccapture guvcview cheese

# pkg dependencies
RUN pip3 install pyserial
RUN pip3 install scikit-image
RUN pip install scikit-image

RUN git config --global user.name "Hayashi Kotaro"
RUN git config --global user.email hayashik@cs.tut.ac.jp

# OpenCV >= 3
RUN apt update
RUN apt install -y python3-opencv libopencv-dev ros-melodic-cv-bridge ros-melodic-opencv-apps
RUN pip3 install opencv-python
RUN pip3 install opencv-contrib-python
RUN pip3 install imutils

# Dlib
RUN pip3 install dlib

RUN export QT_X11_NO_MITSHM=1