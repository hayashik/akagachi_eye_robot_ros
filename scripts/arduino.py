#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import serial
import atexit
import struct
'''
listens to commands sent to the Arduino, and relays them over
the serial port to Arduino. (ROS -> Arduino)
publish sensor information from Arduino over to ros topic. (Arduino -> ROS)

# Arduino-PC communication protocol
## PC -> Arduino
* byte 0 = 0xaa : header
* byte 1 = 0xaa : header
* byte 2 : left yaw (unsigned char between 50 ~ 130)
* byte 3 : right yaw (unsigned char between 50 ~ 130)
* byte 4 : pitch (unsigned char between 50 ~ 130)

## Arduino -> PC
* byte 0 = 0xbb : header
* byte 1 = 0xbb : header
* byte 2 : dummy data
# use this when you want to extend the program to send back sensor data from Arduino, etc.

# ROS topics
## subscribes to:
* /joint_states  JointState
- eye_lyaw, eye_ryaw, eye_pitch: in radians
'''
MESSAGE_RECEIVE_LENGTH = 3
MESSAGE_SEND_LENGTH = 4

eye_angles = [90, 90, 90]

def set_eye(pos, val):
    '''
    parse commands to eye servos, and save to appropriate variables.
    pos: either "eye_lyaw", "eye_ryaw" or "eye_pitch"
    val: in radians
    '''
    val = int(val * 180.0 / 3.14)  # convert to degrees
    val = max(min(val, 40), -40)  # constrain between -40 ~ 40
    val += 90  # data is between 50 ~ 130
    global eye_angles
    if pos == "eye_lyaw":
        eye_angles[0] = int(val)
    elif pos == "eye_ryaw":
        eye_angles[1] = int(val)
    elif pos == "eye_pitch":
        eye_angles[2] = int(val)

def get_bytearray():
    '''
    return an array of bytes that can be sent through the serial port to Arduino.
    '''
    global eye_angles
    check_digit = sum(eye_angles) % 256
    data_array = bytearray(eye_angles + [check_digit])
    if len(data_array) != MESSAGE_SEND_LENGTH:
        raise ValueError()
    return data_array

def parse_data(data):
    '''
    parse data sent from Arduino.
    '''
    dummy_data = struct.unpack('B', data[2]) # load as unsigned char
    rospy.loginfo("dummy data: {}".format(dummy_data))

def callback(msg):
    # when rostopic /joint_states is received.
    for i in range(len(msg.name)):
        if (msg.name[i] == "eye_lyaw" or msg.name[i] == "eye_ryaw"
              or msg.name[i] == "eye_pitch"):
            set_eye(msg.name[i], msg.position[i])     

rospy.init_node('arduino_node', anonymous=True)
rospy.Subscriber('/joint_states', JointState, callback)
try:
    port = rospy.get_param("arduino_port")
except KeyError:
    rospy.logerr("please set ros parameter \"arduino_port\" to the serial port name of Arduino.")
    raise KeyError()
rospy.loginfo("connecting to Arduino at port {}...".format(port))
ser = serial.Serial(port, timeout=1, baudrate=9600)
# automatically disconnect when exiting from script
atexit.register(ser.close)

rate = rospy.Rate(40)
serial_buffer = []
rospy.loginfo("starting communication with Arduino...")
while not rospy.is_shutdown():
    # send data to Arduino
    ser.write(get_bytearray())
    # add all the received serial data to serial_buffer
    while ser.in_waiting>0:
        serial_buffer.append(ser.read())
    i=0
    while True:
        # find the messages saved in serial_buffer
        if i >= len(serial_buffer)-MESSAGE_RECEIVE_LENGTH+1:
            break
        if serial_buffer[i] == '\xbb' and serial_buffer[i+1] =='\xbb':
            parse_data(serial_buffer[i:i+MESSAGE_RECEIVE_LENGTH])
            serial_buffer = serial_buffer[i+MESSAGE_RECEIVE_LENGTH:]
            i = 0
            continue
        i+=1
    rate.sleep()