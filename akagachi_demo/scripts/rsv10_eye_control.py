#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
import sys
import rospy
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
try:
    sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
except:
    pass
import pyrs_rsv10
import velocity_eyes
import numpy as np

'''
Listens to commands sent to the Futaba servo, and relays them over
the serial port to Futaba serial converter. (ROS -> RSC-U485)

/futaba_command  JointState
- eye_lyaw, eye_ryaw, eye_pitch: in radians
'''

USB_PORT = '/dev/ttyUSB0'
USB_BAUDRATE = 115200
SERVO_NUM = 4
debug = False
ORG_PARAM = rospy.get_param("/servo_org_list")
print(ORG_PARAM[0])

'''
Connect to Eye robots with Futaba servos.
'''
class futaba_state:
    def __init__(self):

        self.servos_command = []
        
        #Futaba connecter
        self.rs = pyrs_rsv10.Rs()
        self.vel_eyes = velocity_eyes.Veye()
        self.servo_deg = [0] * SERVO_NUM
        self.servo_org = ORG_PARAM
        self.servo_max = [0] * SERVO_NUM
        self.servo_min = [0] * SERVO_NUM

        #config moter1 : right eye vertical
        self.servo_deg[0] = self.servo_org[0]
        self.servo_max[0] = self.servo_org[0] + 40
        self.servo_min[0] = self.servo_org[0] -40

        #config moter2 : left eye vertical
        self.servo_deg[1] = self.servo_org[1] 
        self.servo_max[1] = self.servo_org[1] + 40
        self.servo_min[1] = self.servo_org[1] -40

        #config moter3 : eye horizontal
        self.servo_deg[2] = self.servo_org[2] 
        self.servo_max[2] = self.servo_org[2] + 30
        self.servo_min[2] = self.servo_org[2] -30

        #config moter4 : blinking
        self.servo_deg[3] = self.servo_org[3] 
        self.servo_max[3] = self.servo_org[3] + 40
        self.servo_min[3] = self.servo_org[3] -60

        #multi-servo setting
        self.servos_on = [(i, 1) for i in range(1, SERVO_NUM+1)]
        self.servos_off = [(i, 0) for i in range(1, SERVO_NUM+1)]
        self.servos_command = [(i+1, self.servo_deg[i], 200) for i in range(SERVO_NUM)]
        
        self.servo_blink_deg = 20
        
        print('servo_on is {}'.format(self.servos_on))
        print('servo_off is {}'.format(self.servos_off))

    def set_servos_turn_on(self, usb_port=USB_PORT):
        '''
        turn on futaba servos
        '''
        self.rs.open_port(usb_port, USB_BAUDRATE, 1)
        self.rs.multi_torque_on(self.servos_on)
        print("servos turn on")
        

    def set_servos_turn_off(self):
        '''
        turn off futaba servos
        '''
        self.rs.multi_torque_on(self.servos_off)
        #for i in range(1, n+1):
        #    self.rs.torque_on(i, 0)

    def set_eye(self, pos, val):
        '''
        pos: either "eye_lyaw", "eye_ryaw", "eye_pitch", "leyelid" or "reyelid"
        val: in radians
        '''
        # convert to degrees
        val = -1 * int(val * 180.0 / 3.14) * 10

        if pos == "eye_lyaw":
            val += self.servo_org[1]
            if val < self.servo_min[1]:
                val = self.servo_min[1]
            elif val > self.servo_max[1]:
                val = self.servo_max[1]
            self.servo_deg[1] = int(val)
        elif pos == "eye_ryaw":
            val += self.servo_org[0]
            if val < self.servo_min[0]:
                val = self.servo_min[0]
            elif val > self.servo_max[0]:
                val = self.servo_max[0]
            self.servo_deg[0] = int(val)
        elif pos == "eye_pitch":
            val = val + self.servo_org[2]
            if val < self.servo_min[2]:
                val = self.servo_min[2]
            elif val > self.servo_max[2]:
                val = self.servo_max[2]
            self.servo_deg[2] = int(val)
        elif pos == "eyelid_joint":
            val = val + self.servo_org[3]
            if val < self.servo_min[3]:
                val = self.servo_min[3]
            elif val > self.servo_max[3]:
                val = self.servo_max[3]
            self.servo_deg[3] = int(val)
        else:
            print('as argument "pos", please put in either "eye_lyaw",\
            "eye_ryaw" or "eye_pitch"')

        self.servos_move(8)

    def set_blink(self, pos, val):
        '''
        pos: blink
        val: in radians
        '''
        # convert to degrees

        if (pos == "eyelid_joint" and val == 1):
            self.rs.target_position(4, self.servo_max[3], 8)
        elif (pos == "eyelid_joint" and val == 0):
            self.rs.target_position(4, self.servo_org[3], 8)
        else:
            print('as argument "pos", please put in "blink"')

    def servos_move(self, time):
        '''
        move servos to target degree
        time: moving time duration (x10msec)
        '''
        self.servos_command = [(i+1,self.servo_deg[i], time) for i in range(0, SERVO_NUM)]
        self.rs.multi_target_position(self.servos_command)

    def servos_init(self):
        '''
        init servo to origin degree
        '''
        self.servos_command = [(i+1, self.servo_org[i], 8) for i in range(SERVO_NUM)]
        self.rs.multi_target_position(self.servos_command)

        print("/set to origin")
        print(self.servos_command)

def callback(msg):
    for i in range(len(msg.name)):
        if(msg.name[i] == "eye_lyaw" or msg.name[i] == "eye_ryaw" or msg.name[i] == "eye_pitch"):
            state.set_eye(msg.name[i], msg.position[i])
        elif(msg.name[i] == "eyelid_joint"):
            state.set_eye(msg.name[i], msg.position[i])
        elif(msg.name[i] == "blink"):
            state.set_natural_blink(msg.name[i], msg.position[i])

def callback_exp_msg(data):
    '''
    '''
    if(data.data == 'exp_end'):
        state.servos_init()
        print("set origin!!")
        rospy.sleep(100.0) 
    

if __name__ == "__main__":
    rospy.init_node('rsv10_eye_controller', anonymous=True)
    pub = rospy.Publisher('/rsv10_servos', JointState, queue_size=1)
    rospy.Subscriber('/joint_states', JointState, callback)
    state = futaba_state()
    rate = rospy.Rate(20)
    servos_deg = []
    #esv10 setting
    usb_port = rospy.get_param('~port', '/dev/ttyUSB0')
    state.set_servos_turn_on(usb_port)
    state.servos_init()

    while not rospy.is_shutdown():
        
        #state.servos_move(300)
        rate.sleep()

    state.servos_init()
    #state.set_servos_turn_off()

