#! /usr/bin/env python3
'''
This script calculates how the eye should be moved in order to "look" at some point.
- uses arctan2 to calculate the angles to turn the eyes in order to look at the /focus frame.
'''
import rospy
import tf
from sensor_msgs.msg import JointState
import numpy as np

def eye(pub, trans_l, trans_r):
    js = JointState()
    js.header.stamp = rospy.Time.now()

    js.name.append("eye_lyaw")
    js.position.append(-np.arctan2(trans_r[1], trans_r[0]))

    js.name.append("eye_ryaw")
    js.position.append(-np.arctan2(trans_l[1], trans_l[0]))

    js.name.append("eye_pitch")
    js.position.append(-np.arctan2(trans_l[2], trans_l[0]))

    #print('clsulator: {} {}'.format(trans_l, trans_r))

    #global pub
    pub.publish(js)

if __name__ == "__main__":
    rospy.init_node('foveation_calculator', anonymous=True)
    listener = tf.TransformListener()
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rate = rospy.Rate(10)
    trans_l, trans_r = [1,0,0], [1,0,0]
    while not rospy.is_shutdown():
        try:
            (trans_l, rot_l) = listener.lookupTransformFull('/focus', rospy.Time(0), '/eye_l', rospy.Time(0), '/eye_base')
            (trans_r, rot_r) = listener.lookupTransformFull('/focus', rospy.Time(0), '/eye_r', rospy.Time(0), '/eye_base')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        eye(pub, trans_l, trans_r)
        rate.sleep()
