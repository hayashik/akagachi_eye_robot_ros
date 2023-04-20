#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

rospy.init_node('joint_state_blink_publisher', anonymous=True)

def joint_state_publisher():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    js = JointState()
    #joint_state.name = ['joint1', 'joint2']
    #joint_state.position = [0.0, 0.0]
    #joint_state.velocity = [] 
    #joint_state.effort = []


    js.header.stamp = rospy.Time.now() 
    js.name.append("eyelid_joint")
    js.position.append(-0.523)
    pub.publish(js) 

    rospy.sleep(0.1)

    js.header.stamp = rospy.Time.now() 
    js.name.append("eyelid_joint")
    js.position.append(0)
    pub.publish(js) 

    rospy.sleep(1)

def callback(msg):
    #print("Message '{}' recieved".format(msg.data))

    #print('msg: {}'.format(msg))
    if(msg == String('blinking')):
        joint_state_publisher()

def subscriber():

    rospy.Subscriber("blink_states", String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
