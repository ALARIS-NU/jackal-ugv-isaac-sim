#!/usr/bin/env python3
import rospy
import time

import sys
import select
import tty
import termios

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty

from std_msgs.msg import Float64

from threading import Timer


from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

barc_count = 0

rospy.init_node('emulator', anonymous=True)

pub = rospy.Publisher('manual_command_topic', Float64MultiArray, queue_size=1)


class Watchdog:
    def __init__(self, timeout, userHandler=None):  # timeout in seconds
        self.timeout = timeout
        self.handler = userHandler if userHandler is not None else self.defaultHandler
        self.timer = Timer(self.timeout, self.handler)
        self.timer.start()

    def reset(self):
        self.timer.cancel()
        self.timer = Timer(self.timeout, self.handler)
        self.timer.start()

    def stop(self):
        self.timer.cancel()

    def defaultHandler(self):
        raise self

def myHandler():
    global watchdog
    global pub
    global barc_count
    vells_array = [0.0, 0.0]
    hello_speed = Float64MultiArray()
    hello_speed.data = vells_array
    pub.publish(hello_speed)
    rospy.loginfo('bark bark')
    barc_count = barc_count + 1
    watchdog.reset() 

watchdog = Watchdog(0.150, myHandler) # run watchdog handler if no messages in 100ms

counter = 0
def jointcommand(data):
    global watchdog
    global pub, counter
    vells_array = [0.0, 0.0]
    vells_array = [data.data[2]*0.5, data.data[1]*0.5] # reverse order
    hello_speed = Float64MultiArray()
    hello_speed.data = vells_array
    pub.publish(hello_speed)
    rospy.loginfo(hello_speed)
    #,data.data)
    if counter%1000 == 0:
        print(vells_array)
    counter = counter + 1
    watchdog.reset() 

    
def main():
    global pub
    global barc_count
    rospy.Subscriber("/joint_group_velocity_controller/command", Float64MultiArray, jointcommand, queue_size=1)
    print('connecting')
    temp = 0
    while (pub.get_num_connections() < 1):
        #print('waiting')
        temp = temp+1
    print('started',temp)
    rate = rospy.Rate(120) # 
    vells_array = [0.0, 0.0]
    print('dog is out')
    ended = 0
    dummy_counter = 0;
    while not rospy.is_shutdown():
        try:
            #if dummy_counter == 0:
            #    #print('bark')
            dummy_counter = (dummy_counter+1)%1000
        except KeyboardInterrupt:
            print('recording finished')
            f.close()
            break
    print('node stopped')

    print('well done, I am out')
    vells_array = [0.0, 0.0]
    hello_speed = Float64MultiArray()
    hello_speed.data = vells_array
    pub.publish(hello_speed)
    pub.publish(hello_speed)
    pub.publish(hello_speed)
    time.sleep(0.5) # wait for 5 seconds
    rospy.loginfo(hello_speed)
    watchdog.stop()

if __name__ == '__main__':
    main()
