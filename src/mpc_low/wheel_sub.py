#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from jackal_msgs.msg import Drive


def callback(data):
    global my_file
    now = rospy.get_rostime()
    print("I heard ",data.drivers, now.to_sec())
    data_list = [now.to_sec(), data.drivers[0], data.drivers[1]]
    my_file.write(' '.join(str(x) for x in data_list))
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    #   ros::Subscriber sim_updater = node.subscribe("/cmd_drive", 1, update_odom);
    rospy.Subscriber("/cmd_drive", Drive, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    global my_file
    my_file = open('comand.txt', 'a')
    listener()
    my_file.close()
