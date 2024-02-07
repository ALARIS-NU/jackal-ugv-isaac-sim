#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


def joint_vels_callback(data):
    global my_file_joint
    now = rospy.get_rostime()
    #print("I heard ",data.drivers, now.to_sec())
    data_list = [now.to_sec(), data.velocity[0], data.velocity[1], data.velocity[2], data.velocity[3]]
    my_file_joint.write(' '.join(str(x) for x in data_list))
    my_file_joint.write('\n')
    
def odom_callback(data):
    global my_file_odom
    now = rospy.get_rostime()
    #print("I heard ",data.drivers, now.to_sec())
    data_list = [now.to_sec(), data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, 
                 data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    my_file_odom.write(' '.join(str(x) for x in data_list))
    my_file_odom.write('\n')
    
def listener():
    rospy.init_node('listener', anonymous=True)
    
    #   ros::Subscriber sim_updater = node.subscribe("/cmd_drive", 1, update_odom);
    rospy.Subscriber("/joint_states", JointState, joint_vels_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    global my_file_joint, my_file_odom
    my_file_joint = open('joint_data.txt', 'w')
    my_file_odom = open('odom_data.txt', 'w')
    listener()
    my_file_joint.close()
    my_file_odom.close()
