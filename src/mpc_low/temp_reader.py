#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid, Path

def callback(data):
    if len(data.poses)>0:
    	rospy.loginfo("Next pase stes")
    	rospy.loginfo(data.poses[0].pose.position)
    	rospy.loginfo(data.poses[1].pose.position)
    else:
    	rospy.loginfo("0 poses")

def callback_map(msg):
    my_list = msg.data
    rospy.loginfo("Costmap")
    rospy.loginfo(msg.info)
    rospy.loginfo(len(msg.data))
    rospy.loginfo(sum(i > 99 for i in my_list))
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/move_base/DWAPlannerROS/global_plan", Path, callback)
    
    rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, callback_map)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
