#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import rospy
import tf2_ros



class PublishPoseFromTF:
    def __init__(self):
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # This frame is used as output pose's reference coordinate
        self._base_frame = 'map'
        # This frame's pose relative to base_frame will be published as geometry_msgs/PoseStamped
        self._pose_frame = 'base_link'
        # Timeout for lookup transform between base_frame and pose_frame
        self._timeout = rospy.get_param('~timeout', 0.1)

        self._pose_pub = rospy.Publisher('/rtabmap_pose_extruction', PoseStamped, queue_size = 1)
        self.my_file = open('pose.txt', 'a')
        self.main_loop()
    
    def main_loop(self):
        while not rospy.is_shutdown():
            try:
                transform = self._tf_buffer.lookup_transform(self._base_frame, self._pose_frame, rospy.Time.now(), rospy.Duration(self._timeout))
            except:
                continue 

            pose = PoseStamped()
            pose.header.frame_id = self._base_frame
            pose.header.stamp = transform.header.stamp

            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            now = rospy.get_rostime()
            data_list = [now.to_sec(), pose.header.stamp, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 
                         pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                         pose.pose.orientation.w ]
            self.my_file.write(' '.join(str(x) for x in data_list))
            self._pose_pub.publish(pose)
        self.my_file.close()
            
 
if __name__ == '__main__':
    rospy.init_node('publish_pose_from_tf')
    publish_pose_from_tf = PublishPoseFromTF()
