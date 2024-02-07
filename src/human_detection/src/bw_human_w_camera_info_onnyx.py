#!/usr/bin/env python3
import cv2
import numpy as np
from numpy import inf
import rospy
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import argparse
from argparse import Namespace
from src.infer import Predictor


import message_filters
from sensor_msgs.msg import Image

bridge_color = CvBridge()
bridge_depth = CvBridge()

args = Namespace(config='/home/robot/September_Artemiy/vision_module/src/human_detection/src/inference_models/human_pp_humansegv2_mobile_192x192_inference_model_with_softmax/deploy.yaml', test_speed=False, use_gpu=True, use_optic_flow=False, use_post_process=False, vertical_screen=False)



#args = Namespace(config='/home/robot/September_Artemiy/vision_module/src/human_detection/src/inference_models/human_pp_humansegv2_lite_192x192_inference_model_with_softmax/deploy.yaml', test_speed=False, use_gpu=True, use_optic_flow=False, use_post_process=False, vertical_screen=False)

predictor = Predictor(args)


def process_img_msg(img_msg, args):
    start = time.time()
    image_pub = args
    np_img_orig = bridge_color.imgmsg_to_cv2(
        img_msg, desired_encoding='rgb8'
      )
    bg_img = get_bg_img(None, np_img_orig.shape)
    out_img = predictor.run(np_img_orig, bg_img)
    img_normalized = (out_img-out_img.min())/(out_img.max()-out_img.min())*256**2
    img_normalized = img_normalized.astype(np.uint16)
    #print('Image precessed', np_img_orig.shape, type(img_normalized), img_normalized.shape)
    image_pub.publish(bridge_color.cv2_to_imgmsg(img_normalized, "mono16"))
    print('Image precessed (s)', time.time()-start)
    

def process_img_msg_sync(img_msg, depth_image, depth_info, args):
    start = time.time()
    image_pub = args[0]
    info_pub = args[1]
    np_img_orig = bridge_color.imgmsg_to_cv2(
        img_msg, desired_encoding='rgb8'
      )
    np_depth_orig = bridge_depth.imgmsg_to_cv2(depth_image)
    bg_img = get_bg_img(None, np_img_orig.shape)
    out_img = predictor.run(np_img_orig, bg_img)
    #print('Image precessed (s)', time.time()-start)
    #print(depth_image.header.frame_id)
    img_normalized = (out_img-out_img.min())/(out_img.max()-out_img.min())*256
    #img_normalized = out_img*256;
    img_normalized = img_normalized.astype(np.uint8)
    #print('Image precessed', np_img_orig.shape, type(img_normalized), img_normalized.shape)
    #image_pub.publish(bridge_color.cv2_to_imgmsg(img_normalized, "mono16"))
    
    ret,thresh = cv2.threshold(img_normalized,200,255,0)
    #thresh = cv2.bitwise_not(thresh)
    output = cv2.bitwise_and(np_depth_orig, np_depth_orig, mask=thresh)
    masked_image_msg = bridge_depth.cv2_to_imgmsg(output)
    #list(im.getdata())
    output[output == inf] = 10.0
    #print(np.nanmean(np.where(output!=0.0,output,np.nan)))
    #print(output.max(),output.min(),output.mean(),'max values')
    #masked_image_msg = bridge_depth.cv2_to_imgmsg(thresh)
    masked_image_msg.header = depth_image.header
    image_pub.publish(masked_image_msg)
    info_pub.publish(depth_info)
    print('Image precessed (s)', time.time()-start, 'distance:',np.nanmean(np.where(output!=0.0,output,np.nan)))



def get_bg_img(bg_img_path, img_shape):
    if bg_img_path is None:
        bg = 255 * np.ones(img_shape)
    elif not os.path.exists(bg_img_path):
        raise Exception('The --bg_img_path is not existed: {}'.format(
            bg_img_path))
    else:
        bg = cv2.imread(bg_img_path)
    return bg
        
        
if __name__ == "__main__":
    rospy.init_node("human_node")
    # init network
    img_topic = '/rgb'
    depth_topic = '/depth'
    image_pub = rospy.Publisher("image_topic_3",Image, queue_size=1)
    info_pub = rospy.Publisher("image_topic_3/camera_info",CameraInfo, queue_size=1)
    #img_subscriber = rospy.Subscriber(img_topic, Image, process_img_msg, (image_pub))
    
    image_sub = message_filters.Subscriber(img_topic, Image)
    depth_sub = message_filters.Subscriber(depth_topic, Image)
    depth_info = message_filters.Subscriber('/camera_info', CameraInfo)
    
    ts = message_filters.TimeSynchronizer([image_sub, depth_sub, depth_info], 1, 0.5)
    ts.registerCallback(process_img_msg_sync, (image_pub, info_pub))
    
    rospy.spin()
