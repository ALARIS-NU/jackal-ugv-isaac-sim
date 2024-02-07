#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import argparse
from argparse import Namespace
from src.infer import Predictor

bridge = CvBridge()

args = Namespace(config='/home/robot/September_Artemiy/vision_module/src/human_detection/src/inference_models/human_pp_humansegv2_mobile_192x192_inference_model_with_softmax/deploy.yaml', test_speed=False, use_gpu=True, use_optic_flow=False, use_post_process=False, vertical_screen=False)

predictor = Predictor(args)


def process_img_msg(img_msg, args):
    start = time.time()
    image_pub = args
    np_img_orig = bridge.imgmsg_to_cv2(
        img_msg, desired_encoding='rgb8'
      )
    bg_img = get_bg_img(None, np_img_orig.shape)
    out_img = predictor.run(np_img_orig, bg_img)
    img_normalized = (out_img-out_img.min())/(out_img.max()-out_img.min())*256**2
    img_normalized = img_normalized.astype(np.uint16)
    #print('Image precessed', np_img_orig.shape, type(img_normalized), img_normalized.shape)
    image_pub.publish(bridge.cv2_to_imgmsg(img_normalized, "mono16"))
    print('Image precessed (s)', time.time()-start)

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
    image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)
    img_subscriber = rospy.Subscriber(img_topic, Image, process_img_msg, (image_pub))
    rospy.spin()
