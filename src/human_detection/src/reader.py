#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import argparse
from argparse import Namespace
from src.infer import Predictor

bridge = CvBridge()

def process_img_msg(self, img_msg: Image):
    """ callback function for publisher """
    np_img_orig = bridge.imgmsg_to_cv2(
        img_msg, desired_encoding='rgb8'
    )
    print('Image precessed', np_img_orig.size)

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
    image_pub = rospy.Publisher("image_topic_2",Image)
    # init network
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="/home/jackal/Jackal_2023/September/Vision_ws/src/human_detection/src/inference_models/human_pp_humansegv2_mobile_192x192_inference_model_with_softmax/deploy.yaml")
    parser.add_argument(
        '--vertical_screen',
        help='The input image is generated by vertical screen, i.e. height is bigger than width.'
        'For the input image, we assume the width is bigger than the height by default.',
        action='store_true')
    parser.add_argument(
        '--test_speed',
        help='Whether to test inference speed',
        action='store_true')
    parser.add_argument(
        '--use_post_process', help='Use post process.', action='store_true')
    parser.add_argument(
        '--use_optic_flow', help='Use optical flow.', action='store_true')
    parser.add_argument(
        '--bg_img_path',
        help='Background image path for replacing. If not specified, a white background is used',
        type=str)
    print(parser.parse_args())
    print('\n\n\n')
    args = Namespace(config='/home/jackal/Jackal_2023/September/Vision_ws/src/human_detection/src/inference_models/human_pp_humansegv2_mobile_192x192_inference_model_with_softmax/deploy.yaml', test_speed=False, use_gpu=True, use_optic_flow=False, use_post_process=False, vertical_screen=False)
    print(args)
    predictor = Predictor(args)
    img_topic = '/camera/color/image_raw'
    img_msg = rospy.wait_for_message(img_topic, Image)
    np_img_orig = bridge.imgmsg_to_cv2(
        img_msg, desired_encoding='rgb8'
      )
    bg_img = get_bg_img(None, np_img_orig.shape)
    out_img = predictor.run(np_img_orig, bg_img)
    img_normalized = (out_img-out_img.min())/(out_img.max()-out_img.min())*256**2
    img_normalized = img_normalized.astype(np.uint16)
    print('Image precessed', np_img_orig.shape, type(img_normalized), img_normalized.shape)
    image_pub.publish(bridge.cv2_to_imgmsg(img_normalized, "mono16"))
    #img_subscriber = rospy.Subscriber(img_topic, Image, process_img_msg)
