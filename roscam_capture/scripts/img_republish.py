#!/usr/bin/env python3

import rospy
import cv2, sys
import time
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
#from memory_profiler import profile

path = '/home/hagi/place/images/img{}.png'
_bridge = CvBridge()
c = _time = 0

def start_node():
    node_name = '/usb_cam/image_raw' 
    _image_sub = rospy.Subscriber(node_name, Image, callback)

def get_colored_area(cv_image, lower, upper):
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask_image = cv2.inRange(hsv_image, lower, upper)
    area = cv2.countNonZero(mask_image)
    return area

#@profile 
def callback(data):
    global c, _time
    now = time.time()
    try:
        cv_image = _bridge.imgmsg_to_cv2(data, 'passthrough')
        cv_image = np.array(cv_image, dtype=np.float32)

    except CvBridgeError as e:
        print(e)
    print('cost time is ', (time.time()-now)*1000, '[ms]')
    _time += (time.time()-now)*1000
    c += 1
    blue_area = get_colored_area(
            cv_image, np.array([50,100,100]), np.array([150,255,255]))
    
    #rospy.loginfo('blue=%d, red=%d' % (blue_area, red_area))
    cv2.imwrite(path.format(str(blue_area)), cv_image.astype(np.uint8))
    if c==30:
        print('avg time is', _time)
        c=0
        _time = 0

if __name__ == '__main__':
    rospy.init_node('color_vel')
    start_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
