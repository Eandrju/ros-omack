#!/usr/bin/env python
import cv2, os, rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image

class Shifter:
    def __init__(self):
        image_in = rospy.get_param('/image_shifter/input', '/camera/rgb/image_rect_color/compressed')
        self.image_sub = rospy.Subscriber(image_in, CompressedImage, self.callback)

        image_out = rospy.get_param('/image_shifter/output', '/shifted_image')
        self.image_pub = rospy.Publisher(image_out, Image, queue_size=2)
        self.image_compressed_pub = rospy.Publisher(
            image_out+'/compressed',
            CompressedImage,
            queue_size=2
        )
        self.bridge = CvBridge()

    def callback(self, img_msg):
        arr = np.fromstring(img_msg.data, np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        rows, cols, depth = image.shape

        vertical_offset = rospy.get_param('image_shifter/vertical_offset', 0)
        horizontal_offset = rospy.get_param('image_shifter/horizontal_offset', 0)

        M = np.float32([[1,0, horizontal_offset], [0, 1, vertical_offset]])
        shifted_image= cv2.warpAffine(image, M, (cols, rows))


        msg = Image()
        msg = self.bridge.cv2_to_imgmsg(shifted_image, encoding='bgr8')
        msg.header = img_msg.header
        self.image_pub.publish(msg)

        msg_compressed = CompressedImage()
        msg_compressed.data = np.array(cv2.imencode('.png', shifted_image)[1]).tostring()
        msg_compressed.header = img_msg.header
        self.image_compressed_pub.publish(msg_compressed)


if __name__ == '__main__':
    shifter = Shifter()
    rospy.init_node('image_shifter')
    rospy.spin()


