#!/usr/bin/env python
import cv2, os, rospy
import numpy as np
import message_filters as mf
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from object_detector.msg import DetectionBundle
from object_detector.msg import Detection

class VisualizerWrapper:
    def __init__(self):
        self.image_sub = mf.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage)
        self.bundle_sub = mf.Subscriber("/detector/detection_bundle", DetectionBundle)

        self.ts = mf.TimeSynchronizer([self.image_sub, self.bundle_sub], 20)
        self.ts.registerCallback(self.callback)

        self.image_pub = rospy.Publisher('/visualizer/rgb/image_raw/compressed',
                                         CompressedImage, queue_size=2)
        self.bridge = CvBridge()
        path = os.path.dirname(os.path.abspath(__file__))
        self.labels = open(os.path.join(path, 'data', 'coco.names')).read().split('\n')[:-1]
        self.colors = np.random.randint(0, 255, size=(len(self.labels), 3), dtype='uint8').tolist()
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.fontScale = 0.35
        self.fontSize = 1
        self.rectThickness = 2

    def callback(self, img_msg, bundle):
        arr = np.fromstring(img_msg.data, np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        np.random.seed(69)
        print(type(image))

        print('------ new bundle -------')
        print(image.shape)
        for d in bundle.detections:
            start = d.x0, d.y0
            end = d.x0 + d.w, d.y0 + d.h
            color = (d.color.r, d.color.g, d.color.b)
            label = d.class_name
            image = cv2.rectangle(image, start, end, color, self.rectThickness)
            x = self.rectThickness
            # label rectangle and text
            text = '{0:s}'.format(label, d.certainty)
            text_size = cv2.getTextSize(text, self.font, self.fontScale, self.fontSize)[0]
            image = cv2.rectangle(image,
                                  (start[0], start[1] - text_size[1] - x,),
                                  (start[0] + text_size[0], start[1] - x,),
                                  tuple(color), -1)
            image = cv2.putText(image, text,
                                (start[0], start[1] - x,),
                                self.font, self.fontScale, (255, 255, 255),
                                self.fontSize, cv2.LINE_AA)
            # certainty rectangle and text
            text = '{1:.1%}'.format(label, d.certainty)
            text_size = cv2.getTextSize(text, self.font, self.fontScale, self.fontSize)[0]
            image = cv2.rectangle(image,
                                  (start[0] + x, start[1] + x,),
                                  (start[0] + text_size[0] + x, start[1] + text_size[1] + x,),
                                  tuple(color), -1)
            image = cv2.putText(image, text,
                                (start[0] + x, start[1] + text_size[1] + x,),
                                self.font, self.fontScale, (255, 255, 255),
                                self.fontSize, cv2.LINE_AA)
           # # distance rectangle and text
           # text = '{0:.2f} m'.format(d.distance)
           # text_size = cv2.getTextSize(text, self.font, self.fontScale, self.fontSize)[0]
           # image = cv2.rectangle(image,
           #                       (end[0] - text_size[0] - x, end[1] - text_size[1] - x,),
           #                       (end[0] - x, end[1] - x,),
           #                       tuple(color), -1)
           # image = cv2.putText(image, text,
           #                     (end[0] - text_size[0] - x, end[1] - x,),
           #                     self.font, self.fontScale, (255, 255, 255),
           #                     self.fontSize, cv2.LINE_AA)

        msg = CompressedImage()
        msg.header.stamp = img_msg.header.stamp
        msg.format = 'jpeg'
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        self.image_pub.publish(msg)


if __name__ == '__main__':
    vis = VisualizerWrapper()
    rospy.init_node('detection_visualizer')
    rospy.spin()


