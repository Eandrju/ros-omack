#!/home/alfierra/anaconda3/envs/ros/bin/python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from object_detector.msg import DetectionBundle
from object_detector.msg import Detection
from gpu_yolo.gpu_yolo import Detector
#from ssd.ssd import Detector
#from yolo.yolo import Detector
#from frcnn.frcnn import Detector

class DetectorWrapper:
    def __init__(self, debug=False):
        self.last_callback = None
        self.debug = debug
        self.detector = Detector(
            conf_thresh=0.35,
            resolution=320,
        )
        self.bundle_pub = rospy.Publisher(
            '/detector/detection_bundle', DetectionBundle, queue_size=1
        )
        self.image_pub = rospy.Publisher(
            '/detector/image_raw/compressed', CompressedImage, queue_size=1
        )

        self.subscriber = rospy.Subscriber(
            '/camera/rgb/image_raw/compressed',
            CompressedImage,
            self.callback,
            queue_size=1,
            buff_size=2**24,
        )

    def callback(self, ros_data):
        # update callabak timestamp
        if self.debug and not rospy.is_shutdown():
            current_time = rospy.get_time()
            if self.last_callback is None:
                self.last_callback = current_time
            else:
                framerate = 1 / (current_time - self.last_callback)
                print(f'Object detection node framerate: {framerate}')
                self.last_callback = current_time

        # get image
        arr = np.fromstring(ros_data.data, np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        print(image.shape)
        self.detector.resolution = image.shape[0]

        # detection
        result, result_image = self.detector.detectObjects(image)

        bundle = DetectionBundle()

        for box in result:
            d = Detection()
            d.x0 = int(box[1])
            d.y0 = int(box[2])
            d.w  = int(box[3]) - d.x0
            d.h  = int(box[4]) - d.y0

            if d.w == 0 or d.h == 0:
                continue

            d.certainty =  float(box[5])
            d.class_name = self.detector.classes[int(box[-1])]
            color = self.detector.colors[int(box[-1])]
            d.color.r = int(color[0])
            d.color.g = int(color[1])
            d.color.b = int(color[2])
            bundle.detections.append(d)

        bundle.size = len(result)
        bundle.header = ros_data.header
        bundle.frame_height = image.shape[0]
        bundle.frame_width = image.shape[1]


        if len(bundle.detections) > 0:
            self.bundle_pub.publish(bundle)

        # create message
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = 'jpeg'
        msg.data = np.array(cv2.imencode('.jpg', result_image)[1]).tostring()
        self.image_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('object_detector', anonymous=True)
    d = DetectorWrapper(debug=True)
    rospy.spin()


