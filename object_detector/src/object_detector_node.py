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

name_to_dtypes = {
        "rgb8":    (np.uint8,  3),
        "rgba8":   (np.uint8,  4),
        "rgb16":   (np.uint16, 3),
        "rgba16":  (np.uint16, 4),
        "bgr8":    (np.uint8,  3),
        "bgra8":   (np.uint8,  4),
        "bgr16":   (np.uint16, 3),
        "bgra16":  (np.uint16, 4),
        "mono8":   (np.uint8,  1),
        "mono16":  (np.uint16, 1),
        # for bayer image (based on cv_bridge.cpp)
        "bayer_rggb8":  (np.uint8,  1),
        "bayer_bggr8":  (np.uint8,  1),
        "bayer_gbrg8":  (np.uint8,  1),
        "bayer_grbg8":  (np.uint8,  1),
        "bayer_rggb16": (np.uint16, 1),
        "bayer_bggr16": (np.uint16, 1),
        "bayer_gbrg16": (np.uint16, 1),
        "bayer_grbg16": (np.uint16, 1),

        # OpenCV CvMat types
        "8UC1":    (np.uint8,   1),
        "8UC2":    (np.uint8,   2),
        "8UC3":    (np.uint8,   3),
        "8UC4":    (np.uint8,   4),
        "8SC1":    (np.int8,    1),
        "8SC2":    (np.int8,    2),
        "8SC3":    (np.int8,    3),
        "8SC4":    (np.int8,    4),
        "16UC1":   (np.uint16,   1),
        "16UC2":   (np.uint16,   2),
        "16UC3":   (np.uint16,   3),
        "16UC4":   (np.uint16,   4),
        "16SC1":   (np.int16,  1),
        "16SC2":   (np.int16,  2),
        "16SC3":   (np.int16,  3),
        "16SC4":   (np.int16,  4),
        "32SC1":   (np.int32,   1),
        "32SC2":   (np.int32,   2),
        "32SC3":   (np.int32,   3),
        "32SC4":   (np.int32,   4),
        "32FC1":   (np.float32, 1),
        "32FC2":   (np.float32, 2),
        "32FC3":   (np.float32, 3),
        "32FC4":   (np.float32, 4),
        "64FC1":   (np.float64, 1),
        "64FC2":   (np.float64, 2),
        "64FC3":   (np.float64, 3),
        "64FC4":   (np.float64, 4)
}

def image_to_numpy(msg):
    dtype_class, channels = name_to_dtypes[msg.encoding]
    dtype = np.dtype(dtype_class)
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    shape = (msg.height, msg.width, channels)

    data = np.fromstring(msg.data, dtype=dtype).reshape(shape)
    data.strides = (
        msg.step,
        dtype.itemsize * channels,
        dtype.itemsize
    )

    if channels == 1:
        data = data[...,0]
    return data

class DetectorWrapper:
    def __init__(self, debug=False):
        self.last_callback = None
        self.debug = debug
        self.detector = Detector(
            conf_thresh=0.65,
            resolution=480,
        )
        self.bundle_pub = rospy.Publisher(
            '/detector/detection_bundle', DetectionBundle, queue_size=1
        )
        image_input = rospy.get_param(
            '/object_detector/image_input',
            '/camera/rgb/image_rect_color/compressed'
        )

        self.subscriber = rospy.Subscriber(
            image_input,
            CompressedImage,
            self.callback,
            queue_size=1,
            buff_size=2**24,
        )
        print(f'Image input endpoint {image_input}')

    def callback(self, msg):
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
        arr = np.fromstring(msg.data, np.uint8)
        # image = image_to_numpy(msg)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
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
            debug_class_filter = rospy.get_param('/debug_class', 0)
            if debug_class_filter and d.class_name != debug_class_filter:
                continue

            ignored_class = rospy.get_param('/debug_ignore_class', 'human')
            if d.class_name == ignored_class:
                continue

            color = self.detector.colors[int(box[-1])]
            d.color.r = int(color[0])
            d.color.g = int(color[1])
            d.color.b = int(color[2])
            bundle.detections.append(d)

        bundle.size = len(bundle.detections)
        bundle.header = msg.header
        bundle.frame_height = image.shape[0]
        bundle.frame_width = image.shape[1]


        if len(bundle.detections) > 0:
            self.bundle_pub.publish(bundle)

        cycle_lag = rospy.get_param(
            '/object_detector/cycle_lag', 0
        )
        rospy.sleep(cycle_lag)


if __name__ == '__main__':
    rospy.init_node('object_detector', anonymous=True)
    d = DetectorWrapper(debug=True)
    rospy.spin()




