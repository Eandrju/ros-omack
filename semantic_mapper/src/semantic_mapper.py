#!/usr/bin/env python
import os, rospy
import numpy as np

from object_detector.msg import DetectionBundle
from position_estimator.msg import Detection
from geometry_messages.msg import StamptedPoint
from visualization_messages.msg import MarkerArray

class Landmark:
    def __init__(point):
        self.center = point
        self._cvx_hull_arcs = []

    def add_point(point):
        if len(self._cvx_hull_arcs) == 0:
            self._conver_hull.append([point, self._center])
        elif len(self._cvx_hull_arcs) == 1:
            arc = self._cvx_hull_arcs[0]
            if self._check_if_point_is_to_the_right(
                point, arc
            ):
                self._cvx_hull_arcs[0] = [arc[1], arc[0]]
                self._cvx_hull_arcs += [
                    [arc[0], point],
                    [point, arc[1]],
                ]
            else:
                self._cvx_hull_arcs += [
                    [arc[1], point],
                    [point, arc[0]],
                ]
        else:
            # first let's find the first inner arc (if idx zero arc is inner we will iterate back)
            idx = 0
            if self._check_if_point_is_to_the_right(point, self._cvx_hull_arcs[0]):
                while self._check_if_point_is_to_the_right(
                    point, _cvx_hull_arcs[idx-1]
                ):
                    idx -= 1
            else:
                idx = 1
                while not self._check_if_point_is_to_the_right(
                    point, _cvx_hull_arcs[idx+1]
                ):
                    idx += 1

            first_new_arc =  [self._cvx_hull_arcs[idx][0], point]

            # once first inner arc is identified we can start removing them
            while self._check_if_point_is_to_the_right(idx):
                self._cvx_hull_arcs.pop(idx)

            second_new_arc = [point, self._cvx_hull_arcs[idx][0]]
            self._cvx_hull_arcs.inser(idx, second_new_arc)
            self._cvx_hull_arcs.inser(idx, first_new_arc)


    def _check_if_point_is_to_the_right(point, arc):
        p = point
        a0 = arc[0]
        a1 = arc[1]
        return (a1.x - a0.x)*(p.y - a0.y) - (a1.y - a0.y)*(p.x - a0.x)) >= 0;

class VisualizerWrapper:
    def __init__(self):
        self.detection_sub = rospy.Subscriber(
            "/position_estimator/detection",
            Detection,
            self.callback,
            queue_size=10,
            buff_size=2**24,
        )

        self.markers_pub = rospy.Publisher(
            '/semantic_mapper/visualization_marker_array',
            MarkerArray,
            queue_size=2,
        )

    def callback(self, detection):

        self.update_map(detection)

        arr = np.fromstring(img_msg.data, np.uint8)
        image = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        np.random.seed(69)
        print(type(image))


        msg = CompressedImage()
        msg.header.stamp = img_msg.header.stamp
        msg.format = 'jpeg'
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        self.image_pub.publish(msg)

    def update_map(self, detection):




# TODO chekc whether there is any object at that position
#  - if yes check whether it's the same
#      - if yes update slightly (how slighly it's positon - maybe proportionaly to what?)
#      - if not resolve conflict of class - based on their's class certaintity
#  - if not add an object at that place (at that exact place? or maybe close?)
#
#
# TODO each object on map is represented by convex hull set of points - this way we won't suffer from moving average
# cumulative error, each time a new point is supposed to be added we first check whether it's part of convex hull if
# not we disard it, we can also discard points during insertion


if __name__ == '__main__':
    mapper = SemanticMapper()
    rospy.init_node('semantic_mapper')
    rospy.spin()

