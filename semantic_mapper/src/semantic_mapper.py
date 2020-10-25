#!/home/alfierra/anaconda3/envs/ros/bin/python
import os, rospy
import numpy as np
import ros_numpy
import sys

from object_detector.msg import DetectionBundle
from position_estimator.msg import LabeledCluster
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2


class SemanticMap:
    EMPTY_CELL = -1
    MAP_FRAME = '/map'

    def __init__(self, cells_per_meter, size):
        self._map = np.full(
            [int(cells_per_meter*size), int(cells_per_meter*size)],
            fill_value=self.EMPTY_CELL,
            dtype=np.int8,
        )
        self.resolution = 1. / cells_per_meter
        path = os.path.dirname(os.path.abspath(__file__))
        self.labels = open(
            os.path.join(path, 'data', 'coco.names')
        ).read().split('\n')[:-1]
        self.colors = np.random.randint(
            0, 255, size=(len(self.labels), 3), dtype='uint8'
        )
        self.label_to_idx = {label: i for i, label in enumerate(self.labels)}

        print('chair', self.colors[self.label_to_idx['chair']])
        print(type(self.colors[self.label_to_idx['chair']]))
        print('laptop', self.colors[self.label_to_idx['laptop']])

        self.colors[self.label_to_idx['pottedplant']] = np.array([65, 181, 47])
        self.colors[self.label_to_idx['backpack']] = np.array([93, 43, 214])
        self.colors[self.label_to_idx['chair']] = np.array([196, 137, 33])
        self.colors[self.label_to_idx['laptop']] = np.array([227, 14, 35])
        self.colors[self.label_to_idx['cup']] = np.array([250, 149, 0])
        self.colors[self.label_to_idx['bottle']] = np.array([0,0,0])
        self.orgin_offset = self._map.shape[0] // 2

        print('chair', self.colors[self.label_to_idx['chair']])
        print('laptop', self.colors[self.label_to_idx['laptop']])

    def update_map(self, ros_cloud, label):
        points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(ros_cloud)
        map_indices = self.from_global_coordinates_to_map_indices(points)
        label_idx = self.label_to_idx[label]
        self._map[map_indices[:,0], map_indices[:,1]] = label_idx
        print(
            'Map updated with class: ',
            label,
            ' label_idx',
            label_idx,
            ' color',
            self.colors[label_idx]
        )

    def from_global_coordinates_to_map_indices(self, points):
        return (points / self.resolution)[:,:2].astype(int)  + self.orgin_offset

    def from_map_indices_to_global_coordinates(self, indices):
        return (indices - self.orgin_offset) * self.resolution + self.resolution/2

    @staticmethod
    def merge_rgb_fields(data):
        r = np.asarray(data[:,0], dtype=np.uint32)
        g = np.asarray(data[:,1], dtype=np.uint32)
        b = np.asarray(data[:,2], dtype=np.uint32)
        rgb_arr = np.array((r << 16) | (g << 8) | (b << 0), dtype=np.uint32)
        rgb_arr.dtype = np.float32
        return rgb_arr

    def generate_point_cloud(self, z_offset=0.02):
        occupied_indices = np.where(self._map != self.EMPTY_CELL)
        x_coords = self.from_map_indices_to_global_coordinates(occupied_indices[0])
        y_coords = self.from_map_indices_to_global_coordinates(occupied_indices[1])
        rgb_columns = self.colors[self._map[occupied_indices]]
        print(rgb_columns)
        ndarray = np.zeros(
            len(x_coords),
            dtype=[
                ('x', 'f4'),
                ('y', 'f4'),
                ('z', 'f4'),
                ('rgb', 'f4'),
            ]
        )
        ndarray['x'] = x_coords
        ndarray['y'] = y_coords
        ndarray['z'] = z_offset
        ndarray['rgb'] = self.merge_rgb_fields(rgb_columns)
        return ros_numpy.point_cloud2.array_to_pointcloud2(
            ndarray, frame_id=self.MAP_FRAME
        )


class SemanticMapperNode:
    def __init__(self):
        self.detection_sub = rospy.Subscriber(
            '/cluster_extractor/labeled_cluster',
            LabeledCluster,
            self.callback,
            queue_size=10,
            buff_size=2**24,
        )
        self.cloud_publisher = rospy.Publisher(
            '/semantic_mapper/points',
            PointCloud2,
            queue_size=10,
        )
        self.semantic_map = SemanticMap(
            cells_per_meter=40,
            size=20,
        )

    def callback(self, labeled_cluster):
        self.semantic_map.update_map(labeled_cluster.cloud, labeled_cluster.label)
        cloud = self.semantic_map.generate_point_cloud()
        self.cloud_publisher.publish(cloud)


if __name__ == '__main__':
    mapper = SemanticMapperNode()
    rospy.init_node('semantic_mapper')
    rospy.spin()

