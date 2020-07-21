#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "visualization_msgs/Marker.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <object_detector/DetectionBundle.h>
#include <position_estimator/LocalizedDetection.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <tuple>
#include <math.h>
#include <time.h>


class PositionEstimator {
    public:
        PositionEstimator(ros::NodeHandle&, bool);

        void callback(const sensor_msgs::LaserScan::ConstPtr& scan,
            const object_detector::DetectionBundle::ConstPtr& bundle_i,
            const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);

    private:
        void get_clusters(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
            std::vector<pcl::PointIndices>* object_indices
        );
        void transform_to_global_frame(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        );
        void extract_region_of_interest(
            const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
            const object_detector::Detection& det,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud
        );
        void filter_out_ground(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud
        );
        void filter_cloud(
            const object_detector::Detection& det,
            const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud
        );
        std::tuple<geometry_msgs::PointStamped, float> estimate_position(
            const object_detector::Detection& det,
            const sensor_msgs::LaserScan::ConstPtr& scan,
            const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
            int w_org, int h_org);
        geometry_msgs::PointStamped transform_point(
            std::string out_frame, 
            geometry_msgs::PointStamped point);
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
        message_filters::Subscriber< pcl::PointCloud<pcl::PointXYZRGB> > cloud_sub_;
        message_filters::Subscriber<object_detector::DetectionBundle> detect_sub_;

        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::LaserScan,
            object_detector::DetectionBundle,
            pcl::PointCloud<pcl::PointXYZRGB>> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
        ros::Publisher publisher_;
        ros::NodeHandle node_handle;
        ros::Publisher laser_publisher_;
        ros::Publisher cloud_publisher_;
        tf::TransformListener tf_listener;
        float shrinkage;
        bool debug;
};

