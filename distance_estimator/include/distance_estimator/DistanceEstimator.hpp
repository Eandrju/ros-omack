#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "visualization_msgs/Marker.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <object_detector/DetectionBundle.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <math.h>


class DistanceEstimator {
    public:
        DistanceEstimator(ros::NodeHandle& nodeHandle, bool debug);

        void callback(const sensor_msgs::LaserScan::ConstPtr& scan,
                      const object_detector::DetectionBundle::ConstPtr& bundle_i,
                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

    private:
        float Estimate(const object_detector::Detection& det,
                       const sensor_msgs::LaserScan::ConstPtr& scan,
                       const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                       int w_org, int h_org);
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
        message_filters::Subscriber< pcl::PointCloud<pcl::PointXYZ> > cloud_sub_;
        message_filters::Subscriber<object_detector::DetectionBundle> detect_sub_;

        typedef message_filters::sync_policies::ApproximateTime<
                    sensor_msgs::LaserScan,
                    object_detector::DetectionBundle,
                    pcl::PointCloud<pcl::PointXYZ>> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
        ros::Publisher publisher_;
        ros::NodeHandle nodeHandle_;
        ros::Publisher laser_publisher_;
        ros::Publisher cloud_publisher_;
        float shrinkage;
        bool debug_;
};

