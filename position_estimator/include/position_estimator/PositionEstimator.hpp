#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "visualization_msgs/Marker.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <object_detector/DetectionBundle.h>
#include <position_estimator/LabeledCluster.h>


/* #include <cv_bridge/cv_bridge.h> */
/* #include <opencv2/imgproc/imgproc.hpp> */
/* #include <opencv2/highgui/highgui.hpp> */
/* #include <image_transport/image_transport.h> */

#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>


#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/buffer_client.h>
#include <tf2_ros/transform_listener.h>


#include <ros/ros.h>
#include <vector>
#include <string>
#include <tuple>
#include <math.h>
#include <time.h>

using PointType = pcl::PointXYZRGB;

struct BoundingBox {
    int x0;
    int x1;
    int y0;
    int y1;
};

class PositionEstimator {
    public:
        PositionEstimator(ros::NodeHandle&, bool);

        void callback(
            const object_detector::DetectionBundle::ConstPtr& bundle_i,
            const pcl::PointCloud<PointType>::ConstPtr& cloud);

        void callback2(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);

    private:
        void remove_possible_walls(
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            boost::shared_ptr<std::vector<int> >& roi_indices,
            boost::shared_ptr<std::vector<int> >& output_indices
        );

        int choose_cluster (
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            std::vector<pcl::PointIndices>* object_indices,
            const object_detector::Detection& det
        );
        void map_cloud_to_ground(
            const pcl::PointCloud<PointType>::ConstPtr& extracted_cluster,
            pcl::PointCloud<PointType>::Ptr& ouput
        );

        void get_clusters(
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            boost::shared_ptr<std::vector<int> >& input_indices,
            std::vector<pcl::PointIndices>* object_indices
       );
        void extract_region_of_interest(
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            const object_detector::Detection& det,
            boost::shared_ptr<std::vector<int> >& indices
        );
        bool remove_ground(
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            boost::shared_ptr<std::vector<int> >& input_indices,
            boost::shared_ptr<std::vector<int> >& output_indices
        );

        void remove_nans(
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            boost::shared_ptr<std::vector<int> >& input_indices,
            boost::shared_ptr<std::vector<int> >& output_indices
        );
        void filter_cloud(
            const object_detector::Detection& det,
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            pcl::PointCloud<PointType>::Ptr& extracted_cluster
        );
        void visualize_clusters(
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            std::vector<pcl::PointIndices> clusters_indices
        );
        bool is_it_nan_free(
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            boost::shared_ptr<std::vector<int> >& indices
        );
        bool is_plane_perpendicular_to_the_floor(
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            pcl::ModelCoefficients::Ptr coefficients
        );
        bool are_outliers_between_plane_and_robot(
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            pcl::ModelCoefficients::Ptr coefficients,
            boost::shared_ptr<std::vector<int> >& outliers
        );

        bool is_point_lying_in_space_between_plane_and_robot(
            pcl::ModelCoefficients::Ptr coefficients,
            PointType p
        );

        void visualize_sub_cloud(
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            boost::shared_ptr<std::vector<int> >& indices,
            std::tuple<int,int,int> color,
            int k
        );

        void visualize_sub_cloud(
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            pcl::PointIndices object_indices,
            std::tuple<int,int,int> color,
            int k
        );
        void downsample(
            const pcl::PointCloud<PointType>::ConstPtr& cloud,
            boost::shared_ptr<std::vector<int> >& input_indices,
            boost::shared_ptr<std::vector<int> >& output_indices
        );


        bool is_normal_vector_pointing_towards_space_in_between_plane_and_robot(
            pcl::ModelCoefficients::Ptr coefficients
        );

        float compute_IOU_metric(
            BoundingBox box1,
            BoundingBox box2
        );

        geometry_msgs::PointStamped transform_point(
            std::string out_frame,
            geometry_msgs::PointStamped point);
        message_filters::Subscriber< pcl::PointCloud<PointType> > cloud_sub_;
        message_filters::Subscriber<object_detector::DetectionBundle> detect_sub_;

        typedef message_filters::sync_policies::ApproximateTime<
            object_detector::DetectionBundle,
            pcl::PointCloud<PointType>> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;
        ros::Publisher publisher_;
        ros::Publisher vis_pub;
        ros::NodeHandle node_handle;
        ros::Publisher cloud_publisher_;
        ros::Publisher rgb_cloud_publisher;
        ros::Publisher rgb_cloud_publisher2;
        ros::Publisher rgb_cloud_publisher3;
        ros::Publisher rgb_cloud_publisher4;
        ros::Publisher rgb_cloud_publisher5;
        ros::Publisher rgb_cloud_publisher6;
        ros::Subscriber subb;
        tf::TransformListener tf_listener_v1;
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tfListener;
        float shrinkage;
        bool debug;
};

