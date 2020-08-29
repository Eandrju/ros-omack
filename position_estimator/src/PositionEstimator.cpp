#include "position_estimator/PositionEstimator.hpp"

//using namespace sensor_msgs;
using namespace object_detector;
using namespace message_filters;
using namespace position_estimator;
using namespace geometry_msgs;
using namespace std;

PositionEstimator::PositionEstimator(
    ros::NodeHandle& node_handle,
    bool debug=false
):
    node_handle(node_handle), debug(debug), tfListener(tf_buffer)
{
    // subscribers:
    std::string cloud_endpoint, laser_endpoint, detector_endpoint;
    node_handle.param<std::string>("/cloud_endpoint", cloud_endpoint, "/camera/depth/points");
    cloud_sub_.subscribe(node_handle, cloud_endpoint, 1);
    node_handle.param<std::string>("/laser_endpoint", laser_endpoint, "/scan");
    laser_sub_.subscribe(node_handle, laser_endpoint, 1);
    node_handle.param<std::string>("/detector_endpoint", detector_endpoint, "/detector/detection_bundle");
    detect_sub_.subscribe(node_handle, detector_endpoint, 1);
    cout<<cloud_endpoint<<" "<<laser_endpoint<<" "<<detector_endpoint<<endl;

    subb = node_handle.subscribe(cloud_endpoint, 1, &PositionEstimator::callback2, this);

    // msg synchronization:
    sync_.reset(new Sync(MySyncPolicy(10), laser_sub_, detect_sub_, cloud_sub_));
    sync_->registerCallback(boost::bind(&PositionEstimator::callback, this, _1, _2, _3));

    // publishers:
    publisher_ = node_handle.advertise<LocalizedDetection>(
            "/position_estimator/detection", 10);


    if (debug) {
        laser_publisher_ = node_handle.advertise<sensor_msgs::LaserScan>(
                        "/position_estimator/scan", 10);
        cloud_publisher_ = node_handle.advertise<pcl::PointCloud<PointType>>(
                        "/position_estimator/points", 10);
        cloud_publisher2_ = node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
                        "/position_estimator/points2", 10);
    }
    ros::Duration(2).sleep();

}

void PositionEstimator::filter_cloud(
    const object_detector::Detection& det,
    const pcl::PointCloud<PointType>::ConstPtr& cloud
    )
{
    bool debug1;
    if (not node_handle.getParam("/debug1", debug1)) {
        debug1 = 1;
    }
 
    cout << "========================" << endl;
    cout << "cloud size: " << cloud->size () << endl;
    // extract region surrounding detected object
    boost::shared_ptr<vector<int> > roi_indices (new vector<int>);
    extract_region_of_interest (cloud, det, roi_indices);

    cout << "roi cloud size: " << roi_indices->size () << endl;
    // discard points being part of ground or ceiling
    boost::shared_ptr<vector<int> > no_ground_indices (new vector<int>);
    if (not remove_ground (cloud, roi_indices, no_ground_indices)) return;

    cout << "ground excluded cloud size: " << no_ground_indices->size () << endl;
    // discard walls
    boost::shared_ptr<vector<int> > no_walls_indices (new vector<int>);
    
    if (debug1) {
        remove_possible_walls(cloud, no_ground_indices, no_walls_indices);
    }
    else {
        no_walls_indices = no_ground_indices;
    }

    cout << "walls excluded cloud size: " << no_walls_indices->size () << endl;
    // using euclidean clusterization group points
    std::vector<pcl::PointIndices> clusters_indices;
    get_clusters (cloud, no_walls_indices, &clusters_indices);
    if (clusters_indices.size () == 0) return;

    // visualize purpose
    visualize_clusters(cloud, clusters_indices);
    
    /* for (int i = 0; i < clusters_indices.size (); ++i){ */
    /*       pcl::PointCloud<PointType>::Ptr  extracted_cluster (new pcl::PointCloud<PointType>); */
    /*       *indices = clusters_indices[i]; */
    /*       extractor.setIndices(indices); */
    /*       extractor.filter(*extracted_cluster); */
    /*   } */
    /*   cloud_publisher_.publish(extracted_cluster); */


    /* cout << "number of clusters: " << clusters_indices.size () << endl; */
    /* // choose cluster most likely to be part of detected object */
    /* int cluster_idx = choose_cluster (cloud, &clusters_indices, det); */

    /* cout << "choosen object cloud size: " << clusters_indices[cluster_idx].indices.size () << endl; */
    /* // extract choosen points and publish them */
    /* pcl::PointCloud<PointType>::Ptr  extracted_cluster (new pcl::PointCloud<PointType>); */
    /* pcl::ExtractIndices<PointType> extractor; */
    /* extractor.setInputCloud(cloud); */
    /* pcl::PointIndices::Ptr indices(new pcl::PointIndices); */
    /* *indices = clusters_indices[cluster_idx]; */
    /* extractor.setIndices(indices); */
    /* extractor.filter(*extracted_cluster); */
    /* cloud_publisher_.publish(extracted_cluster); */
}


void PositionEstimator::visualize_clusters(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    std::vector<pcl::PointIndices> clusters_indices
    )
{
    pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
    output_cloud->header = cloud->header;

    pcl::ExtractIndices<PointType> extractor;
    extractor.setInputCloud(cloud);
    for (int i = 0; i < clusters_indices.size(); ++i) {
        pcl::PointCloud<PointType>::Ptr extracted_cluster(new pcl::PointCloud<PointType>);
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        *indices = clusters_indices[i];
        extractor.setIndices(indices);
        extractor.filter(*extracted_cluster);
        int r = rand() % 255;
        int g = rand() % 255;
        int b = rand() % 255;
        for (int j=0; j < extracted_cluster->points.size(); ++j) {
            /* extracted_cluster->points[j].r = r; */
            /* extracted_cluster->points[j].g = g; */
            /* extracted_cluster->points[j].b = b; */
        }
        *output_cloud += *extracted_cluster;
    }

    cloud_publisher_.publish(output_cloud);
}


int PositionEstimator::choose_cluster (
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    std::vector<pcl::PointIndices>* objects_indices,
    const object_detector::Detection& det
    )

{
    int width = cloud->width;
    int height = cloud->height;
    int best_cluster_idx;
    int y_min, y_max, x_min, x_max, x_idx, y_idx;
    float minimal_error = std::numeric_limits<float>::infinity();
    float error;

    for (int i = 0; i < objects_indices->size(); ++i) {
        /* pcl::PointIndices obj_indices = (*objects_indices)[i]; */
        y_min= height;
        y_max= 0;
        x_min= width;
        x_max= 0;

        for (int j = 0; j < (*objects_indices)[i].indices.size(); ++j) {
            y_idx = (*objects_indices)[i].indices[j] / width;
            x_idx = (*objects_indices)[i].indices[j] % width;

            y_min = (y_idx < y_min) ? y_idx: y_min;
            y_max = (y_idx > y_max) ? y_idx: y_max;
            x_min = (x_idx < x_min) ? x_idx: x_min;
            x_max = (x_idx > x_max) ? x_idx: x_max;

            error = (det.w - (x_max - x_min)) * (det.w - (x_max - x_min)) +
                (det.h - (y_max - y_min)) * (det.h - (y_max - y_min));
            if (error < minimal_error) {
                minimal_error = error;
                best_cluster_idx = i;
            }
        }
    }
    return best_cluster_idx;
}


void PositionEstimator::remove_possible_walls(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    boost::shared_ptr<vector<int> >& input_indices,
    boost::shared_ptr<vector<int> >& output_indices
    )
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    // Fit a plane
    seg.setInputCloud(cloud);
    seg.setIndices(input_indices);
    seg.segment(*inliers, *coefficients);

    // Remove wall indices from ROI
    int i = 0;
    for (int r = 0; r < input_indices->size(); ++r) {
        int roi_idx = (*input_indices)[r];
        int inlier_idx = inliers->indices[i];
        if (roi_idx == inlier_idx) {
            i++;
            if (i >= inliers->indices.size()) {
                break;
            }
        }
        else {
            output_indices->push_back(roi_idx);
        }
     }
    cout << "Output size: " << output_indices->size() <<
        " should be: " << input_indices->size() - inliers->indices.size() << endl;
    cout << "Input size: " << input_indices->size() << " inliers size: " <<
        inliers->indices.size() << " removed : " << i << endl;
}


bool PositionEstimator::remove_ground(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    boost::shared_ptr<vector<int> >& roi_indices,
    boost::shared_ptr<vector<int> >& output_indices
    )
{
    geometry_msgs::TransformStamped transform;

    try{
        transform = tf_buffer.lookupTransform ("odom", cloud->header.frame_id, ros::Time(0));
    }
    catch(tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }

    float ground_cutoff, ceiling_cutoff;
    if (not node_handle.getParam("/detector/ground_cutoff", ground_cutoff)) {
        ground_cutoff = 0.;
    }
    if (not node_handle.getParam("/detector/ceiling_cutoff", ceiling_cutoff)) {
        ceiling_cutoff = 10.;
    }

    geometry_msgs::Quaternion q = transform.transform.rotation;
    Eigen::Quaternionf rotation (q.w ,q.x ,q.y ,q.z );
    geometry_msgs::Vector3 v = transform.transform.translation;
    Eigen::Translation<float, 3> translation (v.x, v.y, v.z);
    Eigen::Transform<float, 3, Eigen::Affine> t (translation * rotation);
    for (size_t i = 0; i < roi_indices->size (); ++i) {
        int idx = (*roi_indices)[i];
        if (!isfinite (cloud->points[idx].x) ||
            !isfinite (cloud->points[idx].y) ||
            !isfinite (cloud->points[idx].z))
        continue;

        Eigen::Matrix<float, 3, 1> pt (cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
        /* cout << cloud->points[idx].x << " should be " << pt.coeffRef(0) << endl << endl; */
        /* /1* cout << "transform used: " << t(2, 0) << " " << t(2, 1) << " " << t(2, 2) << endl; *1/ */
        /* cout << "rotation: " << q.w << " " << q.x << " " << q.y << " " << q.z << endl; */
        /* cout << "translation: " << v.x << " " << v.y << " " << v.z << endl; */
        /* cout << t(0, 0) << " " << t(0, 1) << " " << t(0, 2) << " " << t(0, 3) << endl; */
        /* cout << t(1, 0) << " " << t(1, 1) << " " << t(1, 2) << " " << t(1, 3) << endl; */
        /* cout << t(2, 0) << " " << t(2, 1) << " " << t(2, 2) << " " << t(2, 3) << endl; */

        float z = static_cast<float> (t (2, 0) * pt.coeffRef (0) + t (2, 1) * pt.coeffRef (1) + t (2, 2) * pt.coeffRef (2) + t (2, 3));
        if (z > ground_cutoff && z < ceiling_cutoff) {
            output_indices->push_back(idx);
        }
    }
    return true;
}


void PositionEstimator::extract_region_of_interest(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    const object_detector::Detection& det,
    boost::shared_ptr<vector<int> >& roi_indices
    )
{
    int x0, x1, y0, y1;
    int width = cloud->width;
    int height = cloud->height;
    float shrinkage_ratio;

    if (not node_handle.getParam("/detector/shrinkage_ratio", shrinkage_ratio)) {
        shrinkage_ratio = 1.;
    }

    x0 = (int)( det.x0 + (1. - shrinkage_ratio) / 2. * det.w );
    x1 = (int)( det.x0 + det.w - (1. - shrinkage_ratio) / 2. * det.w);
    y0 = (int)( det.y0 + (1. - shrinkage_ratio) / 2. * det.h );
    y1 = (int)( det.y0 + det.h - (1. - shrinkage_ratio) / 2. * det.h);

    x0 = (x0 < 0) ? 0: x0;
    x1 = (x1 >= width) ? width-1: x1;
    y0 = (y0 < 0) ? 0: y0;
    y1 = (y1 >= height) ? height-1: y1;


    pcl::ExtractIndices<PointType> extractor;
    extractor.setInputCloud(cloud);
    extractor.setIndices(y0, x0, y1-y0, x1-x0);

    for (int i = y0; i < y1; ++i) {
        for (int j = x0; j < x1; ++j) {
            int idx = i * cloud->width + j;
            roi_indices->push_back(idx);
        }
    }
}


void PositionEstimator::get_clusters(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    boost::shared_ptr<vector<int> >& input_indices,
    std::vector<pcl::PointIndices>* object_indices
    )
{
    float tolerance, min_size;
    if (not node_handle.getParam("/detector/cluster/tolerance", tolerance)) {
        tolerance = 0.01;
    }

    if (not node_handle.getParam("/detector/cluster/min_size", min_size)) {
        min_size = 100.;
    }

    pcl::EuclideanClusterExtraction<PointType> euclid;
    euclid.setInputCloud(cloud);
    euclid.setIndices(input_indices);
    euclid.setClusterTolerance(tolerance);
    euclid.setMinClusterSize(min_size);
    euclid.extract(*object_indices);
}



void PositionEstimator::callback(
           const sensor_msgs::LaserScan::ConstPtr& scan,
           const DetectionBundle::ConstPtr& bundle_i,
           const pcl::PointCloud<PointType>::ConstPtr& cloud)
{
    std::vector<object_detector::Detection> boxes_i = bundle_i->detections;
    for (int i = 0; i < bundle_i->size; ++i) {
        std::string classname;
        /* if (not node_handle.getParam("/debug1", classname)) { */
        /*    classname = "pottedplant"; */
        /* } */
        /* if (boxes_i[i].class_name != classname) continue; */
        filter_cloud(boxes_i[i], cloud);
    }
}



void PositionEstimator::callback2(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  out (new pcl::PointCloud<pcl::PointXYZRGB>);
    out->header = cloud->header;
    for (int i = 0; i < cloud->points.size(); ++i) {
        pcl::PointXYZRGB point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        point.r = cloud->points[i].b;
        point.g = cloud->points[i].g;
        point.b = cloud->points[i].r;
        out->points.push_back(point);
    }
    cloud_publisher2_.publish(out);
}


