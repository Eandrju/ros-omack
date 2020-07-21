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
    node_handle(node_handle), debug(debug)
{
    // subscribers:
    laser_sub_.subscribe(node_handle, "/scan", 1);
    detect_sub_.subscribe(node_handle, "/detector/detection_bundle", 1);
    cloud_sub_.subscribe(node_handle, "/camera/depth/points", 1);

    // msg synchronization:
    sync_.reset(new Sync(MySyncPolicy(10), laser_sub_, detect_sub_, cloud_sub_));
    sync_->registerCallback(boost::bind(&PositionEstimator::callback, this, _1, _2, _3));

    // publishers:
    publisher_ = node_handle.advertise<LocalizedDetection>(
            "/position_estimator/detection", 10);
    if (debug) {
        laser_publisher_ = node_handle.advertise<sensor_msgs::LaserScan>(
                        "/position_estimator/scan", 10);
        cloud_publisher_ = node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
                        "/position_estimator/points", 10);
    }

}

void PositionEstimator::filter_cloud(
    const object_detector::Detection& det,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud
    )
{ 

    cout << "height: " << cloud->height << " width: " << cloud->width << endl;
    /* pcl::PointCloud<pcl::PointXYZRGB>::Ptr sliced_cloud (new pcl::PointCloud<pcl::PointXYZRGB>); */

    std::vector<pcl::PointIndices> indices;
    extract_region_of_interest(cloud, det, indices);

    cout << " size: " << sliced_cloud->size() << endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_frame_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    transform_to_global_frame(sliced_cloud, global_frame_cloud);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    filter_out_ground(global_frame_cloud, filtered_cloud);

    std::vector<pcl::PointIndices> object_indices;
    get_clusters(filtered_cloud, &object_indices);
    
    pcl::ExtractIndices<pcl::PointXYZRGB> extractor;
    extractor.setInputCloud(filtered_cloud);

    cout << "Extracted " << object_indices.size() << " clusters." << endl;

    int flag;
    if (not node_handle.getParam("/detector/cluster/flag", flag)) {
        flag = 1;
    }

    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    output_cloud->header = filtered_cloud->header;

    if (flag){
        for (int i = 0; i < object_indices.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointIndices::Ptr indices(new pcl::PointIndices);
            *indices = object_indices[i];
            extractor.setIndices(indices);
            extractor.filter(*extracted_cluster);
            int r = rand() % 255;
            int g = rand() % 255;
            int b = rand() % 255;
            for (int j=0; j < extracted_cluster->points.size(); ++j) {
                extracted_cluster->points[j].r = r;
                extracted_cluster->points[j].g = g;
                extracted_cluster->points[j].b = b;
            }
            *output_cloud += *extracted_cluster;
            //cloud_publisher_.publish(extracted_cluster);
        }
    } else {
        //cloud_publisher_.publish(filtered_cloud);
    }
    cloud_publisher_.publish(output_cloud);

}

void PositionEstimator::filter_out_ground(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud
    )
{
    float ground_cutoff, ceiling_cutoff;
    if (not node_handle.getParam("/detector/ground_cutoff", ground_cutoff)) {
        ground_cutoff = 0.;
    }
    if (not node_handle.getParam("/detector/ceiling_cutoff", ceiling_cutoff)) {
        ceiling_cutoff = 10.;
    }

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (ground_cutoff, ceiling_cutoff);  
    pass.filter (*output_cloud);
    output_cloud->header = cloud->header;
}

void PositionEstimator::extract_region_of_interest(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    const object_detector::Detection& det,
    pcl::PointIndices* indices
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
    for (int i = y0; i < y1; ++i) {
        for (int j = x0; j < x1; ++j) {
            int idx = i * cloud->width + j;
            pcl::PointXYZRGB point;
            point.x = cloud->points[idx].x;
            point.y = cloud->points[idx].y;
            point.z = cloud->points[idx].z;
            point.r = 200;
            point.b = 100;
            point.g = 0;
            output_cloud->points.push_back(point);
        }
    }
    output_cloud->header = cloud->header;
}


void PositionEstimator::transform_to_global_frame(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud
    )
{
    while (true) {
        try{
            //tf_listener.waitForTransform("odom", "camera_depth_frame", ros::Time::now(), ros::Duration(0.1));
            pcl_ros::transformPointCloud("odom", *cloud, *output_cloud, tf_listener);
            //tf_listener.lookupTransform("odom", "lookUpTransform", ros::Time(), transform);
            //pcl_ros::transformPointCloud(*cloud, *output_cloud, transform);
            break;
        }
        catch(tf::ExtrapolationException& ex){   // have no idea why that's not working
        //catch(...){
            ROS_INFO("Caught an exception");
            ros::Duration(0.01).sleep();
        }
    }
}

void PositionEstimator::get_clusters(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    std::vector<pcl::PointIndices>* object_indices
    )
{
    float tolerance, min_size;
    if (not node_handle.getParam("/detector/cluster/tolerance", tolerance)) {
        tolerance = 0.1;
    }

    if (not node_handle.getParam("/detector/cluster/min_size", min_size)) {
        min_size = 10.;
    }

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclid;
    euclid.setInputCloud(cloud);
    //euclid.setIndices(above_surface_indices);
    euclid.setClusterTolerance(tolerance);
    euclid.setMinClusterSize(min_size);
    euclid.extract(*object_indices);
}
                







tuple<geometry_msgs::PointStamped, float> PositionEstimator::estimate_position(
    const object_detector::Detection& det,
    const sensor_msgs::LaserScan::ConstPtr& scan, 
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
    int w_org, 
    int h_org)
{
    // POINTCLOUD
    int x0_shr = (int)( det.x0 + (1.0 - shrinkage) / 2.0 * det.w );
    int y0_shr = (int)( det.y0 + (1.0 - shrinkage) / 2.0 * det.h );
    int w_shr = (int)det.w * shrinkage;
    int h_shr = (int)det.h * shrinkage;


    float x = 0;
    float y = 0;
    float z = 0;

    for (int i = y0_shr; i < y0_shr + h_shr; ++i) {
        for (int j = x0_shr; j < x0_shr + w_shr; ++j) {
            int idx = i * cloud->width + j;
            if (not isnan(cloud->points[idx].x)) {
                 x = cloud->points[idx].x;
                 y = cloud->points[idx].y;
            }
        }
    }

    if (debug) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
        msg->header.frame_id = cloud->header.frame_id;
        msg->height = h_shr;
        msg->width = w_shr;
        for (int i = y0_shr; i < y0_shr + h_shr; ++i) {
            for (int j = x0_shr; j < x0_shr + w_shr; ++j) {
                int idx = i * cloud->width + j;
                x = cloud->points[idx].x;
                y = cloud->points[idx].y;
                z = cloud->points[idx].z;
                msg->points.push_back(pcl::PointXYZ(x, y, z));
            }
        } 
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        cloud_publisher_.publish(msg);
    }

    geometry_msgs::PointStamped foo_point, map_point;
    if (x != 0 and not isnan(x)){
        foo_point.header.frame_id = cloud->header.frame_id;
        foo_point.header.stamp = scan->header.stamp;
        foo_point.point.x = x;
        foo_point.point.y = y; 
        foo_point.point.z = z;
        map_point = transform_point("map", foo_point);
        return make_tuple(map_point, 0.);
    }
         
    // LIDAR
    float angle_min = scan->angle_min;
    float distance = std::numeric_limits<float>::infinity();
    float angle = -1;

    // TODO change the the way you compute start and angle of lidar analysis
    // get them from cloud
    
    int n = scan->ranges.size();
    float shrinkage_ratio = (float)det.w / (float)w_org * (1. - shrinkage) / 2.;
    float first_ratio = (float)det.x0 / (float)w_org + shrinkage_ratio;
    float second_ratio = (float)(det.x0 + det.w) / (float)w_org - shrinkage_ratio;
    // assuming 360 FOV of lidar and 60 FOV of camera
    // fking stupid 0 idx of lidar is in fornt of robot and 1/4 of idx_max is on his 9 (clock)
    int start_idx = (int)(-n * 1/12 + (1 - second_ratio) * n/6);
    int end_idx = (int)(-n * 1/12 + (1 - first_ratio) * n/6);

    std::vector<int> active_idxs;


    for (int i = start_idx; i < end_idx + 1; i++) {
        float r;
        float a;

        if (i < 0){
            active_idxs.push_back(i+n);
            r = scan->ranges[i+n];  
            a = scan->angle_min + (scan->angle_increment * (i+n));
        } else {
            active_idxs.push_back(i);
            r = scan->ranges[i];  
            a = scan->angle_min + (scan->angle_increment * i);
        }

        if (r < scan->range_max && r > scan->range_min && r < distance) {
            distance = r;
            angle = a;
        }
    }
    
    if (debug) {   
        std::cout << "\nLaser minimal distance: " << distance << std::endl;
        std::cout << "Active points number: " << active_idxs.size() << std::endl;
        std::cout << "angle: " << angle << endl;
        std::cout << "Index from " << start_idx << " to " << end_idx << std::endl;
        std::cout << "\n\n";
        sensor_msgs::LaserScan::Ptr msg (new sensor_msgs::LaserScan);
        msg->header.frame_id = "laser"; 
        msg->header.stamp = scan->header.stamp;
        msg->angle_min = scan->angle_min;
        msg->angle_max = scan->angle_max;
        msg->angle_increment = scan->angle_increment;
        msg->range_min = 0.;
        msg->range_max = scan->range_max;
        for (int i = 0; i < n; i++) {
            // check if idx was previously detected as active (near object)
            if(std::find(active_idxs.begin(), active_idxs.end(), i) != active_idxs.end()){
                msg->ranges.push_back(scan->ranges[i]);
                msg->intensities.push_back(scan->intensities[i]);
            } else {
                msg->ranges.push_back(0.);
                msg->intensities.push_back(0.);
            }
        }
        laser_publisher_.publish(msg);
    }

    geometry_msgs::PointStamped laser_point;
    laser_point.header.frame_id = "laser";
    laser_point.header.stamp = scan->header.stamp;
    laser_point.point.x = distance * cos(angle);
    laser_point.point.y = distance * sin(angle);
    laser_point.point.z = 0;
    map_point = transform_point("map", laser_point);
    return make_tuple(map_point, 0.);
}

void PositionEstimator::callback(
           const sensor_msgs::LaserScan::ConstPtr& scan,
           const DetectionBundle::ConstPtr& bundle_i,
           const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    std::vector<object_detector::Detection> boxes_i = bundle_i->detections;
    for (int i = 0; i < bundle_i->size; ++i) {
        filter_cloud(boxes_i[i], cloud);
    }



    //object_detector::DetectionBundle bundle_o;
    //bundle_o.size = bundle_i->size;
    //bundle_o.header = bundle_i->header;
    //std::vector<object_detector::Detection> boxes_o, boxes_i;
    //boxes_i = bundle_i->detections;

    //for (int i = 0; i < bundle_i->size; ++i) {
    //    LocalizedDetection detection;
    //    detection.class_certainty = boxes_i[i].certainty;
    //    detection.class_name = boxes_i[i].class_name;

    //    auto tuple = estimate_position(
    //        boxes_i[i],
    //        scan,
    //        cloud,
    //        bundle_i->frame_width,
    //        bundle_i->frame_height);

    //    detection.point = get<0>(tuple);
    //    detection.position_certainty = get<1>(tuple);
    //    publisher_.publish(detection);
    //}
}


geometry_msgs::PointStamped PositionEstimator::transform_point(
    string out_frame,
    geometry_msgs::PointStamped in_point)
{
    cout << "Converting point from " << in_point.header.frame_id << " frame to " << out_frame << " frame." << endl;
    geometry_msgs::PointStamped out_point;
    try{
        tf_listener.transformPoint(out_frame, in_point, out_point);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point: %s", ex.what());
    }
    return out_point;
}

