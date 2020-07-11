#include "position_estimator/PositionEstimator.hpp"

//using namespace sensor_msgs;
using namespace object_detector;
using namespace message_filters;
using namespace position_estimator;
using namespace std;

PositionEstimator::PositionEstimator(ros::NodeHandle& nodeHandle, bool debug=false) :
    nodeHandle_(nodeHandle), debug(debug)
{
    // subscribers:
    laser_sub_.subscribe(nodeHandle, "/scan", 1);
    detect_sub_.subscribe(nodeHandle, "/detector/detection_bundle", 1);
    cloud_sub_.subscribe(nodeHandle, "/camera/depth/points", 1);

    // msg synchronization:
    sync_.reset(new Sync(MySyncPolicy(10), laser_sub_, detect_sub_, cloud_sub_));
    sync_->registerCallback(boost::bind(&PositionEstimator::callback, this, _1, _2, _3));

    // publishers:
    publisher_ = nodeHandle.advertise<LocalizedDetection>(
            "/position_estimator/detection", 10);
    if (debug) {
        laser_publisher_ = nodeHandle.advertise<sensor_msgs::LaserScan>(
                        "/position_estimator/scan", 10);
        cloud_publisher_ = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>(
                        "/position_estimator/points", 1);
    }

    // other
    shrinkage = 0.5; // parameter tuning shrinkage of box before distance estimation
}

// TODO decide which frame is used when returning point laser or camera, rather importand TODO

tuple<float, float, float> PositionEstimator::estimate_position(
    const object_detector::Detection& det,
    const sensor_msgs::LaserScan::ConstPtr& scan, 
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    int w_org, 
    int h_org)
{
    // POINTCLOUD
    int x0_shr = (int)( det.x0 + (1.0 - shrinkage) / 2.0 * det.w );
    int y0_shr = (int)( det.y0 + (1.0 - shrinkage) / 2.0 * det.h );
    int w_shr = (int)det.w * shrinkage;
    int h_shr = (int)det.h * shrinkage;

    float sum_x = 0.;
    float sum_y = 0.;
    float counter = 0;
    for (int i = y0_shr; i < y0_shr + h_shr; ++i) {
        for (int j = x0_shr; j < x0_shr + w_shr; ++j) {
            int idx = i * cloud->width + j;
            if (not isinf(cloud->points[idx].x)) {
                sum_x += cloud->points[idx].x;
                sum_y += cloud->points[idx].y;
                counter++;
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
                float x = cloud->points[idx].x;
                float y = cloud->points[idx].y;
                float z = cloud->points[idx].z;
                msg->points.push_back(pcl::PointXYZ(x, y, z));
            }
        } 
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        cloud_publisher_.publish(msg);
    }

    if (counter != 0)
        return make_tuple(sum_x/counter, sum_y/counter, 0.);
         
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
            angle = scan->range_min + (scan->angle_increment * (i+n));
        } else {
            active_idxs.push_back(i);
            r = scan->ranges[i];  
            angle = scan->range_min + (scan->angle_increment * i);
        }

        if (r < scan->range_max && r > scan->range_min && r < distance) {
            distance = r;
            angle = a;
        }
    }
    
    if (debug) {   
        std::cout << "\nLaser minimal distance: " << distance << std::endl;
        std::cout << "Active points number: " << active_idxs.size() << std::endl;
        std::cout << "angle min: " << scan->angle_min << " , angle max: "; 
        std::cout << scan->angle_max << ", angle inc: " << scan->angle_increment << std::endl;
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
    return make_tuple(distance * cos(angle), distance * sin(angle), 0);
}

void PositionEstimator::callback(
           const sensor_msgs::LaserScan::ConstPtr& scan,
           const DetectionBundle::ConstPtr& bundle_i,
           const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    object_detector::DetectionBundle bundle_o;
    bundle_o.size = bundle_i->size;
    bundle_o.header = bundle_i->header;
    std::vector<object_detector::Detection> boxes_o, boxes_i;
    boxes_i = bundle_i->detections;

    for (int i = 0; i < bundle_i->size; ++i) {
        LocalizedDetection detection;
        detection.class_certainty = boxes_i[i].certainty;
        detection.class_name = boxes_i[i].class_name;

        auto tuple = estimate_position(
            boxes_i[i],
            scan,
            cloud,
            bundle_i->frame_width,
            bundle_i->frame_height);
        geometry_msgs::PointStamped point;
        point.header.frame_id = "laser";
        point.header.stamp = ros::Time();
        point.point.x = get<0>(tuple);
        point.point.y = get<1>(tuple);
        point.point.z = 0.;

        detection.point = point;
        detection.position_certainty = get<2>(tuple);
        publisher_.publish(detection);
        if(debug) {
            cout << "Estimated position: " << point.point.x << point.point.y << endl;
        }
    }
}

