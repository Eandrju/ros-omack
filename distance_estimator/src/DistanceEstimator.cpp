#include "distance_estimator/DistanceEstimator.hpp"

//using namespace sensor_msgs;
using namespace object_detector;
using namespace message_filters;

DistanceEstimator::DistanceEstimator(ros::NodeHandle& nodeHandle, bool debug=false) :
    nodeHandle_(nodeHandle), debug_(debug)
{
    // subscribers:
    laser_sub_.subscribe(nodeHandle, "/scan", 1);
    detect_sub_.subscribe(nodeHandle, "/detector/detection_bundle", 1);
    cloud_sub_.subscribe(nodeHandle, "/camera/depth/points", 1);

    // msg synchronization:
    sync_.reset(new Sync(MySyncPolicy(10), laser_sub_, detect_sub_, cloud_sub_));
    sync_->registerCallback(boost::bind(&DistanceEstimator::callback, this, _1, _2, _3));

    // publishers:
    publisher_ = nodeHandle.advertise<DetectionBundle>(
            "/distance_estimator/detection_bundle", 10);
    if (debug_) {
        laser_publisher_ = nodeHandle.advertise<sensor_msgs::LaserScan>(
                        "distance_estimator/scan", 0);
        cloud_publisher_ = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>(
                        "/distance_estimator/points", 0);
    }

    // other
    shrinkage = 0.5;  // parameter tuning shrinkage of box before distance estimation
}

float DistanceEstimator::Estimate(
               const object_detector::Detection& det,
               const sensor_msgs::LaserScan::ConstPtr& scan, 
               const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
               int w_org, int h_org)
{
    float min_distance;
    // POINTCLOUD
    int x0_shr = (int)( det.x0 + (1.0 - shrinkage) / 2.0 * det.w );
    int y0_shr = (int)( det.y0 + (1.0 - shrinkage) / 2.0 * det.h );
    int w_shr = (int)det.w * shrinkage;
    int h_shr = (int)det.h * shrinkage;

    min_distance = 1000000.;
    for (int i = y0_shr; i < y0_shr + h_shr; ++i) {
        for (int j = x0_shr; j < x0_shr + w_shr; ++j) {
            int idx = i * cloud->width + j;
            float x = cloud->points[idx].x;
            float y = cloud->points[idx].y;
            float z = cloud->points[idx].z;

            float distance =  sqrt(x*x + y*y + z*z);
            if (distance < min_distance) {
                min_distance = distance;
            }
        }
    }
    
    if (debug_) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
        msg->header.frame_id = "some_frame";
        msg->height = msg->width = 1;
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

    // LIDAR
    if (std::isinf(min_distance) || debug_) 
    {
        float angle_min = scan->angle_min;
        float increment = scan->angle_increment;
  

        // TODO change the the way you compute start and angle of lidar analysis
        // get them from cloud
        
        float camera_FOV = M_PI;
        float start_angle = (float)det.x0 / (float)w_org * camera_FOV - camera_FOV / 2;
        float end_angle = (float)(det.x0 + det.w) / (float)w_org * camera_FOV - camera_FOV / 2;

        // map angles to laser frame
        start_angle = start_angle > 0 ? -(start_angle - M_PI) : -(start_angle + M_PI);
        end_angle = end_angle > 0 ? -(end_angle - M_PI) : -(end_angle + M_PI);
        

        int end_idx = (int)( (start_angle - angle_min) / (float)increment );
        int start_idx = (int)( (end_angle - angle_min) / (float)increment );

        min_distance = 10000.;
        for (int i = start_idx; i < end_idx + 1; i++) {
            float r = scan->ranges[i];  
            if (r < scan->range_max && r > scan->range_min && r < min_distance) {
                min_distance = r;
            }
        }
        std::cout << "min_distance LIDAR: " << min_distance << std::endl;
        
        if (debug_) {
            sensor_msgs::LaserScan::Ptr msg (new sensor_msgs::LaserScan);
            msg->header.frame_id = "some_frame";
            msg->angle_min = scan->angle_min;
            msg->angle_max = scan->angle_max;
            msg->angle_increment = scan->angle_increment;
            msg->range_min = scan->range_min;
            msg->range_max = scan->range_max;
            for (int i = start_idx; i < end_idx + 1; i++) {
                msg->ranges.push_back(scan->ranges[i]);
            }
            laser_publisher_.publish(msg);
        }
    }
    return min_distance;
}

void DistanceEstimator::callback(
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
        auto d = new Detection;
        d->x0 = boxes_i[i].x0;
        d->y0 = boxes_i[i].y0;
        d->h = boxes_i[i].h;
        d->w = boxes_i[i].w;
        d->x0 = (int)( boxes_i[i].x0 + (1.0 - shrinkage) / 2.0 * boxes_i[i].w );
        d->y0 = (int)( boxes_i[i].y0 + (1.0 - shrinkage) / 2.0 * boxes_i[i].h );
        d->w = (int)boxes_i[i].w * shrinkage;
        d->h = (int)boxes_i[i].h * shrinkage;
        d->certainty = boxes_i[i].certainty;
        d->class_id = boxes_i[i].class_id;
        d->distance = Estimate(boxes_i[i], scan, cloud,
                bundle_i->frame_width, bundle_i->frame_height);
        boxes_o.push_back(*d);
    }
    bundle_o.detections = boxes_o;
    publisher_.publish(bundle_o);
}
