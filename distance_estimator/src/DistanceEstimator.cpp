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
                        "/distance_estimator/scan", 10);
        cloud_publisher_ = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>(
                        "/distance_estimator/points", 1);
    }

    // other
    shrinkage = 0.5; // parameter tuning shrinkage of box before distance estimation
}

float DistanceEstimator::Estimate(
               const object_detector::Detection& det,
               const sensor_msgs::LaserScan::ConstPtr& scan, 
               const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
               int w_org, int h_org)
{
    // POINTCLOUD
    int x0_shr = (int)( det.x0 + (1.0 - shrinkage) / 2.0 * det.w );
    int y0_shr = (int)( det.y0 + (1.0 - shrinkage) / 2.0 * det.h );
    int w_shr = (int)det.w * shrinkage;
    int h_shr = (int)det.h * shrinkage;

    float distance;
    std::vector<float> distances;
    for (int i = y0_shr; i < y0_shr + h_shr; ++i) {
        for (int j = x0_shr; j < x0_shr + w_shr; ++j) {
            int idx = i * cloud->width + j;
            float x = cloud->points[idx].x;
            float y = cloud->points[idx].y;
            float z = cloud->points[idx].z;

            float d =  sqrt(x*x + y*y + z*z);
 
            distances.push_back(d);
        }
    }

    /*
     * In case of even number of elements this
     * implementation returns pseudo-median
     * n/2th element, rather than mean of middle 
     * two elements, but has linear complexity
    */
    if(distances.size() != 0) {
        size_t n = distances.size() / 2;
        std::nth_element(distances.begin(), distances.begin()+n, distances.end());
        std::cout << "vector size " << distances.size() << " n " << n << " distance " << distances[n] << std::endl;
        
        distance = distances[n];
    }
            

    //for (int i = y0_shr; i < y0_shr + h_shr; ++i) {
    //    for (int j = x0_shr; j < x0_shr + w_shr; ++j) {
    //        int idx = i * cloud->width + j;
    //        float x = cloud->points[idx].x;
    //        float y = cloud->points[idx].y;
    //        float z = cloud->points[idx].z;

    //        float distance =  sqrt(x*x + y*y + z*z);
    //        if (distance < min_distance) {
    //            min_distance = distance;
    //        }
    //    }
    //}
    
    if (debug_) {
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

    // LIDAR
    if (std::isnan(distance)) 
    {
        float angle_min = scan->angle_min;
        float increment = scan->angle_increment;
        distance = std::numeric_limits<float>::infinity();

        // TODO change the the way you compute start and angle of lidar analysis
        // get them from cloud
        
        //float camera_FOV = M_PI/3.;
        //float start_angle = (float)det.x0 / (float)w_org * camera_FOV - camera_FOV / 2;
        //float end_angle = (float)(det.x0 + det.w) / (float)w_org * camera_FOV - camera_FOV / 2;
        
        int n = scan->ranges.size();
        float shrinkage_ratio = (float)det.w / (float)w_org * (1. - shrinkage) / 2.;
        float first_ratio = (float)det.x0 / (float)w_org + shrinkage_ratio;
        float second_ratio = (float)(det.x0 + det.w) / (float)w_org - shrinkage_ratio;
        // assuming 360 FOV of lidar and 60 FOV of camera
        // fking stupid 0 idx of lidar is in fornt of robot and 1/4 of idx_max is on his 9 (clock)
        int start_idx = (int)(-n * 1/12 + (1 - second_ratio) * n/6);
        int end_idx = (int)(-n * 1/12 + (1 - first_ratio) * n/6);

        std::vector<int> active_idxs;

        //for (int i = -60; i <  61; i++) {
        //    if( (i+60)%60 == 0)
        //        if(i<0)
        //            active_idxs.push_back(i+n);
        //        else
        //            active_idxs.push_back(i);
        //}

        for (int i = start_idx; i < end_idx + 1; i++) {
            std::cout << "+";
            float r;

            if (i < 0){
                active_idxs.push_back(i+n);
                r = scan->ranges[i+n];  
            } else {
                active_idxs.push_back(i);
                r = scan->ranges[i];  
            }



            if (r < scan->range_max && r > scan->range_min && r < distance) {
                distance = r;
            }
        }
        
        
        debug_ = true;
        if (debug_) {   
            //std::cout << "Angle from " << start_angle << " to " << end_angle << std::endl;
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
            //for (int i = start_idx; i < end_idx + 1; i++) {
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
    }
    return distance;
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
        d->certainty = boxes_i[i].certainty;
        d->class_name = boxes_i[i].class_name;
        d->color = boxes_i[i].color;
        d->distance = Estimate(boxes_i[i], scan, cloud,
                bundle_i->frame_width, bundle_i->frame_height);
        boxes_o.push_back(*d);
    }
    bundle_o.detections = boxes_o;
    publisher_.publish(bundle_o);
}

