#include "distance_estimator/DistanceEstimator.hpp"

//using namespace sensor_msgs;
using namespace object_detector;
using namespace message_filters;

DistanceEstimator* g_ptr;

void onMouse(int event, int x, int y, int, void*)
{
    g_ptr->callbackChuj(event, x, y);
}

DistanceEstimator::DistanceEstimator(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle)
{
    image_sub_.subscribe(nodeHandle, "/camera/depth/image_raw", 1);
    laser_sub_.subscribe(nodeHandle, "/scan", 1);
    detect_sub_.subscribe(nodeHandle, "/detector/detection_bundle", 1);
    cloud_sub_.subscribe(nodeHandle, "/camera/depth/points", 1);
    sync_.reset(new Sync(MySyncPolicy(10), image_sub_,  laser_sub_, detect_sub_, cloud_sub_));
    sync_->registerCallback(boost::bind(&DistanceEstimator::callback, this, _1, _2, _3, _4));

    publisher_ = nodeHandle.advertise<DetectionBundle>(
            "/distance_estimator/detection_bundle", 10);
    shrinkage = 0.5;  // parameter tuning shrinkage of box before distance estimation
    camera_FOV = M_PI / 3;
    vis_publisher_ = nodeHandle.advertise<visualization_msgs::Marker>(
                    "visualization_marker", 0 );
    vis_publisher_2 = nodeHandle.advertise<visualization_msgs::Marker>(
                    "visualization_marker2", 0 );
    cv::namedWindow("chuj");
    cv::setMouseCallback("chuj", onMouse, 0);
    cv::Mat chuj2;
    chujFlag = true;
    chujText = "chuj";
}

void DistanceEstimator::callbackChuj(int event, int x, int y){
    //std::cout << "x: " << x << " y: " << y << " value: " << chuj2.at<float>(y,x) << std::endl;
    //std::cout << "rows: " << chuj2.rows << " cols: " << chuj2.cols << std::endl;
    chujText = std::to_string(chuj2.at<float>(y,x));
}

float DistanceEstimator::Estimate(const object_detector::Detection& det,
               cv_bridge::CvImagePtr& cv_ptr,
               const sensor_msgs::LaserScan::ConstPtr& scan, 
               const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
               int w_org, int h_org)
{

    double min;
    float scale_x, scale_y;
    scale_x = static_cast< float >( cv_ptr->image.rows ) / h_org;
    scale_y = static_cast< float >( cv_ptr->image.cols ) / w_org;
    
    g_ptr = this;

    // RGBD
    int x0_shr, y0_shr, w_shr, h_shr;

    x0_shr = (int)( det.x0 + (1.0 - shrinkage) / 2.0 * det.w );
    y0_shr = (int)( det.y0 + (1.0 - shrinkage) / 2.0 * det.h );
    w_shr = (int)det.w * shrinkage;
    h_shr = (int)det.h * shrinkage;
    cv::Rect roi(x0_shr, y0_shr, w_shr, h_shr);
    cv::Mat image_roi = cv_ptr->image(roi);
    cv::minMaxLoc(image_roi, &min, NULL);

    chuj2 = cv_ptr->image;
    cv::putText(chuj2, chujText, cv::Point(550, 450), 1, 1.0, cv::Scalar(0.));
    cv::imshow("chuj", chuj2);
    cv::waitKey(3);

    // POiNTCLOUD
    //std::cout << "cloud width: " << cloud->width << " height: " << cloud->height << std::endl;
    //std::cout << "size: " << cloud->points.size() << std::endl;

    float x, y, z, min_distance_cloud, distance;
    int idx;
    min_distance_cloud = 1000000.0;
    for (int i = y0_shr; i < y0_shr + h_shr; ++i) {
        for (int j = x0_shr; j < x0_shr + w_shr; ++j) {
            idx = i * cloud->width + j;
            x = cloud->points[idx].x;
            y = cloud->points[idx].y;
            z = cloud->points[idx].z;

            distance =  sqrt(x*x + y*y + z*z);
            if (distance < min_distance_cloud) {
                min_distance_cloud = distance;
            }
        }
    }
    std::cout << "min_distance cloud: " << min_distance_cloud << std::endl;

    // LIDAR
    //if (std::isinf(min)) 
    
    if (1) 
    {
        float increment, angle_min, range_max, start_angle, end_angle;
        int start_idx, end_idx;
        angle_min = scan->angle_min;
        range_max = scan->range_max;
        increment = scan->angle_increment;

        start_angle = (float)det.x0 / (float)w_org * camera_FOV - camera_FOV / 2;
        end_angle = (float)(det.x0 + det.w) / (float)w_org * camera_FOV - camera_FOV / 2;

        // map angles to laser frame
        start_angle = start_angle > 0 ? -(start_angle - M_PI) : -(start_angle + M_PI);
        end_angle = end_angle > 0 ? -(end_angle - M_PI) : -(end_angle + M_PI);

        end_idx = static_cast< int >( (start_angle - angle_min) / (float)increment );
        start_idx = static_cast< int >( (end_angle - angle_min) / (float)increment );
        //std::cout << "start: " << start_angle << std::endl;
        //std::cout << "end: " << end_angle << std::endl;
        //std::cout << "w: " << det.w << std::endl;
        //std::cout << "start_idx: " << start_idx << " end_idx: " << end_idx << std::endl;
        //std::cout << "start_idx: " << scan->ranges[start_idx] << " end_idx: " << scan->ranges[end_idx] << std::endl;
        std::cout << "min_distance  RGBD: " << min << std::endl;
        float min_distance = 10000;
        float r;
        for (int i = start_idx; i < end_idx + 1; i++) {
            r = scan->ranges[i];  
            if (r < scan->range_max && r > scan->range_min && r < min_distance) {
                min_distance = r;
            }
        }
        std::cout << "min_distance LIDAR: " << min_distance << std::endl;



        visualization_msgs::Marker marker;
        marker.header.frame_id = "laser";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = .1;
        marker.scale.y = .1;
        marker.scale.z = .1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.pose.position.x = cos(start_angle) * scan->ranges[start_idx];
        marker.pose.position.y = sin(start_angle) * scan->ranges[start_idx];
        marker.pose.position.z = 0;
        vis_publisher_2.publish(marker);
    
        visualization_msgs::Marker marker2;
        marker2.header.frame_id = "laser";
        marker2.header.stamp = ros::Time();
        marker2.ns = "my_namespace";
        marker2.id = 0;
        marker2.type = visualization_msgs::Marker::SPHERE;
        marker2.action = visualization_msgs::Marker::ADD;
        marker2.pose.orientation.x = 0.0;
        marker2.pose.orientation.y = 0.0;
        marker2.pose.orientation.z = 0.0;
        marker2.pose.orientation.w = 1.0;
        marker2.scale.x = .1;
        marker2.scale.y = .1;
        marker2.scale.z = .1;
        marker2.color.a = 1.0; // Don't forget to set the alpha!
        marker2.color.r = 1.0;
        marker2.color.g = 0.0;
        marker2.color.b = 0.0;
        marker2.pose.position.x = cos(end_angle) * scan->ranges[end_idx];
        marker2.pose.position.y = sin(end_angle) * scan->ranges[end_idx];
        vis_publisher_.publish(marker2);

    }

    //std::cout << "class_id: " << det.class_id << " distance: " << min << std::endl;

    return min;
}

void DistanceEstimator::callback(const sensor_msgs::Image::ConstPtr& image,
                                 const sensor_msgs::LaserScan::ConstPtr& scan,
                                 const DetectionBundle::ConstPtr& bundle_i,
                                 const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    // convert Image msg to cv Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1  );
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

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
        d->distance = Estimate(boxes_i[i], cv_ptr, scan, cloud,
                bundle_i->frame_width, bundle_i->frame_height);
        boxes_o.push_back(*d);
    }
    bundle_o.detections = boxes_o;
    publisher_.publish(bundle_o);
}
