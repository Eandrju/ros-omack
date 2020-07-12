#pragma once

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <position_estimator/LocalizedDetection.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <math.h>

using namespace position_estimator;

class SemanticMapper{
    public:
        SemanticMapper(ros::NodeHandle& nodeHandle, bool debug);

        void callback(const LocalizedDetection::ConstPtr& detection);

    private:
        std::vector<LocalizedDetection*> detections; 
        ros::Subscriber subscriber;
        ros::Publisher publisher;
        ros::NodeHandle nodeHandle;
};

