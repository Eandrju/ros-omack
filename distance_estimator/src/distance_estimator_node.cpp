#include "ros/ros.h"
#include "distance_estimator/DistanceEstimator.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "distance_estimator");
    ros::NodeHandle nodeHandle("~");
    
    DistanceEstimator distanceEstimator(nodeHandle, true);

    ros::spin();
    return 0;
}

