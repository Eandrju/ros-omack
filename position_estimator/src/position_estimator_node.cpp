#include "ros/ros.h"
#include "position_estimator/PositionEstimator.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "position_estimator");
    ros::NodeHandle nodeHandle("~");
    
    PositionEstimator position_estimator(nodeHandle, true);

    ros::spin();
    return 0;
}

