#include "ros/ros.h"
#include "position_estimator/PositionEstimator.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "position_estimator");
    ros::NodeHandle nodeHandle("~");
    
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tf_listener(buffer);
    PositionEstimator position_estimator(nodeHandle, true);

    ros::spin();
    return 0;
}

