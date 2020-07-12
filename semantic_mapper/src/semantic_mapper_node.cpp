#include "ros/ros.h"
#include "semantic_mapper/SemanticMapper.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "semantic_mapper");
    ros::NodeHandle nodeHandle("~");
    
    SemanticMapper semantic_mapper(nodeHandle, true);

    ros::spin();
    return 0;
}

