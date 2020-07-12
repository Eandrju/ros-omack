#include "semantic_mapper/SemanticMapper.hpp"

//using namespace sensor_msgs;
using namespace visualization_msgs;
using namespace position_estimator;
using namespace std;

SemanticMapper::SemanticMapper(ros::NodeHandle& nodeHandle, bool debug=false) :
    nodeHandle(nodeHandle)
{
    // subscribers:
    subscriber = nodeHandle.subscribe(
        "/position_estimator/detection", 10, &SemanticMapper::callback, this);

    // publishers:
    publisher = nodeHandle.advertise<MarkerArray>(
        "/semantic_mapper/visualization_marker_array", 10);

}


void SemanticMapper::callback(const LocalizedDetection::ConstPtr& detection)
{
    //pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
    LocalizedDetection* det = new LocalizedDetection(*detection);

    detections.push_back(det);
    if (detections.size() > 100) {
        delete detections.at(0);
        detections.erase(detections.begin());
    }
    
    MarkerArray marker_array;
    for (auto &d: detections) {
        cout << "Marker: " << d->point.point.x << " " << d->point.point.y << endl;
        //pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
        Marker::Ptr marker (new Marker);
        marker->header.frame_id = "map";
        marker->header.stamp = ros::Time();
        marker->ns = "my_namespace";
        marker->id = 0;
        marker->type = visualization_msgs::Marker::CYLINDER;
        marker->action = visualization_msgs::Marker::ADD;
        marker->pose.position.x = d->point.point.x;
        marker->pose.position.y = d->point.point.y;
        marker->pose.position.z = 0;
        marker->pose.orientation.x = 0.0;
        marker->pose.orientation.y = 0.0;
        marker->pose.orientation.z = 0.0;
        marker->pose.orientation.w = 1.0;
        marker->scale.x = 0.3;
        marker->scale.y = 0.3;
        marker->scale.z = 0.01;
        marker->color.a = 1.0; // Don't forget to set the alpha!
        marker->color.r = rand() / (float)RAND_MAX;
        marker->color.g = rand() / (float)RAND_MAX;
        marker->color.b = rand() / (float)RAND_MAX;
        marker_array.markers.push_back(*marker);
    }
        publisher.publish( marker_array );
}

