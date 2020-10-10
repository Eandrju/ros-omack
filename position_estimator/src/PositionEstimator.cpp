#include "position_estimator/PositionEstimator.hpp"

//using namespace sensor_msgs;
using namespace object_detector;
using namespace message_filters;
using namespace position_estimator;
using namespace geometry_msgs;
using namespace std;

PositionEstimator::PositionEstimator(
    ros::NodeHandle& node_handle,
    bool debug=false
):
    node_handle(node_handle), debug(debug), tfListener(tf_buffer)
{
    // subscribers:
    std::string cloud_endpoint, detector_endpoint;
    node_handle.param<std::string>("/cluster_extractor/cloud_input", cloud_endpoint, "/generated_on_pc/points");
    cloud_sub_.subscribe(node_handle, cloud_endpoint, 1);
    node_handle.param<std::string>("/detector_endpoint", detector_endpoint, "/detector/detection_bundle");
    detect_sub_.subscribe(node_handle, detector_endpoint, 1);
    cout<<cloud_endpoint<<" "<<detector_endpoint<<endl;

    subb = node_handle.subscribe(cloud_endpoint, 1, &PositionEstimator::callback2, this);

    // msg synchronization:
    sync_.reset(new Sync(MySyncPolicy(10), detect_sub_, cloud_sub_));
    sync_->registerCallback(boost::bind(&PositionEstimator::callback, this, _1, _2));

    // publishers:
    publisher_ = node_handle.advertise<LabeledCluster>(
            "/cluster_extractor/labeled_cluster", 10);


    vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    if (debug) {
        cloud_publisher_ = node_handle.advertise<pcl::PointCloud<PointType>>(
                        "/position_estimator/points", 10);
        rgb_cloud_publisher = node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
                        "/position_estimator/points2", 10);
        rgb_cloud_publisher2 = node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
                        "/position_estimator/points3", 10);
        rgb_cloud_publisher3 = node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
                        "/position_estimator/points_roi", 10);
        rgb_cloud_publisher4 = node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
                        "/position_estimator/points_no_ground", 10);
        rgb_cloud_publisher5 = node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
                        "/position_estimator/points_no_walls", 10);
        rgb_cloud_publisher6 = node_handle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
                        "/position_estimator/points_downsampled", 10);
 
    }

    ros::Duration(1).sleep();

}

void PositionEstimator::filter_cloud(
    const object_detector::Detection& det,
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    pcl::PointCloud<PointType>::Ptr& extracted_cluster
    )
{
    bool ground_filter_flag, wall_filter_flag, downsampling_flag, mapping_flag;
    node_handle.param<bool>("/debug/ground_filter", ground_filter_flag, true);
    node_handle.param<bool>("/debug/wall_filter", wall_filter_flag, true);
    node_handle.param<bool>("/debug/downsampling_filter", downsampling_flag, false);

    boost::shared_ptr<vector<int> > input_indices (new vector<int>);
    boost::shared_ptr<vector<int> > output_indices (new vector<int>);

    int cloud_min_size;
    node_handle.param<int>("/cluster_extractor/cloud_min_size", cloud_min_size, 10);


    // extract region surrounding detected object
    extract_region_of_interest (cloud, det, output_indices);
    visualize_sub_cloud(cloud, output_indices, std::make_tuple(-1, -1, -1), 2);
    *input_indices = *output_indices;
    output_indices->clear();
    remove_nans(cloud, input_indices, output_indices);
    if(output_indices->size() < cloud_min_size) return;
    cout << "roi without nans size: " << output_indices->size() << endl;
    if (downsampling_flag) {
        *input_indices = *output_indices;
        downsample(cloud, input_indices, output_indices);
    }

    cout << "roi cloud size: " << output_indices->size () << endl;
    // discard points being part of ground or ceiling
    if (ground_filter_flag) {
        *input_indices = *output_indices;
        output_indices->clear();
        if (not remove_ground (cloud, input_indices, output_indices)) return;
        cout << "ground excluded cloud size: " << output_indices->size () << endl;
        if(output_indices->size() < cloud_min_size) return;
        visualize_sub_cloud(cloud, output_indices, std::make_tuple(-1, -1, -1), 3);
    }

    // discard walls
    if (wall_filter_flag) {
        *input_indices = *output_indices;
        output_indices->clear();
        remove_possible_walls(cloud, input_indices, output_indices);
        cout << "walls excluded cloud size: " << output_indices->size () << endl;
        if(output_indices->size() < cloud_min_size) return;
//        visualize_sub_cloud(cloud, output_indices, std::make_tuple(-1, -1, -1), 4);
    }

    // using euclidean clusterization group points
    std::vector<pcl::PointIndices> clusters_indices;
    get_clusters (cloud, output_indices, &clusters_indices);
    if (clusters_indices.size () == 0) return;

    // visualize purpose
    visualize_clusters(cloud, clusters_indices);

    cout << "number of clusters: " << clusters_indices.size () << endl;
    // choose cluster most likely to be part of detected object
    int cluster_idx = choose_cluster (cloud, &clusters_indices, det);
    if (cluster_idx == -1) return;

    cout << "choosen object cloud size: " << clusters_indices[cluster_idx].indices.size () << endl;
    // extract choosen points and publish them
    pcl::ExtractIndices<PointType> extractor;
    extractor.setInputCloud(cloud);
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = clusters_indices[cluster_idx];
    extractor.setIndices(indices);
    extractor.filter(*extracted_cluster);
    int r, g, b;
    node_handle.param<int>("/r", r, -1);
    node_handle.param<int>("/g", g, -1);
    node_handle.param<int>("/b", b, -1);

    cout << "publishing cloud" << endl;
    cloud_publisher_.publish(extracted_cluster);


    node_handle.param<bool>("/debug/mapping", mapping_flag, true);
    if (mapping_flag) {
        pcl::PointCloud<PointType>::Ptr mapped_cluster (new pcl::PointCloud<PointType>);
        extracted_cluster->header = cloud->header;
        map_cloud_to_ground(extracted_cluster, mapped_cluster);
        *extracted_cluster = *mapped_cluster;
        cout << "mapping cloud to ground" << endl;
    }
    
    cloud_publisher_.publish(extracted_cluster);
}

void PositionEstimator::map_cloud_to_ground(
        const pcl::PointCloud<PointType>::ConstPtr& extracted_cluster,
        pcl::PointCloud<PointType>::Ptr& mapped_cluster 
    )
{
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud(extracted_cluster);
    float leaf_size;
    node_handle.param<float>("/detector/leaf_size", leaf_size, 0.01);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<PointType>::Ptr  downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_grid.filter(*downsampled);
  
    //ros::Time t = ros::Time(0);
    //pcl_ros::transformPointCloud("/map", t, *downsampled, "/camera_rgb_optical_frame", *mapped_cluster, tf_listener_v1);
    pcl_ros::transformPointCloud("/map", *downsampled, *mapped_cluster, tf_listener_v1);
    for (int j=0; j < mapped_cluster->points.size(); ++j) {
            mapped_cluster->points[j].z = 0.01;
        }
}


// TODO this won't work with PointType XYZ - needs an update
void PositionEstimator::visualize_clusters(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    std::vector<pcl::PointIndices> clusters_indices
    )
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    output_cloud->header = cloud->header;

    pcl::ExtractIndices<pcl::PointXYZRGB> extractor;
    extractor.setInputCloud(cloud);
    for (int i = 0; i < clusters_indices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        *indices = clusters_indices[i];
        extractor.setIndices(indices);
        extractor.filter(*extracted_cluster);
        int r = rand() % 255;
        int g = rand() % 255;
        int b = rand() % 255;
        for (int j=0; j < extracted_cluster->points.size(); ++j) {
            extracted_cluster->points[j].r = r;
            extracted_cluster->points[j].g = g;
            extracted_cluster->points[j].b = b;
        }
        *output_cloud += *extracted_cluster;
    }

    rgb_cloud_publisher2.publish(output_cloud);
}

void PositionEstimator::visualize_sub_cloud(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    pcl::PointIndices indices,
    std::tuple<int,int,int> color,
    int publisher
)
{
    auto indices_ = boost::make_shared<vector<int> >(indices.indices);
    visualize_sub_cloud(cloud, indices_, color, publisher);
}


void PositionEstimator::visualize_sub_cloud(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    boost::shared_ptr<vector<int> >& indices,
    std::tuple<int,int,int> color,
    int publisher
)
{
    pcl::PointCloud<PointType>::Ptr extracted_sub_cloud(new pcl::PointCloud<PointType>);
    pcl::ExtractIndices<PointType> extractor;
    extractor.setInputCloud(cloud);
    extractor.setIndices(indices);
    extractor.filter(*extracted_sub_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    output_cloud->header = cloud->header;
    for (int i=0; i < extracted_sub_cloud->points.size(); ++i) {
        pcl::PointXYZRGB point;
        point.x = extracted_sub_cloud->points[i].x;
        point.y = extracted_sub_cloud->points[i].y;
        point.z = extracted_sub_cloud->points[i].z;
        if (std::get<0>(color) == -1 || std::get<1>(color) == -1 || std::get<2>(color) == -1) {
            point.r = extracted_sub_cloud->points[i].r; 
            point.g = extracted_sub_cloud->points[i].g;
            point.b = extracted_sub_cloud->points[i].b;
        } else {
            point.r = std::get<0>(color);
            point.g = std::get<1>(color);
            point.b = std::get<2>(color);
        }

        output_cloud->points.push_back(point);
    }
    switch(publisher) {
        case 1:
            rgb_cloud_publisher.publish(output_cloud);
            break;
        case 2:
            rgb_cloud_publisher3.publish(output_cloud);
            break;
        case 3:
            rgb_cloud_publisher4.publish(output_cloud);
            break;
        case 4:
            rgb_cloud_publisher5.publish(output_cloud);
            break;
        case 5:
            rgb_cloud_publisher6.publish(output_cloud);
            break;
    }
}

float PositionEstimator::compute_IOU_metric(
        BoundingBox box1,
        BoundingBox box2
    )
{
    int overlap_x0 , overlap_y0, overlap_x1, overlap_y1;
    int union_x0, union_y0, union_x1, union_y1;
    cout << "cluster rect x0: " << box1.x0 << " y0: " << box1.y0 << " w: " << box1.x1-box1.x0 << " h: " << box1.y1-box1.y0 << endl;
    cout << "dtecion rect x0: " << box2.x0 << " y0: " << box2.y0 << " w: " << box2.x1-box2.x0 << " h: " << box2.y1-box2.y0 << endl;

    overlap_x0 = std::max(box1.x0, box2.x0);
    overlap_x1 = std::min(box1.x1, box2.x1);
    overlap_y0 = std::max(box1.y0, box2.y0);
    overlap_y1 = std::min(box1.y1, box2.y1);

    union_x0 = std::min(box1.x0, box2.x0);
    union_x1 = std::max(box1.x1, box2.x1);
    union_y0 = std::min(box1.y0, box2.y0);
    union_y1 = std::max(box1.y1, box2.y1);

    float union_area = (union_x1 - union_x0) * (union_y1 - union_y0);
    float overlap_area = (overlap_x1 - overlap_x0) * (overlap_y1 - overlap_y0);
    float roi = overlap_area / union_area;
    return roi;

}

int PositionEstimator::choose_cluster (
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    std::vector<pcl::PointIndices>* clusters,
    const object_detector::Detection& det
    )

{
    int width = cloud->width;
    int height = cloud->height;
    int best_cluster_idx;
    int y_min, y_max, x_min, x_max, x_idx, y_idx;
    float best_iou = 0.;
    float iou;

    for (int i = 0; i < clusters->size(); ++i) {
        /* pcl::PointIndices obj_indices = (*clusters)[i]; */
        y_min= height;
        y_max= 0;
        x_min= width;
        x_max= 0;

        for (int j = 0; j < (*clusters)[i].indices.size(); ++j) {
            y_idx = (*clusters)[i].indices[j] / width;
            x_idx = (*clusters)[i].indices[j] % width;

            y_min = (y_idx < y_min) ? y_idx: y_min;
            y_max = (y_idx > y_max) ? y_idx: y_max;
            x_min = (x_idx < x_min) ? x_idx: x_min;
            x_max = (x_idx > x_max) ? x_idx: x_max;
        }

        BoundingBox cluster_rect = {x_min, x_max, y_min, y_max};
        BoundingBox detection_rect = {
            static_cast<int>(det.x0),
            static_cast<int>(det.x0 + det.w),
            static_cast<int>(det.y0),
            static_cast<int>(det.y0 + det.h)
        };

        iou = compute_IOU_metric(cluster_rect, detection_rect);

        if (iou > best_iou) {
            best_iou = iou;
            best_cluster_idx = i;
        }
    }
    cout << "Best IOU metric: "<< best_iou << endl;

    float iou_threshold;
    node_handle.param<float>("/iou_threshold", iou_threshold, 0.8);

    if (best_iou >= iou_threshold)
        return best_cluster_idx;
    else
        return -1;
}


void PositionEstimator::remove_possible_walls(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    boost::shared_ptr<vector<int> >& input_indices,
    boost::shared_ptr<vector<int> >& output_indices
    )
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    float ramsac_threshold;
    node_handle.param<float>("/ramsac_threshold", ramsac_threshold, 0.01);
    seg.setDistanceThreshold(ramsac_threshold);

    // Fit a plane
    seg.setInputCloud(cloud);

    boost::shared_ptr<vector<int> > indices_to_be_removed (new vector<int>);
    boost::shared_ptr<vector<int> > indices_to_check(new vector<int>);
    *indices_to_check = *input_indices;

    int wall_iterations;
    node_handle.param<int>("/wall_iterations", wall_iterations, 3);
    
    for (int i = 0; i < wall_iterations; ++i) {
        seg.setIndices(indices_to_check);
        seg.segment(*inliers, *coefficients);

        if (not is_plane_perpendicular_to_the_floor(cloud, coefficients)){
            continue;
        }

        boost::shared_ptr<vector<int> > global_outliers (new vector<int>);
        std::set_difference(
            input_indices->begin(),
            input_indices->end(),
            inliers->indices.begin(),
            inliers->indices.end(),
            std::inserter(*global_outliers, global_outliers->begin())
        );

        if (are_outliers_between_plane_and_robot(cloud, coefficients, global_outliers)){
            visualize_sub_cloud(cloud, *inliers, std::make_tuple(0, 255, 0), 1);
            indices_to_be_removed->insert(
                indices_to_be_removed->end(), 
                inliers->indices.begin(),
                inliers->indices.end()
            );
        }
        else {
            visualize_sub_cloud(cloud, *inliers, std::make_tuple(255, 0, 0), 1);
        }

        // remove found plane from input indices and search again
        boost::shared_ptr<vector<int> > outliers (new vector<int>);
        std::set_difference(
            indices_to_check->begin(),
            indices_to_check->end(),
            inliers->indices.begin(),
            inliers->indices.end(),
            std::inserter(*outliers, outliers->begin())
        );
        /* cout << "input indices size: " << input_indices->size() << endl; */
        /* cout << "indices_to_check size: " << indices_to_check->size() << endl; */
        /* cout << "inliers size" << inliers->indices.size() << endl; */
        /* cout << "outliers size: " << outliers->size() << endl; */
        /* cout << "global outliers size: " << global_outliers->size() << endl; */
        /* cout << "indices to be removed size: " << indices_to_be_removed->size() << endl; */
        /* cout << "-----------------------------------" << endl; */
        indices_to_check->swap(*outliers);
    }

    std::sort (indices_to_be_removed->begin(), indices_to_be_removed->end());
    std::set_difference(
        input_indices->begin(),
        input_indices->end(),
        indices_to_be_removed->begin(),
        indices_to_be_removed->end(),
        std::inserter(*output_indices, output_indices->begin())
    );



    cout << "Output size: " << output_indices->size() <<
        " should be: " << input_indices->size() - indices_to_be_removed->size() << endl;
}


bool PositionEstimator::is_normal_vector_pointing_towards_space_in_between_plane_and_robot(
    pcl::ModelCoefficients::Ptr coefficients
)
{
    if ( coefficients->values[2] > 0)
        return false;
    else
        return true;
}


bool PositionEstimator::is_point_lying_in_space_between_plane_and_robot(
    pcl::ModelCoefficients::Ptr coefficients,
    PointType p
)
{
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];
    if (a * p.x + b * p.y + c * p.z + d > 0)
        return false;
    else
        return true;
}

bool PositionEstimator::are_outliers_between_plane_and_robot(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    pcl::ModelCoefficients::Ptr coefficients,
    boost::shared_ptr<vector<int> >& outliers
)
{
    bool normal_vector_switched = false;
    // chekc whether normal vector is pointing in the direction of robot or not
    if (is_normal_vector_pointing_towards_space_in_between_plane_and_robot(coefficients)) {
        normal_vector_switched = true;
        for (int i = 0; i < 3; ++i)
            coefficients->values[i] *= -1;
    }

    boost::shared_ptr<vector<int> > in_between_indices (new vector<int>);
    boost::shared_ptr<vector<int> > elsewhere (new vector<int>);
    // count points lying in beetwen
    int points_in_between = 0;
    for (int i = 0; i < outliers->size(); ++i) {
        int idx = (*outliers)[i];
        if (is_point_lying_in_space_between_plane_and_robot(coefficients, cloud->points[idx])) {
            points_in_between++;
            in_between_indices->push_back(idx);
        }
        else {
            elsewhere->push_back(idx);
        }
    }
    visualize_sub_cloud(cloud, in_between_indices, std::make_tuple(245, 171, 0), 1);
    visualize_sub_cloud(cloud, elsewhere, std::make_tuple(0, 0, 240), 1);
    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud->header.frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    if (normal_vector_switched) {
        marker.color.r = 0.5;
        marker.color.g = 0.5;
    } else {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
    }
    marker.color.b = 0.0;
    geometry_msgs::Point p0, p1;
    p0.x = 0; p0.y = 0; p0.z = 0;
    marker.points.push_back(p0);
    p1.x = coefficients->values[0];
    p1.y = coefficients->values[1];
    p1.z = coefficients->values[2];
    marker.points.push_back(p1);
    vis_pub.publish( marker );

    cout << "Percentage of points in between: " << static_cast<float>(points_in_between) / outliers->size() << endl;
    // check if plane is in fact wall <==> most of points lies in between
    float wall_threshold;
    node_handle.param<float>("/wall_threshold", wall_threshold, 0.95);
    if (static_cast<float>(points_in_between) / outliers->size() > wall_threshold)
        return true;
    else
        return false;

    }


bool PositionEstimator::is_plane_perpendicular_to_the_floor(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    pcl::ModelCoefficients::Ptr coefficients
){
    float x = coefficients->values[0];
    float y = coefficients->values[1];
    float z = coefficients->values[2];

    geometry_msgs::TransformStamped transform;

    try{
        transform = tf_buffer.lookupTransform ("odom", cloud->header.frame_id, ros::Time(0));
        /* transform = tf_buffer.lookupTransform ("odom", cloud->header.frame_id, cloud->header.stamp); // that won't work wrong format of stamp - maybe need a further investigation */
    }
    catch(tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }

    geometry_msgs::Quaternion q = transform.transform.rotation;
    Eigen::Quaternionf rotation (q.w ,q.x ,q.y ,q.z );
    geometry_msgs::Vector3 v = transform.transform.translation;
    Eigen::Translation<float, 3> translation (v.x, v.y, v.z);
    Eigen::Transform<float, 3, Eigen::Affine> t (translation * rotation);

    Eigen::Matrix<float, 3, 1> pt (x, y, z);

    float z1 = static_cast<float> (t (2, 0) * pt.coeffRef (0) + t (2, 1) * pt.coeffRef (1) + t (2, 2) * pt.coeffRef (2) + t (2, 3));
    float z0 = static_cast<float> (t (2, 3));

    // if z coeff is close to zero in global frame then plane is perpendicular
    cout << "Z coeff of transformed normal vector: "<<abs(z1-z0)<<endl;
    if (abs(z1 - z0) < 0.1)
        return true;
    else
        return false;
}




void PositionEstimator::downsample(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    boost::shared_ptr<vector<int> >& input_indices,
    boost::shared_ptr<vector<int> >& output_indices
    )
{
    pcl::VoxelGrid<PointType> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setIndices(input_indices);
    float leaf_size;
    node_handle.param<float>("/detector/leaf_size", leaf_size, 0.01);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    pcl::PointCloud<PointType>::Ptr  downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_grid.filter(*downsampled);
    cout << "Leaf size: "<<leaf_size<<" cloud size: "<< downsampled->points.size()<<endl;
    rgb_cloud_publisher6.publish(downsampled); 
}


void PositionEstimator::remove_nans(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    boost::shared_ptr<vector<int> >& input_indices,
    boost::shared_ptr<vector<int> >& output_indices
    )
{
    for (size_t i = 0; i < input_indices->size (); ++i) {
        int idx = (*input_indices)[i];
        if (!isfinite (cloud->points[idx].x) ||
            !isfinite (cloud->points[idx].y) ||
            !isfinite (cloud->points[idx].z))
            continue;
        output_indices->push_back(idx);
    }
}

bool PositionEstimator::remove_ground(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    boost::shared_ptr<vector<int> >& roi_indices,
    boost::shared_ptr<vector<int> >& output_indices
    )
{
    geometry_msgs::TransformStamped transform;


    try{
        transform = tf_buffer.lookupTransform ("odom", cloud->header.frame_id, ros::Time(0));
        /* transform = tf_buffer.lookupTransform ("odom", cloud->header.frame_id, cloud->header.stamp); */
    }
    catch(tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }

    float ground_cutoff, ceiling_cutoff;
    if (not node_handle.getParam("/cluster_extractor/ground_cutoff", ground_cutoff)) {
        ground_cutoff = 0.05;
    }
    if (not node_handle.getParam("/cluster_extractor/ceiling_cutoff", ceiling_cutoff)) {
        ceiling_cutoff = 10.;
    }

    geometry_msgs::Quaternion q = transform.transform.rotation;
    Eigen::Quaternionf rotation (q.w ,q.x ,q.y ,q.z );
    geometry_msgs::Vector3 v = transform.transform.translation;
    Eigen::Translation<float, 3> translation (v.x, v.y, v.z);
    Eigen::Transform<float, 3, Eigen::Affine> t (translation * rotation);
    for (size_t i = 0; i < roi_indices->size (); ++i) {
        int idx = (*roi_indices)[i];
        if (!isfinite (cloud->points[idx].x) ||
            !isfinite (cloud->points[idx].y) ||
            !isfinite (cloud->points[idx].z))
            continue;

        Eigen::Matrix<float, 3, 1> pt (cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);

        float z = static_cast<float> (t (2, 0) * pt.coeffRef (0) + t (2, 1) * pt.coeffRef (1) + t (2, 2) * pt.coeffRef (2) + t (2, 3));
        if (z > ground_cutoff && z < ceiling_cutoff) {
            output_indices->push_back(idx);
        }
    }
    return true;
}


void PositionEstimator::extract_region_of_interest(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    const object_detector::Detection& det,
    boost::shared_ptr<vector<int> >& roi_indices
    )
{
    int x0, x1, y0, y1;
    int width = cloud->width;
    int height = cloud->height;
    float shrinkage_ratio;

    if (not node_handle.getParam("/detector/shrinkage_ratio", shrinkage_ratio)) {
        shrinkage_ratio = 1.5;
    }

    x0 = (int)( det.x0 + (1. - shrinkage_ratio) / 2. * det.w );
    x1 = (int)( det.x0 + det.w - (1. - shrinkage_ratio) / 2. * det.w);
    y0 = (int)( det.y0 + (1. - shrinkage_ratio) / 2. * det.h );
    y1 = (int)( det.y0 + det.h - (1. - shrinkage_ratio) / 2. * det.h);

    x0 = (x0 < 0) ? 0: x0;
    x1 = (x1 >= width) ? width-1: x1;
    y0 = (y0 < 0) ? 0: y0;
    y1 = (y1 >= height) ? height-1: y1;


    pcl::ExtractIndices<PointType> extractor;
    extractor.setInputCloud(cloud);
    extractor.setIndices(y0, x0, y1-y0, x1-x0);

    for (int i = y0; i < y1; ++i) {
        for (int j = x0; j < x1; ++j) {
            int idx = i * cloud->width + j;
            roi_indices->push_back(idx);
        }
    }
}


void PositionEstimator::get_clusters(
    const pcl::PointCloud<PointType>::ConstPtr& cloud,
    boost::shared_ptr<vector<int> >& input_indices,
    std::vector<pcl::PointIndices>* object_indices
    )
{
    float tolerance, min_size;
    if (not node_handle.getParam("/detector/cluster/tolerance", tolerance)) {
        tolerance = 0.05;
    }

    if (not node_handle.getParam("/detector/cluster/min_size", min_size)) {
        min_size = 100.;
    }

    pcl::EuclideanClusterExtraction<PointType> euclid;
    euclid.setInputCloud(cloud);
    euclid.setIndices(input_indices);
    euclid.setClusterTolerance(tolerance);
    euclid.setMinClusterSize(min_size);
    euclid.extract(*object_indices);
}



void PositionEstimator::callback(
           const DetectionBundle::ConstPtr& bundle_i,
           const pcl::PointCloud<PointType>::ConstPtr& cloud)
{
    std::vector<object_detector::Detection> boxes_i = bundle_i->detections;
    for (int i = 0; i < bundle_i->size; ++i) {
        cout << "========================" << endl;
        std::string label = boxes_i[i].class_name;
        cout << "Detected object: " << label << endl;

        pcl::PointCloud<PointType>::Ptr  extracted_cluster (new pcl::PointCloud<PointType>);
        filter_cloud(boxes_i[i], cloud, extracted_cluster);
        cout << "Filtered cloud size: " << extracted_cluster->points.size() << endl;
        if(extracted_cluster->points.size() == 0) continue;

        sensor_msgs::PointCloud2 output_cloud;
        pcl::toROSMsg(*extracted_cluster, output_cloud);

        LabeledCluster::Ptr labeled_cluster (new LabeledCluster);
        labeled_cluster->cloud = output_cloud;
        labeled_cluster->label = label; 
        labeled_cluster->certainty = boxes_i[i].certainty; 
        publisher_.publish(labeled_cluster);
    }
    float callback_lag;

    node_handle.param<float>("/detector/callback_lag", callback_lag, 0);
    ros::Duration(callback_lag).sleep();
}




void PositionEstimator::callback2(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  out (new pcl::PointCloud<pcl::PointXYZRGB>);
    out->header = cloud->header;
    for (int i = 0; i < cloud->points.size(); ++i) {
        pcl::PointXYZRGB point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        point.r = cloud->points[i].b;
        point.g = cloud->points[i].g;
        point.b = cloud->points[i].r;
        out->points.push_back(point);
    }
    //rgb_cloud_publisher.publish(out);
}


