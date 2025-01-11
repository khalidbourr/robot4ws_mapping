#ifndef GRIDMAP_NODE_H
#define GRIDMAP_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>

class multi_layer_map {
public:
    multi_layer_map();
    virtual ~multi_layer_map() = default;

protected:
    ros::NodeHandle nh_;
    grid_map::GridMap globalMap_;
    double cell_size;
    double local_map_size;
    bool pose_received;
    geometry_msgs::TransformStamped odom_transform_msg;
    
    void updateGlobalMap(grid_map::GridMap& localMap, const std::string& layer_name, 
                        geometry_msgs::TransformStamped transform);
    void publish_gridmap();
    void load_params();
    void load_robot_static_tf();
    virtual void setGridMapTopic(const std::string& topic_name) {
        grid_map_topic_name = topic_name;
        grid_map_pub = nh_.advertise<grid_map_msgs::GridMap>(grid_map_topic_name, 1, true);
    }

private:
    // ... rest of your existing private members ...
    ros::Subscriber cloud_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber costmap_sub;
    ros::Publisher grid_map_pub;
    
    std::string grid_map_topic_name;
    std::string lidar_topic_name;
    std::string odom_topic_name;
    std::string costmap2d_topic_name;
    
    grid_map::Position last_center_position_global_frame;
    std::string grid_map_frame_id;
    
    double global_map_size;
    
    std::string elevation_layer_name;
    std::string elevation_variance_layer_name;
    std::string obstacles_layer_name;
    
    double pc_height_threshold;
    
    double lidar_variance;
    std::string lidar_frame_id;
    std::string foot_print_frame_id;
    tf2::Transform foot2lidar_transform;

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void costmap_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
};

#endif // GRIDMAP_NODE_H