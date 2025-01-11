#ifndef COLOR_GRID_MAPPER_H
#define COLOR_GRID_MAPPER_H

#include "../gridmap_node.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class color_grid_mapper : public multi_layer_map {
public:
    color_grid_mapper();
    
protected:
    void colorImageCallback(const sensor_msgs::Image::ConstPtr& img_msg);
    void load_color_params();
    void publish_color_gridmap();  // New method for color publishing
    
    // Color specific members
    std::string color_layer_name;
    std::string color_topic_name;
    ros::Subscriber color_image_sub;
    ros::Publisher color_grid_map_pub;  // Separate publisher for colored grid map
    std::string color_grid_map_topic;  // Topic name for colored grid map
    
    // TF2 members
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif // COLOR_GRID_MAPPER_H