#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <vision_msgs/Detection3DArray.h>
#include <robot4ws_perception/ColorDetection3DArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/core/core.hpp>
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <vector>
#include <array>

class ColorGridDetector {
public:
    ColorGridDetector() : nh_("~"), tf_listener_(tf_buffer_) {
        loadParameters();
        initializeGridMap();
        setupSubscribers();
    }

    void run() {
        ros::spin();
    }

private:
    struct ColorDefinition {
        float encoded;
        
        ColorDefinition(uint8_t b = 0, uint8_t g = 0, uint8_t r = 0) {
            uint32_t argb = (0xFF << 24) | 
                           ((uint32_t)b << 16) | 
                           ((uint32_t)g << 8) | 
                           (uint32_t)r;
            std::memcpy(&encoded, &argb, sizeof(float));
        }
    };

    struct GridMapConfig {
        double cell_size;
        double map_size;
        std::string frame_id;
        double cell_size_inv;  
        
        GridMapConfig() : 
            cell_size(0.05),
            map_size(6.0),
            frame_id("Archimede_foot_start") {
            updateInverse();
        }
        
        void updateInverse() {
            cell_size_inv = 1.0 / cell_size;
        }
    };

    void loadParameters() {
        config_.cell_size = nh_.param("gridmap/cell_size", config_.cell_size);
        config_.map_size = nh_.param("gridmap/local_map_size", config_.map_size);
        config_.updateInverse();
        initializeColors();
    }

    void initializeColors() {
        static const std::array<std::pair<std::string, std::tuple<uint8_t, uint8_t, uint8_t>>, 7> color_values{{
            {"red",    {255, 0, 0}},
            {"blue",   {0, 0, 255}},
            {"green",  {0, 255, 0}},
            {"yellow", {255, 255, 0}},
            {"purple", {128, 0, 128}},
            {"cyan",   {0, 255, 255}},
            {"orange", {255, 165, 0}}
        }};

        colors_.reserve(color_values.size());
        for (const auto& [name, values] : color_values) {
            const auto& [b, g, r] = values;
            colors_.emplace(name, ColorDefinition(b, g, r));
        }
        
        bg_color_ = ColorDefinition(128, 128, 128);  // Grey background
    }

    void initializeGridMap() {
        grid_map_.setFrameId(config_.frame_id);
        grid_map_.setGeometry(grid_map::Length(config_.map_size, config_.map_size), 
                             config_.cell_size);
        
        static const std::array<std::string, 3> layers{{"elevation", "color", "confidence"}};
        for (const auto& layer : layers) {
            grid_map_.add(layer);
        }
        
        resetGridMap();
        grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("color_grid_map", 1, true);
    }

    void resetGridMap() {
        grid_map::Matrix& elevation = grid_map_["elevation"];
        grid_map::Matrix& color = grid_map_["color"];
        grid_map::Matrix& confidence = grid_map_["confidence"];
        
        elevation.setZero();
        color.setConstant(bg_color_.encoded);
        confidence.setZero();
    }

    void setupSubscribers() {
        detection_sub_ = nh_.subscribe("/color_detection/detections3d", 1, 
            &ColorGridDetector::detectionCallback, this);
    }

    void detectionCallback(const robot4ws_perception::ColorDetection3DArray::ConstPtr& msg) {
        bool updated = false;
        for (const auto& color_detection : msg->detections) {
            updated |= processDetection(color_detection);
        }
        if (updated) {
            publishGridMap();
        }
    }

    bool processDetection(const robot4ws_perception::ColorDetection3D& color_detection) {
        const auto& detection = color_detection.detection;
        const auto& pos = detection.bbox.center.position;
        const float confidence = detection.results[0].score;

        auto color_it = colors_.find(color_detection.color_name);
        if (color_it == colors_.end()) {
            ROS_WARN_STREAM_THROTTLE(1.0, "Unknown color received: " << color_detection.color_name);
            return false;
        }

        // Use the bounding box corners directly
        const grid_map::Position min_pos(pos.x - detection.bbox.size.x/2, 
                                    pos.y - detection.bbox.size.y/2);
        const grid_map::Position max_pos(pos.x + detection.bbox.size.x/2, 
                                    pos.y + detection.bbox.size.y/2);
        
        // Create rectangle from bounding box corners
        std::vector<grid_map::Position> surface_points = {
            grid_map::Position(min_pos.x(), min_pos.y()),
            grid_map::Position(max_pos.x(), min_pos.y()),
            grid_map::Position(max_pos.x(), max_pos.y()),
            grid_map::Position(min_pos.x(), max_pos.y())
        };

        return fillSurfaceArea(min_pos, max_pos, surface_points, 
                            confidence, color_it->second.encoded, pos.z);
    }

    inline void updateGridCell(const grid_map::Index& index, float height, 
                             float color, float confidence) {
        grid_map_.at("elevation", index) = height;
        grid_map_.at("color", index) = color;
        grid_map_.at("confidence", index) = confidence;
    }


    bool fillSurfaceArea(const grid_map::Position& min_pos,
                        const grid_map::Position& max_pos,
                        const std::vector<grid_map::Position>& surface_points,
                        float confidence, float color, float height) {
        if (surface_points.size() < 3) return false;

        bool updated = false;
        grid_map::Position pos;
        grid_map::Index index;
        
        // Calculate bounding box dimensions for confidence falloff
        const double bbox_size = std::max(max_pos.x() - min_pos.x(), 
                                        max_pos.y() - min_pos.y());
        const double max_distance = bbox_size * 0.5;
        const grid_map::Position center((min_pos.x() + max_pos.x()) * 0.5,
                                      (min_pos.y() + max_pos.y()) * 0.5);

        for (pos.x() = min_pos.x(); pos.x() <= max_pos.x(); 
             pos.x() += config_.cell_size) {
            for (pos.y() = min_pos.y(); pos.y() <= max_pos.y(); 
                 pos.y() += config_.cell_size) {
                if (!grid_map_.isInside(pos)) continue;
                if (!isPointInPolygon(pos, surface_points)) continue;

                // Calculate confidence falloff based on distance from center
                const double dist = (pos - center).norm();
                const float cell_conf = confidence * 
                    std::max(0.0f, float(1.0 - (dist / max_distance)));

                grid_map_.getIndex(pos, index);
                if (cell_conf > grid_map_.at("confidence", index)) {
                    updateGridCell(index, height, color, cell_conf);
                    updated = true;
                }
            }
        }
        return updated;
    }

    bool isPointInPolygon(const grid_map::Position& point,
                         const std::vector<grid_map::Position>& polygon) const {
        bool inside = false;
        const size_t size = polygon.size();
        for (size_t i = 0, j = size - 1; i < size; j = i++) {
            if (((polygon[i].y() > point.y()) != (polygon[j].y() > point.y())) &&
                (point.x() < (polygon[j].x() - polygon[i].x()) * 
                (point.y() - polygon[i].y()) / (polygon[j].y() - polygon[i].y()) + 
                polygon[i].x())) {
                inside = !inside;
            }
        }
        return inside;
    }


    void publishGridMap() {
        grid_map_.setTimestamp(ros::Time::now().toNSec());
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(grid_map_, message);
        grid_map_pub_.publish(message);
        ROS_INFO_THROTTLE(1.0, "Published grid map");
    }

    ros::NodeHandle nh_;
    ros::Subscriber detection_sub_;
    ros::Publisher grid_map_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    grid_map::GridMap grid_map_;
    std::unordered_map<std::string, ColorDefinition> colors_;
    ColorDefinition bg_color_;
    GridMapConfig config_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "color_grid_detector");
    ColorGridDetector detector;
    detector.run();
    return 0;
}