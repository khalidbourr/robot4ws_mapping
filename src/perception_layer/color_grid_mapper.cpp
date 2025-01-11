#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <robot4ws_perception/ColorDetection3DArray.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>
#include <vector>
#include <array>

class ColorGridDetector {
public:
    ColorGridDetector() : nh_("~"), tf_listener_(tf_buffer_) {
        initializeSystem();
    }

    void run() {
        ros::spin();
    }

private:
    struct Color {
        float encoded;
        
        Color(uint8_t b = 0, uint8_t g = 0, uint8_t r = 0) {
            uint32_t argb = (0xFF << 24) | 
                           ((uint32_t)b << 16) | 
                           ((uint32_t)g << 8) | 
                           (uint32_t)r;
            std::memcpy(&encoded, &argb, sizeof(float));
        }
    }; 

    struct Config {
        double cell_size;
        double map_size;
        std::string frame_id;
        double height_threshold;
        double area_threshold;
        
        Config() : 
            cell_size(0.05),
            map_size(6.0),
            frame_id("Archimede_foot_start"),
            height_threshold(0.1),
            area_threshold(0.25) {}
    };

    void initializeSystem() {
        loadConfig();
        initializeColors();
        initializeGridMap();
        detection_sub_ = nh_.subscribe("/color_detection/detections3d", 1, 
            &ColorGridDetector::onDetection, this);
    }

    void loadConfig() {
        config_.cell_size = nh_.param("gridmap/cell_size", config_.cell_size);
        config_.map_size = nh_.param("gridmap/local_map_size", config_.map_size);
        config_.height_threshold = nh_.param("gridmap/height_threshold", config_.height_threshold);
        config_.area_threshold = nh_.param("gridmap/area_threshold", config_.area_threshold);
    }

    void initializeColors() {
        const std::array<std::pair<std::string, std::array<uint8_t, 3>>, 7> colors{{
            {"red",    {{255, 0, 0}}},
            {"blue",   {{0, 0, 255}}},
            {"green",  {{0, 255, 0}}},
            {"yellow", {{255, 255, 0}}},
            {"purple", {{128, 0, 128}}},
            {"cyan",   {{0, 255, 255}}},
            {"orange", {{255, 165, 0}}}
        }};

        for (const auto& [name, rgb] : colors) {
            color_map_.emplace(name, Color(rgb[0], rgb[1], rgb[2]));
        }
        background_ = Color(128, 128, 128);
    }

    void initializeGridMap() {
        grid_map_.setFrameId(config_.frame_id);
        grid_map_.setGeometry(grid_map::Length(config_.map_size, config_.map_size), 
                             config_.cell_size);
        
        for (const auto& layer : {"elevation", "color", "confidence"}) {
            grid_map_.add(layer);
        }
        
        resetGridMap();
        grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("color_grid_map", 1, true);
    }

    void resetGridMap() {
        auto& elevation = grid_map_["elevation"];
        auto& color = grid_map_["color"];
        auto& confidence = grid_map_["confidence"];
        
        elevation.setZero();
        color.setConstant(background_.encoded);
        confidence.setZero();
    }

    void onDetection(const robot4ws_perception::ColorDetection3DArray::ConstPtr& msg) {
        bool updated = false;
        for (const auto& detection : msg->detections) {
            updated |= processDetection(detection);
        }
        
        if (updated) {
            publishGridMap();
        }
    }

    bool processDetection(const robot4ws_perception::ColorDetection3D& detection) {
        const auto& bbox = detection.detection.bbox;
        const float z = bbox.center.position.z;
        const float area = bbox.size.x * bbox.size.y;
        
        auto color_it = color_map_.find(detection.color_name);
        if (color_it == color_map_.end()) {
            ROS_WARN_STREAM_THROTTLE(1.0, "Unknown color: " << detection.color_name);
            return false;
        }

        // Use bounding box for elevated or small objects
        if (z > config_.height_threshold || area < config_.area_threshold) {
            return processBBox(detection, color_it->second.encoded);
        }
        
        return processContour(detection, color_it->second.encoded);
    }

    bool processBBox(const robot4ws_perception::ColorDetection3D& detection, float color) {
        const auto& bbox = detection.detection.bbox;
        const auto& pos = bbox.center.position;
        const float conf = detection.detection.results[0].score;

        grid_map::Position min_pos(pos.x - bbox.size.x/2, pos.y - bbox.size.y/2);
        grid_map::Position max_pos(pos.x + bbox.size.x/2, pos.y + bbox.size.y/2);
        
        std::vector<grid_map::Position> corners = {
            {min_pos.x(), min_pos.y()},
            {max_pos.x(), min_pos.y()},
            {max_pos.x(), max_pos.y()},
            {min_pos.x(), max_pos.y()}
        };

        return updateGridArea(min_pos, max_pos, corners, conf, color, pos.z);
    }

    bool processContour(const robot4ws_perception::ColorDetection3D& detection, float color) {
        if (detection.contour_points.empty()) return false;

        const auto& pos = detection.detection.bbox.center.position;
        const float conf = detection.detection.results[0].score;
        
        std::vector<grid_map::Position> points;
        grid_map::Position min_pos(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
        grid_map::Position max_pos(-min_pos.x(), -min_pos.y());

        points.reserve(detection.contour_points.size());
        for (const auto& p : detection.contour_points) {
            grid_map::Position point(p.x, p.y);
            points.push_back(point);
            min_pos = min_pos.cwiseMin(point);
            max_pos = max_pos.cwiseMax(point);
        }

        return updateGridArea(min_pos, max_pos, points, conf, color, pos.z);
    }

    bool updateGridArea(const grid_map::Position& min_pos,
                       const grid_map::Position& max_pos,
                       const std::vector<grid_map::Position>& points,
                       float confidence, float color, float height) {
        if (points.size() < 3) return false;

        const grid_map::Position center = (min_pos + max_pos) * 0.5;
        const double falloff_radius = std::max(max_pos.x() - min_pos.x(), 
                                             max_pos.y() - min_pos.y()) * 0.5;
        
        bool updated = false;
        grid_map::Position pos;
        grid_map::Index idx;

        for (pos.x() = min_pos.x(); pos.x() <= max_pos.x(); pos.x() += config_.cell_size) {
            for (pos.y() = min_pos.y(); pos.y() <= max_pos.y(); pos.y() += config_.cell_size) {
                if (!grid_map_.isInside(pos) || !isInPolygon(pos, points)) continue;

                const double dist = (pos - center).norm();
                const float cell_conf = confidence * std::max(0.0f, float(1.0 - (dist / falloff_radius)));

                grid_map_.getIndex(pos, idx);
                if (cell_conf > grid_map_.at("confidence", idx)) {
                    grid_map_.at("elevation", idx) = height;
                    grid_map_.at("color", idx) = color;
                    grid_map_.at("confidence", idx) = cell_conf;
                    updated = true;
                }
            }
        }
        return updated;
    }

    bool isInPolygon(const grid_map::Position& point,
                     const std::vector<grid_map::Position>& polygon) const {
        bool inside = false;
        for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
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
        grid_map_msgs::GridMap msg;
        grid_map::GridMapRosConverter::toMessage(grid_map_, msg);
        grid_map_pub_.publish(msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber detection_sub_;
    ros::Publisher grid_map_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    grid_map::GridMap grid_map_;
    std::unordered_map<std::string, Color> color_map_;
    Color background_;
    Config config_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "color_grid_detector");
    ColorGridDetector detector;
    detector.run();
    return 0;
}