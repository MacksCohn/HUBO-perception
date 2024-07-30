#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/passthrough.h"
#include "std_msgs/msg/string.hpp"
#include <map>
class Pipe : public rclcpp::Node {
private:
    rclcpp::CallbackGroup::SharedPtr _group_cloud;
    rclcpp::CallbackGroup::SharedPtr _group_info;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber_cloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_cloud;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber_info;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher_info;

    std::map<std::pair<int, int>, std::string> objects_types;
    sensor_msgs::msg::PointCloud2 _cloud;

    void _on_subscriber(sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
        // convert to pcl cloud
        pcl::PCLPointCloud2::Ptr cloud_pcl(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(*cloud, *cloud_pcl);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conversion(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(*cloud_pcl, *cloud_conversion);
        // Box Crop
        // RCLCPP_INFO(get_logger(), "CONVERTED");
        pcl::PassThrough<pcl::PointXYZ> crop(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>());
        crop.setInputCloud(cloud_conversion);
        crop.setFilterFieldName("x");
        crop.setFilterLimits(-5.0,5.0);
        // RCLCPP_INFO(get_logger(), "ABOUT TO CROP");
        crop.filter(*cloud_cropped);
        crop.setInputCloud(cloud_cropped);
        crop.setFilterFieldName("z");
        crop.setFilterLimits(0.0, 3.0);
        crop.filter(*cloud_cropped);
        // RCLCPP_INFO(get_logger(), "CROPPED");
        // Do downsampling
        pcl::VoxelGrid<pcl::PointXYZ> downsampling;
        downsampling.setInputCloud(cloud_cropped);
        downsampling.setLeafSize(0.012, 0.012, 0.012);
        downsampling.filter(*cloud_conversion);
        // Publish downsampled cloud to topic
        cloud_conversion->width = cloud_conversion->size();
        cloud_conversion->height = 1;
        cloud_conversion->is_dense = true;
        cloud_conversion->header.frame_id = cloud->header.frame_id;
        sensor_msgs::msg::PointCloud2 msg;
        // RCLCPP_INFO(get_logger(), "READY_TO_SEND");
        pcl::toROSMsg(*cloud_conversion, msg);
        _publisher_cloud->publish(msg);
        _cloud = msg;
    }

    std::map<std::pair<int, int>, std::string> parse_info(std_msgs::msg::String msg) {
        std::string data = msg.data;
        if (data == "{}")
            return objects_types;
        std::map<std::pair<int, int>, std::string> type_locations;
        // remove curly braces
        data = data.substr(1,data.length()-2);
        size_t num_objects = std::count(data.begin(), data.end(), ',') / 2 + 1;
        for (size_t i = 0; i < num_objects; i++) {
            std::pair<int, int> coords;
            int second_comma = data.find(',', data.find(',')+1);
            std::string object = data.substr(0, second_comma);
            data = data.substr(second_comma+1);
            // get ints between parens
            coords.first = std::stoi(object.substr(object.find('(')+1, object.find(',')));
            coords.second = std::stoi(object.substr(object.find(',')+1, object.find(')')));
            // get string in between ''
            object = object.substr(object.find('\'') + 1);
            object = object.substr(0, object.find('\''));
            type_locations[coords] = object;
        }
        std_msgs::msg::String locations;
        locations.data = "";
        for (auto p : type_locations) {
            pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
            pcl_conversions::toPCL(_cloud, *cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conversion(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromPCLPointCloud2(*cloud, *cloud_conversion);
            RCLCPP_INFO(get_logger(), "OBJECT: (%d, %d) - %s", p.first.first, p.first.second, p.second.c_str());
            double percent_x = p.first.first / 1024.0; // 544 is height, 1024 is width for multisense
            double percent_y = p.first.second / 544.0; // This is mapping to the combined camera from the left

            double min_x = 1e99;
            double max_x = -1e99;
            double min_y = 1e99;
            double max_y = -1e99;
            for (auto pt : cloud_conversion->points) {
                min_x = min_x < pt.x ? min_x : pt.x;
                max_x = max_x > pt.x ? max_x : pt.x;
                min_y = min_y < pt.y ? min_y : pt.y;
                max_y = max_y > pt.y ? max_y : pt.y;
            }
            // RCLCPP_INFO(get_logger(), "%lf-%lf, %lf-%lf", min_x, max_x, min_y, max_y);
            pcl::PointXYZ pt;
            pt.x = min_x + (max_x - min_x) * percent_x;
            pt.y = max_y + (min_y - max_y) * percent_y;
            // RCLCPP_INFO(get_logger(), "BEFORE: %f, %f, %f", pt.x, pt.y, pt.z); 
            pcl::PointXYZ closest_real_point;
            for (auto point : cloud_conversion->points) {
                if ((abs(point.x - pt.x) < abs(closest_real_point.x - pt.x)) && (abs(point.y - pt.y) < abs(closest_real_point.y - pt.y)))
                    closest_real_point = point;
            }
            pt = closest_real_point;
            RCLCPP_INFO(get_logger(), "AFTER: %f, %f, %f", pt.x, pt.y, pt.z); 
            // RCLCPP_INFO(get_logger(), "NO?");
            locations.data += "[" + p.second + ": (" + std::to_string(pt.x) + " " + std::to_string(pt.y)  + " " + std::to_string(pt.z) + ")]\n";
        }
        _publisher_info->publish(locations);
        return objects_types;
    }
    void _on_info(std_msgs::msg::String msg) {
        parse_info(msg);
    }

public:
    Pipe() : Node("pipe") {
        _group_cloud = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options_cloud;
        options_cloud.callback_group = _group_cloud;
        _subscriber_cloud = create_subscription<sensor_msgs::msg::PointCloud2>("multisense/image_points2", 1, std::bind(&Pipe::_on_subscriber, this, std::placeholders::_1), options_cloud);
        _publisher_cloud = create_publisher<sensor_msgs::msg::PointCloud2>("piped_pointcloud", 10);

        _group_info = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options_info;
        options_info.callback_group = _group_info;
        _subscriber_info = create_subscription<std_msgs::msg::String>("detected_locations", 1, std::bind(&Pipe::_on_info, this, std::placeholders::_1), options_info);
        _publisher_info = create_publisher<std_msgs::msg::String>("space_to_type", 10);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto pipe = std::make_shared<Pipe>();
    rclcpp::executors::MultiThreadedExecutor ex;
    ex.add_node(pipe);
    ex.spin();
    rclcpp::shutdown();
    return 0;
}