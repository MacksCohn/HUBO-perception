#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/search/kdtree.h"
#include "std_msgs/msg/string.hpp"
#include "pcl/filters/voxel_grid.h"

#include <chrono>
#include "tf2_ros/transform_broadcaster.h"

#include <cstdlib>
#include <map>
// Run the following to get point cloud information
// gazebo /opt/ros/humble/share/gazebo_plugins/worlds/gazebo_ros_depth_camera_demo.world

const std::string SUB_TOPIC = "piped_pointcloud";
const double MAX_DIST = 10.0;
const double MAX_DIST2 = MAX_DIST * MAX_DIST;
const double FLOOR_HEIGHT = 3;

class Vector3 {
    public:
        double x;
        double y;
        double z;
        Vector3(double x=0.0, double y=0.0, double z=0.0) : x(x), y(y), z(z) {} 
};

class PointCloudParser : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscriber_cloud;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber_parsing;

    rclcpp::CallbackGroup::SharedPtr _group_cloud;
    rclcpp::CallbackGroup::SharedPtr _group_parsing;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _object_location_broadcaster;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_centers;
    sensor_msgs::msg::PointCloud2 _last_cloud;

    std::map<std::string, std::pair<pcl::PointXYZ, rclcpp::Time>> _point_names;


    void _on_subscriber(sensor_msgs::msg::PointCloud2 initial_cloud) {
        parse_cloud(initial_cloud);
    }

    // Takes in piped points and parses the data into usuable format
    void _on_info(std_msgs::msg::String msg) {
        std::string data = msg.data;
        size_t num_objects = std::count(data.begin(), data.end(), '[');
        std::vector<std::string> used_names;
        for (size_t i = 0; i < num_objects; i++) {
            std::string object = data.substr(0, data.find(']'));
            std::string type = object.substr(object.find('[')+1,object.find(':')-2);
            pcl::PointXYZ pt;
            object = object.substr(object.find('(')+1);
            if (object.substr(0, object.find(' ')) != "nan")
                pt.x = std::stof(object.substr(0, object.find(' ')));
            object = object.substr(object.find(' ')+1);
            if (object.substr(0, object.find(' ')) != "nan")
                pt.y = std::stof(object.substr(0, object.find(' ')));
            object = object.substr(object.find(' ')+1);
            if (object.substr(0, object.find(')')) != "nan")
                pt.z = std::stof(object.substr(0, object.find(')'))); 
            // bc of these if statements the top left object will be named whatever is nan
            used_names.push_back(replace_closest(_point_names, pt, type));
            data = data.substr(data.find(']')+1);
            RCLCPP_INFO(get_logger(), "BEFORE_BROADCAST: %f, %f, %f", pt.x, pt.y, pt.z); 
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "camera_link";
            t.child_frame_id = "TEST_OBJECT_HERE";
            t.transform.translation.x = pt.x;
            t.transform.translation.y = pt.y;
            t.transform.translation.z = pt.z;
            _object_location_broadcaster->sendTransform(t);
        }
        remove_all_except(_point_names, used_names);
    }

    // Publish Tfs, Cluster Clouds, publish center points
    void parse_cloud(sensor_msgs::msg::PointCloud2 initial_cloud) {
        double TOLERANCE = get_parameter("TOLERANCE").as_double();
        std::vector<sensor_msgs::msg::PointCloud2> clusters;
        // convert to pcl cloud
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(initial_cloud, *cloud);
        // convert from pointcloud2 to pointcloud to then use algorithms on
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_conversion(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(*cloud, *cloud_conversion);
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        *object_cluster_cloud = *cloud_conversion;
        // REMOVE FLOOR
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(TOLERANCE);
        seg.setInputCloud(object_cluster_cloud);
        seg.segment(*inliers, *coefficients);
        // Remove the floor from the cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(object_cluster_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr removed_floor(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*removed_floor);
        // REMOVE TABLE
        pcl::PointCloud<pcl::PointXYZ>::Ptr removed_table(new pcl::PointCloud<pcl::PointXYZ>);
        if (get_parameter("REMOVE_FLOOR").as_bool()) {
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(get_parameter("TOLERANCE").as_double());
            seg.setInputCloud(removed_floor);
            seg.segment(*inliers, *coefficients);
            // Remove the table from the cloud
            extract.setInputCloud(removed_floor);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*removed_table);
        }
        else {
            *removed_table = *removed_floor;
        }
        // Extract clusters
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> clusterer;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method(new pcl::search::KdTree<pcl::PointXYZ>);
        std::vector<pcl::PointIndices> cluster_vector;
        clusterer.setClusterTolerance(TOLERANCE);
        clusterer.setInputCloud(removed_table);
        clusterer.setMinClusterSize(get_parameter("MIN_CLUSTER_SIZE").as_int());
        clusterer.setMaxClusterSize(get_parameter("MAX_CLUSTER_SIZE").as_int());
        clusterer.setSearchMethod(search_method);
        clusterer.extract(cluster_vector);
        // Find and publish cluster centers
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_centers(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto cluster : cluster_vector) {
            pcl::PointXYZ center_point;
            for (auto index : cluster.indices) {
                pcl::PointXYZ pt = removed_table->points[index];
                center_point.x += pt.x;
                center_point.y += pt.y;
                center_point.z += pt.z;
                output_cloud->push_back(pt);
            }
            center_point.x /= cluster.indices.size();
            center_point.y /= cluster.indices.size();
            center_point.z /= cluster.indices.size();
            output_centers->push_back(center_point);
        }
        // Output final cloud
        output_cloud->width = output_cloud->size();
        output_cloud->height = 1;
        output_cloud->is_dense = true;
        output_cloud->header.frame_id = cloud_conversion->header.frame_id;
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*output_cloud, msg);
        _publisher->publish(msg);
        // Broadcast centers as tfs
        name_closest_points(output_centers);
        // output a cloud of just those centers as well (NOT NECESSARY)
        output_centers->width = output_centers->size();
        output_centers->height = 1;
        output_centers->is_dense = true;
        output_centers->header.frame_id = cloud_conversion->header.frame_id;
        sensor_msgs::msg::PointCloud2 msg_centers;
        pcl::toROSMsg(*output_centers, msg_centers);
        _publisher_centers->publish(msg_centers);
    }

    std::string replace_closest(std::map<std::string, std::pair<pcl::PointXYZ, rclcpp::Time>> &point_map, pcl::PointXYZ pt, std::string type) {
        std::string modified_name;
        // find closest point
        double TOLERANCE = 0.01;
        double distance2 = static_cast<double>(INT_MAX);
        std::pair<std::string, std::pair<pcl::PointXYZ, rclcpp::Time>> closest_pt;
        size_t type_count = 0;
        for (auto pair : point_map) {
            double temp_dist2 = dist2(pair.second.first, pt);
            if (temp_dist2 < distance2) {
                distance2 = temp_dist2;
                closest_pt = pair;
            }
            if (pair.first.substr(0, pair.first.find("_")) == type) {
                type_count++;
            }
        }
        // Keep names consistent, or make new ones if the object is new
        if (distance2 >= TOLERANCE) {
            modified_name = (type + "_" + std::to_string(type_count));
            point_map[modified_name] = std::pair<pcl::PointXYZ, rclcpp::Time>(pt, this->get_clock()->now());
        }
        else if (closest_pt.first.substr(0, closest_pt.first.find("_")) == type) {
            point_map.erase(closest_pt.first);
            point_map[closest_pt.first] = std::pair<pcl::PointXYZ, rclcpp::Time>(pt, this->get_clock()->now());
            modified_name = closest_pt.first;
        }
        else {
            modified_name = type + "_" + std::to_string(type_count);
            point_map[modified_name] = std::pair<pcl::PointXYZ, rclcpp::Time>(pt, this->get_clock()->now());
        }
        return modified_name;
    }

    double dist2(pcl::PointXYZ a, pcl::PointXYZ b) {
        return pow(b.x - a.x, 2) + pow(b.y - a.y, 2) + pow(b.z - a.z, 2);
    }
    
    // used to cull old and unused points
    void remove_all_except(std::map<std::string, std::pair<pcl::PointXYZ, rclcpp::Time>>& map, std::vector<std::string> not_to_be_removed) {
        std::vector<std::string> to_remove;
        rclcpp::Time current_time = this->get_clock()->now();
        int LIFETIME = get_parameter("LIFETIME").as_int();
        for (auto pair : map) {
            if (std::find(not_to_be_removed.begin(), not_to_be_removed.end(), pair.first) == not_to_be_removed.end() && (current_time.nanoseconds() - pair.second.second.nanoseconds() > LIFETIME))
                to_remove.push_back(pair.first);
        }
        for (std::string str : to_remove)
            map.erase(str);
    }
public:
    PointCloudParser() : Node("pointcloud_parser") {
        using namespace std::chrono_literals;
        _group_cloud = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options_cloud;
        options_cloud.callback_group = _group_cloud;

        _group_parsing = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options_parsing;
        options_parsing.callback_group = _group_parsing;

        _subscriber_cloud = create_subscription<sensor_msgs::msg::PointCloud2>(SUB_TOPIC, 1, std::bind(&PointCloudParser::_on_subscriber, this, std::placeholders::_1), options_cloud);
        _subscriber_parsing = create_subscription<std_msgs::msg::String>("space_to_type", 1, std::bind(&PointCloudParser::_on_info, this, std::placeholders::_1), options_parsing);

        _publisher = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_point_cloud", 1);
        _object_location_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        _publisher_centers = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_centers", 10);
        declare_parameter<int>("MIN_CLUSTER_SIZE");
        declare_parameter<int>("MAX_CLUSTER_SIZE");
        declare_parameter<double>("TABLE_HEIGHT");
        declare_parameter<double>("TOLERANCE");
        declare_parameter<bool>("REMOVE_FLOOR");
        declare_parameter<int>("LIFETIME");
        set_parameter(rclcpp::Parameter("MIN_CLUSTER_SIZE", 3));
        set_parameter(rclcpp::Parameter("MAX_CLUSTER_SIZE", 100));
        set_parameter(rclcpp::Parameter("TABLE_HEIGHT", 2.1));
        set_parameter(rclcpp::Parameter("TOLERANCE", 0.018));
        set_parameter(rclcpp::Parameter("REMOVE_FLOOR", true));
        set_parameter(rclcpp::Parameter("LIFETIME", 5 * 1e9));
    }
    
    // Gets the closest stored name for a cluster
    // Could reverse order so it gets closest object to name rather than closest name to the object
    std::string get_closest_name(pcl::PointXYZ pt, size_t num) {
        double distance2 = static_cast<double>(INT_MAX); //
        double TOLERANCE = get_parameter("TOLERANCE").as_double();
        std::string closest_name;
        for (auto pair : _point_names) {
            double temp2 = dist2(pair.second.first, pt); // pow(pt.x - pair.first.x, 2) + pow(pt.y - pair.first.y, 2) + pow(pt.z-pair.first.z, 2);
            if (temp2 < distance2) {
                distance2 = temp2;
                closest_name = pair.first;
            }
        }
        if (closest_name == "" || distance2 >= TOLERANCE)
            closest_name = "_" + std::to_string(num);
        return closest_name;
    }

    void name_closest_points(pcl::PointCloud<pcl::PointXYZ>::Ptr center_points) {
        double TOLERANCE = get_parameter("TOLERANCE").as_double();
        // get closest point to the name
        for (auto pair : _point_names) {
            // RCLCPP_INFO(get_logger(), "___%s___", pair.first.c_str());
            // find closest point, save point as closest
            double distance2 = static_cast<double>(INT_MAX);
            pcl::PointXYZ closest_point;
            for (size_t pt = 0; pt < center_points->points.size(); pt++) {
                double temp_dist2 = dist2(center_points->points[pt], pair.second.first);
                if (temp_dist2 < distance2) {
                    distance2 = temp_dist2;
                    closest_point = center_points->points[pt];
                }
            }
            // if that is within tolerance,
            // publish that name
            if (distance2 < TOLERANCE) {
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "camera_link";
                std::string name = pair.first;
                t.child_frame_id = name;
                t.transform.translation.x = closest_point.x;
                t.transform.translation.y = closest_point.y;
                t.transform.translation.z = closest_point.z;
                // For rotation, some factor of pi/2 - angle helps
                _object_location_broadcaster->sendTransform(t);
            }
        }
        // Somehow name the others ?
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor ex;
    auto parser = std::make_shared<PointCloudParser>();
    ex.add_node(parser);
    ex.spin();
    rclcpp::shutdown();
    return 0;
}

// KNOWN ISSUES
/*
If two objects are close together, objects sort of name incorrectly
Set up vm with ros melodic
ubuntu 20.04 vm - docker
*/