#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>

    std::vector<std::pair<double, double>> loadPathFromYaml(const std::string& yaml_file) {
    std::vector<std::pair<double, double>> path_data;
    
    try {
        YAML::Node config = YAML::LoadFile(yaml_file);
        
        if (config.IsSequence()) {
            for (const auto& point : config) {
                if (point.IsSequence() && point.size() >= 2) {
                    double x = point[0].as<double>();
                    double y = point[1].as<double>();
                    path_data.push_back(std::make_pair(x, y));
                }
            }
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Failed to load %s: %s", yaml_file.c_str(), e.what());
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to open %s: %s", yaml_file.c_str(), e.what());
    }
    
    return path_data;
}

// void publishPathOnce() {
//     ros::NodeHandle nh;
//     ros::Publisher publisher = nh.advertise<nav_msgs::Path>("/turtlepath", 1, true); // true for latching
    
//     std::vector<std::pair<double, double>> path_data = loadPathFromYaml("/home/bairi/project/full_path/a.yaml");
    
//     if (path_data.empty()) 
//     {
//         ROS_WARN("No path data available; aborting publish");
//         return;
//     }
    
//     nav_msgs::Path path_msg;
//     path_msg.header.frame_id = "map";
//     path_msg.header.stamp = ros::Time::now();
    
//     ros::Time stamp = ros::Time::now();
    
//     for (const auto& point : path_data) {
//         geometry_msgs::PoseStamped pose;
//         pose.header.frame_id = "map";
//         pose.header.stamp = ros::Time::now();
//         pose.pose.position.x = point.first;
//         pose.pose.position.y = point.second;
//         pose.pose.position.z = 0.0;
//         pose.pose.orientation.w = 1.0;
//         path_msg.poses.push_back(pose);
//     }
    
//     publisher.publish(path_msg);
//     ROS_INFO("路径已发布到 /path 话题");
    
//     ros::Duration(3).sleep();
// }

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "path_publisher_once");
    // publishPathOnce();
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<nav_msgs::Path>("/turtlepath", 1000, true); // true for latching
    
    std::vector<std::pair<double, double>> path_data = loadPathFromYaml("/home/bairi/project/full_path/a.yaml");
    
    if (path_data.empty()) 
    {
        ROS_WARN("No path data available; aborting publish");
        return 0;
    }
    
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    ros::Time stamp = ros::Time::now();
    while (ros::ok())
    {
        for (const auto& point : path_data) {
        geometry_msgs::PoseStamped pose;
        // pose.header.frame_id = "map";
        // pose.header.stamp = ros::Time::now();
        pose.pose.position.x = point.first;
        pose.pose.position.y = point.second;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 0.0;
        path_msg.poses.push_back(pose);
    }
    publisher.publish(path_msg);
    ros::spin();
    }
    // ROS_INFO("路径已发布到 /path 话题");

    return 0;
}