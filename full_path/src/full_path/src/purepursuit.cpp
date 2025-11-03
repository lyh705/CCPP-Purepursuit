// #include <ros/ros.h>
// #include <turtlesim/Pose.h>
// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <tf/transform_datatypes.h>
// #include <vector>

// class TurtlePathDrawer {
// private:
//     ros::NodeHandle nh_;
//     ros::Subscriber path_sub_;
//     ros::Publisher velocity_pub_;
//     ros::Subscriber pose_sub_;
    
//     turtlesim::Pose current_pose_;
//     std::vector<geometry_msgs::Point> path_points_;
//     int current_waypoint_index_;
//     bool path_received_;
//     double linear_speed_;
//     double angular_speed_;
//     double tolerance_;

// public:
//     TurtlePathDrawer() : current_waypoint_index_(0), path_received_(false), 
//                         linear_speed_(5.0), angular_speed_(3.0), tolerance_(0.1) {
        
//         // 订阅路径话题
//         path_sub_ = nh_.subscribe("/turtlepath", 10, &TurtlePathDrawer::pathCallback, this);
        
//         // 订阅小海龟位姿
//         pose_sub_ = nh_.subscribe("/turtle1/pose", 10, &TurtlePathDrawer::poseCallback, this);
        
//         // 发布控制命令
//         velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
//     }

//     void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
//         current_pose_ = *msg;
//     }

//     void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
//         ROS_INFO("Received path with %lu waypoints", msg->poses.size());
        
//         path_points_.clear();
//         for (const auto& pose_stamped : msg->poses) {
//             geometry_msgs::Point point;
//             point.x = pose_stamped.pose.position.x;
//             point.y = pose_stamped.pose.position.y;
//             path_points_.push_back(point);
//         }
        
//         current_waypoint_index_ = 0;
//         path_received_ = true;
        
//         ROS_INFO("Path stored successfully");
//     }

//     double calculateDistance(double x1, double y1, double x2, double y2) {
//         return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
//     }

//     double normalizeAngle(double angle) {
//         while (angle > M_PI) angle -= 2 * M_PI;
//         while (angle < -M_PI) angle += 2 * M_PI;
//         return angle;
//     }

//     void followPath() {
//         if (!path_received_ || path_points_.empty() || current_waypoint_index_ >= path_points_.size()) {
//             return;
//         }

//         // 获取当前目标点
//         geometry_msgs::Point target_point = path_points_[current_waypoint_index_];
        
//         // 计算到目标点的距离和角度
//         double distance = calculateDistance(current_pose_.x, current_pose_.y, 
//                                           target_point.x, target_point.y);
//         double target_angle = atan2(target_point.y - current_pose_.y, 
//                                    target_point.x - current_pose_.x);
//         double angle_diff = normalizeAngle(target_angle - current_pose_.theta);

//         geometry_msgs::Twist cmd_vel;

//         if (distance > tolerance_) {
//             // 如果角度偏差较大，先调整方向
//             if (fabs(angle_diff) > 0.1) {
//                 cmd_vel.angular.z = angular_speed_ * angle_diff;
//                 cmd_vel.linear.x = 0.0;
//             } else {
//                 // 朝目标点移动
//                 cmd_vel.linear.x = linear_speed_ * std::min(distance, 1.0);
//                 cmd_vel.angular.z = angular_speed_ * angle_diff;
//             }
//         } else {
//             // 到达当前目标点，移动到下一个
//             ROS_INFO("Reached waypoint %d", current_waypoint_index_);
//             current_waypoint_index_++;
            
//             if (current_waypoint_index_ >= path_points_.size()) {
//                 ROS_INFO("Path completed!");
//                 cmd_vel.linear.x = 0.0;
//                 cmd_vel.angular.z = 0.0;
//             }
//         }

//         velocity_pub_.publish(cmd_vel);
//     }

//     void run() {
//         ros::Rate rate(10); // 10Hz
        
//         while (ros::ok()) {
//             ros::spinOnce();
            
//             if (path_received_) {
//                 followPath();
//             }
            
//             rate.sleep();
//         }
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "turtle_path_drawer");
    
//     TurtlePathDrawer drawer;
//     drawer.run();
    
//     return 0;
// }

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <cmath>
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
class TurtlePathDrawer {
private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Publisher velocity_pub_;
    ros::Subscriber pose_sub_;
    
    turtlesim::Pose current_pose_;
    std::vector<geometry_msgs::Point> path_points_;
    int current_waypoint_index_;
    bool path_received_;
    double linear_speed_;
    double angular_speed_;
    double tolerance_;
    
    // 纯追踪参数
    double look_ahead_distance_;
    int current_path_index_;
    double max_linear_speed_;
    double min_linear_speed_;
    double max_angular_speed_;
    double curvature_threshold_;

public:
    TurtlePathDrawer() : current_waypoint_index_(0), path_received_(false), 
                        linear_speed_(2.0), angular_speed_(2.0), tolerance_(0.1),
                        look_ahead_distance_(0.2), current_path_index_(0),
                        max_linear_speed_(3.0), min_linear_speed_(0.2),
                        max_angular_speed_(2.0), curvature_threshold_(0.5) {
        
        path_sub_ = nh_.subscribe("/turtlepath", 10, &TurtlePathDrawer::pathCallback, this);
        pose_sub_ = nh_.subscribe("/turtle1/pose", 10, &TurtlePathDrawer::poseCallback, this);
        velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    }

    void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
        current_pose_ = *msg;
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        ROS_INFO("Received path with %lu waypoints", msg->poses.size());
        
        path_points_.clear();
        for (const auto& pose_stamped : msg->poses) {
            geometry_msgs::Point point;
            point.x = pose_stamped.pose.position.x;
            point.y = pose_stamped.pose.position.y;
            path_points_.push_back(point);
        }
        
        current_path_index_ = 0;
        path_received_ = true;
        
        ROS_INFO("Path stored successfully with %lu points", path_points_.size());
    }

    double calculateDistance(double x1, double y1, double x2, double y2) {
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    // 纯追踪：寻找前瞻点
    geometry_msgs::Point findLookAheadPoint() {
        geometry_msgs::Point look_ahead_point;
        look_ahead_point.x = 0.0;
        look_ahead_point.y = 0.0;
        
        if (path_points_.empty()) {
            return look_ahead_point;
        }

        // 从当前路径索引开始寻找
        for (int i = current_path_index_; i < path_points_.size(); i++) {
            double distance = calculateDistance(current_pose_.x, current_pose_.y, 
                                             path_points_[i].x, path_points_[i].y);
            
            if (distance >= look_ahead_distance_) {
                current_path_index_ = i;  // 更新当前路径索引
                return path_points_[i];
            }
        }
        
        // 如果没有找到足够远的点，返回路径终点
        if (!path_points_.empty()) {
            current_path_index_ = path_points_.size() - 1;
            return path_points_.back();
        }
        
        return look_ahead_point;
    }

    // 计算曲率
    double calculateCurvature(const geometry_msgs::Point& look_ahead_point) {
        // 计算相对位置
        double dx = look_ahead_point.x - current_pose_.x;
        double dy = look_ahead_point.y - current_pose_.y;
        
        // 计算相对于机器人坐标系的角度
        double alpha = atan2(dy, dx) - current_pose_.theta;
        alpha = normalizeAngle(alpha);
        
        // 计算曲率：k = 2 * sin(alpha) / look_ahead_distance
        double curvature = 2.0 * sin(alpha) / look_ahead_distance_;
        
        return curvature;
    }

    // 根据曲率调整速度
    double adjustSpeedByCurvature(double curvature) {
        // 曲率越大，速度越小
        double speed_factor = 1.0 / (1.0 + fabs(curvature) * 2.0);
        double adjusted_speed = max_linear_speed_ * speed_factor;
        
        // 限制最小速度
        return std::max(adjusted_speed, min_linear_speed_);
    }

    void purePursuitControl() {
        if (!path_received_ || path_points_.empty() || current_path_index_ >= path_points_.size()) {
            return;
        }

        // 寻找前瞻点
        geometry_msgs::Point look_ahead_point = findLookAheadPoint();
        
        // 计算到前瞻点的距离
        double distance_to_look_ahead = calculateDistance(
            current_pose_.x, current_pose_.y, 
            look_ahead_point.x, look_ahead_point.y
        );
        
        // 计算曲率
        double curvature = calculateCurvature(look_ahead_point);
        
        // 根据曲率调整线速度
        double adjusted_linear_speed = adjustSpeedByCurvature(curvature);
        
        // 计算目标角度
        double target_angle = atan2(
            look_ahead_point.y - current_pose_.y,
            look_ahead_point.x - current_pose_.x
        );
        
        // 计算角度差
        double angle_diff = normalizeAngle(target_angle - current_pose_.theta);
        
        geometry_msgs::Twist cmd_vel;
        
        // 判断是否接近终点
        bool isNearEnd = (current_path_index_ == path_points_.size() - 1);
        double distance_to_end = calculateDistance(
            current_pose_.x, current_pose_.y,
            path_points_.back().x, path_points_.back().y
        );
        
        if (isNearEnd && distance_to_end < tolerance_) {
            // 接近终点，停止
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 6.0;
            ROS_INFO("Reached the end of path!");
        } else {
            // 纯追踪控制
            cmd_vel.linear.x = adjusted_linear_speed;
            
            // 角速度控制：根据角度差和曲率
            double angular_control = max_angular_speed_ * angle_diff;
            
            // 限制角速度
            angular_control = std::max(std::min(angular_control, max_angular_speed_), -max_angular_speed_);
            
            cmd_vel.angular.z = angular_control;
            
            // 额外的曲率控制（可选）
            cmd_vel.angular.z += curvature * max_angular_speed_ * 0.5;
            
            // 限制角速度范围
            cmd_vel.angular.z = std::max(std::min(cmd_vel.angular.z, max_angular_speed_), -max_angular_speed_);
        }
        
        // 发布控制命令
        velocity_pub_.publish(cmd_vel);
        
        // 调试信息
        ROS_DEBUG("Look-ahead point: (%.2f, %.2f)", look_ahead_point.x, look_ahead_point.y);
        ROS_DEBUG("Distance to look-ahead: %.2f, Angle diff: %.2f, Curvature: %.2f", 
                 distance_to_look_ahead, angle_diff, curvature);
        ROS_DEBUG("Linear speed: %.2f, Angular speed: %.2f", cmd_vel.linear.x, cmd_vel.angular.z);
    }

    void run() {
        ros::Rate rate(100); // 提高频率到20Hz以获得更好的跟踪效果
        
        while (ros::ok()) {
            ros::spinOnce();
            
            if (path_received_) {
                purePursuitControl();
            }
            
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_path_drawer_pure_pursuit");
    
    TurtlePathDrawer drawer;
    
    drawer.run();
    
    return 0;
}
