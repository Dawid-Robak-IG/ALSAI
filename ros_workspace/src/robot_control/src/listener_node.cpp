#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <filesystem>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

class StateListener : public rclcpp::Node{
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub; 
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;

    std::string sessions_path;
    std::ofstream odom_file;
    std::ofstream joint_file;
    std::ofstream laser_file;

    StateListener() : Node("StateListener"){
        sessions_path = "~/inzynierka/ALSAI/sessions";
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",
            10,
            std::bind(&StateListener::odom_callback, this, std::placeholders::_1)
        );
        joint_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",
            10,
            std::bind(&StateListener::joint_callback, this, std::placeholders::_1)
        );
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "laser/out ",
            10,
            std::bind(&StateListener::laser_callback, this, std::placeholders::_1)
        );

        std::string created_path = create_session_folder(sessions_path);
        open_files(created_path);
    }
    ~StateListener(){
        odom_file.close();
        laser_file.close();
        joint_file.close();
    }


    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);        
        double v_linear = msg->twist.twist.linear.x;
        double v_angular = msg->twist.twist.angular.z;

        odom_file << msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9
                << "," << x << "," << y << "," << yaw
                << "," << v_linear << "," << v_angular << "\n";
    }
    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
        double left_vel = msg->velocity[0];
        double right_vel = msg->velocity[1];

        joint_file << msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9
                << "," << left_vel << "," << right_vel << "\n";
    }
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        laser_file << msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9 << ",";
        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            laser_file << msg->ranges[i];
            if (i < msg->ranges.size()-1) laser_file << ",";
        }
        laser_file << "\n";
    }
    std::string create_session_folder(const std::string &base_path){
        auto t = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(t);
        std::tm tm = *std::localtime(&tt);

        std::ostringstream folder_name;
        folder_name << "session_" << std::put_time(&tm, "%S-%M-%H_%d-%d-%Y");

        fs::path session_path = fs::path(base_path) / folder_name.str();

        if(!fs::exists(session_path)){
            fs::create_directories(session_path);
        }

        return session_path.string();
    }
    void open_files(const std::string &session_folder){
        odom_file.open(session_folder + "/odom.txt", std::ios::out);
        laser_file.open(session_folder + "/scan.txt", std::ios::out);
        joint_file.open(session_folder + "/joint.txt", std::ios::out);

        odom_file << "time,x,y,yaw,v_linear,v_angular\n";
        joint_file << "time,left_wheel_vel,right_wheel_vel\n";
        laser_file << "time,scan_data\n";
    }

};


int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);

    rclcpp::shutdown();
    return 0;
}