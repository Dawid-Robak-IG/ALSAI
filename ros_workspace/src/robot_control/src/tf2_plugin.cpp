#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <thread>
#include <memory>
#include <iostream>

namespace gazebo
{
class GazeboTfPlugin : public ModelPlugin
{
    physics::ModelPtr model_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::shared_ptr<std::thread> spinner_thread_;

    std::string odom_topic_;
    std::string base_link_frame_;
public:
    GazeboTfPlugin() = default;
    ~GazeboTfPlugin(){
        if (executor_){
        executor_->cancel();
        }
        if (spinner_thread_ && spinner_thread_->joinable()){
        spinner_thread_->join();
        }
    }
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        model_ = _model;

        if (!rclcpp::ok()) {
            int argc = 0;
            char **argv = nullptr;
            rclcpp::init(argc, argv);
        }

        std::cout<< "Going to create node for tf plugin\n";

        node_ = rclcpp::Node::make_shared("tf2_plugin_node");

        RCLCPP_INFO(node_->get_logger(), "Node created");

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

        odom_topic_ = "/odom";

        base_link_frame_ = "base_link";

        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&GazeboTfPlugin::OdomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "Odom sub created");
        
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
        BroadcastStaticLaserTf();

        RCLCPP_INFO(node_->get_logger(), "Static laser broadcasted");

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        spinner_thread_ = std::make_shared<std::thread>([this]() {
            this->executor_->spin();
        });

        RCLCPP_INFO(node_->get_logger(), "GazeboTfPlugin loaded");
    }
private:
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = "odom";  
        tf_msg.child_frame_id = "base_link"; 
        tf_msg.transform.translation.x = msg->pose.pose.position.x;
        tf_msg.transform.translation.y = msg->pose.pose.position.y;
        tf_msg.transform.translation.z = msg->pose.pose.position.z;
        tf_msg.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(tf_msg);
    }
    void BroadcastStaticLaserTf()
    {
        geometry_msgs::msg::TransformStamped static_tf;

        static_tf.header.stamp = node_->get_clock()->now();
        static_tf.header.frame_id = "base_link";
        static_tf.child_frame_id = "laser_link";

        static_tf.transform.translation.x = 0.01;
        static_tf.transform.translation.y = 0.0;
        static_tf.transform.translation.z = 0.2;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        static_tf.transform.rotation.x = q.x();
        static_tf.transform.rotation.y = q.y();
        static_tf.transform.rotation.z = q.z();
        static_tf.transform.rotation.w = q.w();

        static_broadcaster_->sendTransform(static_tf);
    }
};
    GZ_REGISTER_MODEL_PLUGIN(GazeboTfPlugin)
}  // namespace gazebo
