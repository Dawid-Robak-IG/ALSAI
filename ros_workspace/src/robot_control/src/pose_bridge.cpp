#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <thread>
#include <memory>

namespace gazebo
{
  class PoseBridgePlugin : public ModelPlugin
  {
  public:
    ~PoseBridgePlugin()
    {
      if (executor_)
      {
        executor_->cancel();
      }
      if (spinner_thread_ && spinner_thread_->joinable())
      {
        spinner_thread_->join();
      }
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
      model_ = _model;

      if (!rclcpp::ok())
        rclcpp::init(0, nullptr);

      ros_node_ = rclcpp::Node::make_shared("pose_bridge_model");
      pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);

      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(ros_node_);
      spinner_thread_ = std::make_shared<std::thread>([this]() {
          this->executor_->spin();
      });

      update_connection_ = event::Events::ConnectWorldUpdateBegin(
          std::bind(&PoseBridgePlugin::OnUpdate, this));

      RCLCPP_INFO(ros_node_->get_logger(), "PoseBridgePlugin loaded for model: %s", model_->GetName().c_str());
    }

  private:
    void OnUpdate()
    {
      auto pose = model_->WorldPose();


      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = ros_node_->get_clock()->now();
      msg.header.frame_id = "world";

      msg.pose.position.x = pose.Pos().X();
      msg.pose.position.y = pose.Pos().Y();
      msg.pose.position.z = pose.Pos().Z();

      msg.pose.orientation.x = pose.Rot().X();
      msg.pose.orientation.y = pose.Rot().Y();
      msg.pose.orientation.z = pose.Rot().Z();
      msg.pose.orientation.w = pose.Rot().W();

      pose_pub_->publish(msg);
    }

    physics::ModelPtr model_;
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::shared_ptr<std::thread> spinner_thread_;
    event::ConnectionPtr update_connection_;
  };

  GZ_REGISTER_MODEL_PLUGIN(PoseBridgePlugin)
}
