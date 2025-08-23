#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>

namespace gazebo
{
  class PoseBridgePlugin : public ModelPlugin
  {
  public:
    ~PoseBridgePlugin(){
        if (executor_) executor_->cancel();
        if (spinner_thread_ && spinner_thread_->joinable()) spinner_thread_->join();
    }
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) override
    {
      model_ = _model;

      if (!rclcpp::ok())
        rclcpp::init(0, nullptr);

      ros_node_ = rclcpp::Node::make_shared("pose_bridge_model");
      pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);

      gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
      gz_node_->Init(model_->GetWorld()->Name());

      pose_sub_ = gz_node_->Subscribe("/gazebo/default/pose/info",
                                      &PoseBridgePlugin::OnPoseMsg, this);

      RCLCPP_INFO(ros_node_->get_logger(), "PoseBridgePlugin loaded for model: %s", model_->GetName().c_str());

      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(ros_node_);
      spinner_thread_ = std::make_shared<std::thread>([this]() {
          this->executor_->spin();
      });
    }

  private:
    void OnPoseMsg(ConstPosesStampedPtr &msg)
    {
      for (int i = 0; i < msg->pose_size(); ++i)
      {
        std::string full_link_name = model_->GetName();
        if (msg->pose(i).name() == full_link_name)
        {
          geometry_msgs::msg::PoseStamped pose_msg;
          pose_msg.header.stamp = ros_node_->get_clock()->now();
          pose_msg.header.frame_id = "world";

          pose_msg.pose.position.x = msg->pose(i).position().x();
          pose_msg.pose.position.y = msg->pose(i).position().y();
          pose_msg.pose.position.z = msg->pose(i).position().z();

          pose_msg.pose.orientation.x = msg->pose(i).orientation().x();
          pose_msg.pose.orientation.y = msg->pose(i).orientation().y();
          pose_msg.pose.orientation.z = msg->pose(i).orientation().z();
          pose_msg.pose.orientation.w = msg->pose(i).orientation().w();

          pose_pub_->publish(pose_msg);
        }
      }
    }

    physics::ModelPtr model_;
    gazebo::transport::NodePtr gz_node_;
    gazebo::transport::SubscriberPtr pose_sub_;
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::shared_ptr<std::thread> spinner_thread_;
  };

  GZ_REGISTER_MODEL_PLUGIN(PoseBridgePlugin)
}
