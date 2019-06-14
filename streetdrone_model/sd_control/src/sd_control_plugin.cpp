#include "sd_control/sd_control_plugin.hpp"

#include <gazebo/physics/physics.hh>

namespace sd_control
{

  SdControlPlugin::SdControlPlugin()
    : fl_wheel_radius_{0.265},
      fr_wheel_radius_{0.265},
      bl_wheel_radius_{0.281},
      br_wheel_radius_{0.281},
      robot_namespace_{""}
  {
  }

  void SdControlPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    std::cerr << "Loading plugin!!\n";
    ROS_INFO("Loading plugin!");

    model_ = model_;
    world_ = model_->GetWorld();
    auto physicsEngine = world_->Physics();
    physicsEngine->SetParam("friction_model", std::string{"cone_model"});

    gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gznode_->Init();

    if (sdf->HasElement("robotNamespace"))
      robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

    ros::NodeHandle nh(this->robot_namespace_);
    throttle_sub_ = nh.subscribe(
      "/throttle", 10, &SdControlPlugin::throttleCallback, this
      );

    steer_sub_ = nh.subscribe(
      "/steer", 10, &SdControlPlugin::steerCallback, this
      );

    auto findLink = [&](std::string const& link_name) {
                      auto full_link_name = model_->GetName() + "::"
                        + sdf->Get<std::string>(link_name);
                      auto link = model_->GetLink(full_link_name);
                      if (!link)
                        std::cerr << "could not find link: " << full_link_name << "\n";
                      return link;
                    };

    auto findJoint = [&](std::string const& joint_name) {
                       auto full_joint_name = model_->GetName() + "::"
                         + sdf->Get<std::string>(joint_name);
                       auto joint = model_->GetJoint(full_joint_name);
                       if (!joint)
                         std::cerr << "could not find joint: " << full_joint_name << "\n";
                       return joint;
                     };

    chassis_link = findLink("chassis");

    fl_wheel_joint_ = findJoint("front_left_wheel");
    fr_wheel_joint_ = findJoint("front_right_wheel");
    bl_wheel_joint_ = findJoint("back_left_wheel");
    br_wheel_joint_ = findJoint("back_right_wheel");
    fl_wheel_steering_joint_ = findJoint("front_left_wheel_steering");
    fr_wheel_steering_joint_ = findJoint("front_right_wheel_steering");
  }

  void SdControlPlugin::throttleCallback(const std_msgs::UInt8 & msg)
  {
    throttle_cmd_ = msg.data;
  }

  void SdControlPlugin::steerCallback(const std_msgs::UInt8 & msg)
  {
    steer_cmd_ = msg.data;
  }

GZ_REGISTER_MODEL_PLUGIN(SdControlPlugin)

}
