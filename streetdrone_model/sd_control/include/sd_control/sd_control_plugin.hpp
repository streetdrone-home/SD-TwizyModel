#ifndef SD_CONTROL__SD_CONTROL_PLUGIN_HPP
#define SD_CONTROL__SD_CONTROL_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>

namespace sd_control
{

  class SdControlPlugin : public gazebo::ModelPlugin
  {
  public:
    SdControlPlugin();

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  private:
    double fl_wheel_radius_;
    double fr_wheel_radius_;
    double bl_wheel_radius_;
    double br_wheel_radius_;

    std::string robot_namespace_;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
    gazebo::transport::NodePtr gznode_;

    gazebo::physics::LinkPtr chassis_link;

    gazebo::physics::JointPtr fl_wheel_joint_;
    gazebo::physics::JointPtr fr_wheel_joint_;
    gazebo::physics::JointPtr bl_wheel_joint_;
    gazebo::physics::JointPtr br_wheel_joint_;
    gazebo::physics::JointPtr fl_wheel_steering_joint_;
    gazebo::physics::JointPtr fr_wheel_steering_joint_;

    ros::Subscriber throttle_sub_;
    ros::Subscriber steer_sub_;

    double steer_cmd_;
    double throttle_cmd_;
    
    void throttleCallback(const std_msgs::UInt8 & msg);
    void steerCallback(const std_msgs::UInt8 & msg);

  };

}

#endif
