#include "yatabot_base/yatabot_hardware.h"
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>

#include <functional>

void control_loop(Yatabot* yatabot, controller_manager::ControllerManager* cm) {
  ROS_INFO("hello again");
  yatabot->dump_cmd();
  ros::Duration elapsed(100);
  cm->update(ros::Time::now(), elapsed);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "yatabot_base");
  ros::NodeHandle nh, private_nh("~");

  std::string port;
  double cmd_mul;

  private_nh.param<std::string>("port", port, "unknown");
  private_nh.param<double>("cmd_mul", cmd_mul, 1.0);

  ROS_ERROR_STREAM("port: " << port << ", cmd_mul: " << cmd_mul);

  Yatabot yatabot(port, cmd_mul);

  ROS_ERROR("Welcome to Yatabot node");

  controller_manager::ControllerManager cm(&yatabot, nh);

  ros::CallbackQueue queue;
  ros::AsyncSpinner spinner(1, &queue);

  ros::TimerOptions ctrl_timer(
      ros::Duration(0.05),
      std::bind(control_loop, &yatabot, &cm),
      &queue);
  ros::Timer ctrl_loop = nh.createTimer(ctrl_timer);

  spinner.start();
  ros::spin();

  return 0;
}
