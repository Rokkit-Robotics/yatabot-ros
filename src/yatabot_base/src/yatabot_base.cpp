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
  ros::NodeHandle nh;

  std::string port;
  double cmd_mul;

  nh.getParam("port", port);
  nh.getParam("cmd_mul", cmd_mul);

  Yatabot yatabot(port, cmd_mul);

  ROS_ERROR("Welcome to Yatabot node");

  controller_manager::ControllerManager cm(&yatabot, nh);

  ros::CallbackQueue queue;
  ros::AsyncSpinner spinner(1, &queue);

  ros::TimerOptions ctrl_timer(
      ros::Duration(0.1),
      std::bind(control_loop, &yatabot, &cm),
      &queue);
  ros::Timer ctrl_loop = nh.createTimer(ctrl_timer);

  spinner.start();
  ros::spin();

  return 0;
}
