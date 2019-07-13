#include "yatabot_base/yatabot_hardware.h"
#include <iostream>

Yatabot::Yatabot(const std::string& port, double in_cmd_mul)
  : serial(port, 115200, serial::Timeout::simpleTimeout(1000))
  , cmd_mul(in_cmd_mul) {
  hardware_interface::JointStateHandle state_handle_left("left_wheel_joint", &pos[0], &vel[0], &eff[0]);
  hardware_interface::JointStateHandle state_handle_right("right_wheel_joint", &pos[1], &vel[1], &eff[1]);

  jnt_state_interface.registerHandle(state_handle_left);
  jnt_state_interface.registerHandle(state_handle_right);

  registerInterface(&jnt_state_interface);

  hardware_interface::JointHandle vel_handle_left(jnt_state_interface.getHandle("left_wheel_joint"), &cmd[0]);
  jnt_velocity_interface.registerHandle(vel_handle_left);

  hardware_interface::JointHandle vel_handle_right(jnt_state_interface.getHandle("right_wheel_joint"), &cmd[1]);
  jnt_velocity_interface.registerHandle(vel_handle_right);

  registerInterface(&jnt_velocity_interface);

  // initialize hardware
  uint8_t data[4];
  data[0] = '@';
  data[1] = 2;
  data[2] = 0x01; // chassis
  data[3] = 0x01; // enable

  size_t ss = serial.write(data, sizeof(data));
  if (ss != sizeof(data)) {
    ROS_ERROR_STREAM("failed to write chassis enable cmd, written " << ss);
  } else {
    uint8_t reply;
    serial.read(&reply, 1);
    ROS_ERROR_STREAM("init answer " << int(reply));
  }
}

void Yatabot::dump_cmd() {
  uint8_t data[8];

  data[0] = '@';
  data[1] = 6;
  data[2] = 0x01;  // chassis control
  data[3] = 0x02;  // write

  int16_t left = static_cast<int16_t>(cmd[0] * cmd_mul);
  int16_t right = static_cast<int16_t>(cmd[1] * cmd_mul);

  data[4] = left & 0xFF;
  data[5] = (left >> 8) & 0xFF;
  data[6] = right & 0xFF;
  data[7] = (right >> 8) & 0xFF;

  size_t ss = serial.write(data, sizeof(data));
  if (ss != sizeof(data)) {
    ROS_ERROR_STREAM("failed to write chassis cmd, written " << ss);
  } else {
    uint8_t reply;
    serial.read(&reply, 1);
    ROS_ERROR_STREAM("cmd " << cmd[0] << ":" << cmd[1] << ", answer " << int(reply));
  }
}
