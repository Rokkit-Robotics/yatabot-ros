#include "yatabot_base/yatabot_hardware.h"
#include <iostream>

Yatabot::Yatabot(const std::string& port, double in_cmd_mul)
  : serial(port, 115200)
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
}

void Yatabot::dump_cmd() {
  uint8_t data[7];

  data[0] = '@';
  data[1] = 4;
  data[2] = 0x01;  // chassis control

  int16_t left = static_cast<int16_t>(cmd[0] * cmd_mul);
  int16_t right = static_cast<int16_t>(cmd[0] * cmd_mul);

  data[3] = left & 0xFF;
  data[4] = (left >> 8) & 0xFF;
  data[5] = right & 0xFF;
  data[6] = (right >> 8) & 0xFF;

  if (serial.write(data, sizeof(data)) != sizeof(data)) {
    ROS_ERROR("failed to write cmd to serial");
  }
}
