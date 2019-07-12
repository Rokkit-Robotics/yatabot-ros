#ifndef OLOLO_
#define OLOLO_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <serial/serial.h>

class Yatabot : public hardware_interface::RobotHW {
 public:
   Yatabot(const std::string& port, double cmd_mul);

   void dump_cmd();

  private:
   hardware_interface::JointStateInterface jnt_state_interface;
   hardware_interface::VelocityJointInterface jnt_velocity_interface;

   double cmd[2];
   double pos[2];
   double vel[2];
   double eff[2];

   serial::Serial serial;
   double cmd_mul;
};

#endif  // OLOLO_
