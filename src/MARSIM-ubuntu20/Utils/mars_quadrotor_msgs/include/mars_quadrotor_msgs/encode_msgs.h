#ifndef __MARS_QUADROTOR_MSGS_MARS_QUADROTOR_MSGS_H__
#define __MARS_QUADROTOR_MSGS_MARS_QUADROTOR_MSGS_H__

#include <stdint.h>
#include <vector>
#include <mars_quadrotor_msgs/SO3Command.h>
#include <mars_quadrotor_msgs/TRPYCommand.h>
#include <mars_quadrotor_msgs/Gains.h>

namespace mars_quadrotor_msgs
{

void encodeSO3Command(const mars_quadrotor_msgs::SO3Command &so3_command,
                      std::vector<uint8_t> &output);
void encodeTRPYCommand(const mars_quadrotor_msgs::TRPYCommand &trpy_command,
                       std::vector<uint8_t> &output);

void encodePPRGains(const mars_quadrotor_msgs::Gains &gains,
                    std::vector<uint8_t> &output);
}

#endif
