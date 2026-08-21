#ifndef __MARS_QUADROTOR_MSGS_MARS_QUADROTOR_MSGS_H__
#define __MARS_QUADROTOR_MSGS_MARS_QUADROTOR_MSGS_H__

#include <stdint.h>
#include <vector>
#include <mars_quadrotor_msgs/OutputData.h>
#include <mars_quadrotor_msgs/StatusData.h>
#include <mars_quadrotor_msgs/PPROutputData.h>

namespace mars_quadrotor_msgs
{

bool decodeOutputData(const std::vector<uint8_t> &data,
                      mars_quadrotor_msgs::OutputData &output);

bool decodeStatusData(const std::vector<uint8_t> &data,
                      mars_quadrotor_msgs::StatusData &status);

bool decodePPROutputData(const std::vector<uint8_t> &data,
                         mars_quadrotor_msgs::PPROutputData &output);
}

#endif
