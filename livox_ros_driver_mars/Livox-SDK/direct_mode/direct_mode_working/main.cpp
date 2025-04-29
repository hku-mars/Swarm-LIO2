//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include "livox_def.h"
#include "livox_sdk.h"
#include "cmdline.h"

/** Cmdline input broadcast code */
static std::vector<std::string> cmdline_broadcast_code;

/** Set the program options.
* You can input the registered device broadcast code and decide whether to save the log file.
*/
void SetProgramOption(int argc, const char *argv[]) {
  cmdline::parser cmd;
  cmd.add<std::string>("code", 'c', "Register device broadcast code", false);
  cmd.add("log", 'l', "Save the log file");
  cmd.add("help", 'h', "Show help");
  cmd.parse_check(argc, const_cast<char **>(argv));
  if (cmd.exist("code")) {
    std::string sn_list = cmd.get<std::string>("code");
    printf("Register broadcast code: %s\n", sn_list.c_str());
    size_t pos = 0;
    cmdline_broadcast_code.clear();
    while ((pos = sn_list.find("&")) != std::string::npos) {
      cmdline_broadcast_code.push_back(sn_list.substr(0, pos));
      sn_list.erase(0, pos + 1);
    }
    cmdline_broadcast_code.push_back(sn_list);
  }
  if (cmd.exist("log")) {
    printf("Save the log file.\n");
    SaveLoggerFile();
  }
  return;
}

int data_recveive_count = 0;
void GetLidarData(uint8_t slot, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
  if (data) {
    data_recveive_count ++ ;
    if (data_recveive_count % 100 == 0) {
      /** Parsing the timestamp and the point cloud data. */
      uint64_t cur_timestamp = *((uint64_t *)(data->timestamp));
      if(data ->data_type == kCartesian) {
        LivoxRawPoint *p_point_data = (LivoxRawPoint *)data->data;
      }else if ( data ->data_type == kSpherical) {
        LivoxSpherPoint *p_point_data = (LivoxSpherPoint *)data->data;
      }else if ( data ->data_type == kExtendCartesian) {
        LivoxExtendRawPoint *p_point_data = (LivoxExtendRawPoint *)data->data;
      }else if ( data ->data_type == kExtendSpherical) {
        LivoxExtendSpherPoint *p_point_data = (LivoxExtendSpherPoint *)data->data;
      }else if ( data ->data_type == kDualExtendCartesian) {
        LivoxDualExtendRawPoint *p_point_data = (LivoxDualExtendRawPoint *)data->data;
      }else if ( data ->data_type == kDualExtendSpherical) {
        LivoxDualExtendSpherPoint *p_point_data = (LivoxDualExtendSpherPoint *)data->data;
      }else if ( data ->data_type == kImu) {
        LivoxImuPoint *p_point_data = (LivoxImuPoint *)data->data;
      }else if ( data ->data_type == kTripleExtendCartesian) {
        LivoxTripleExtendRawPoint *p_point_data = (LivoxTripleExtendRawPoint *)data->data;
      }else if ( data ->data_type == kTripleExtendSpherical) {
        LivoxTripleExtendSpherPoint *p_point_data = (LivoxTripleExtendSpherPoint *)data->data;
      }
      printf("data_type %d packet num %d\n", data->data_type, data_recveive_count);
    }
  }
}

void GetImuData(uint8_t slot, LivoxEthPacket *data, uint32_t data_num, void *client_data) {
  if (data) {
      printf("data_type %d imu packet\n", data->data_type);
  }
}

void GetStatusData(uint8_t slot, LidarStatusInfo* status_info, void *client_data) {
  if (status_info) {
    if (status_info->state == kLidarStateInit) {
      printf("Init percentage: %d\n", status_info->status.progress);
    } else {
      printf("work state: %d\n", status_info->state);
      printf("temp_status : %u\n", status_info->status.status_code.lidar_error_code.temp_status);
      printf("volt_status : %u\n", status_info->status.status_code.lidar_error_code.volt_status);
      printf("motor_status : %u\n", status_info->status.status_code.lidar_error_code.motor_status);
      printf("dirty_warn : %u\n", status_info->status.status_code.lidar_error_code.dirty_warn);
      printf("firmware_err : %u\n", status_info->status.status_code.lidar_error_code.firmware_err);
      printf("pps_status : %u\n", status_info->status.status_code.lidar_error_code.device_status);
      printf("fan_status : %u\n", status_info->status.status_code.lidar_error_code.fan_status);
      printf("self_heating : %u\n", status_info->status.status_code.lidar_error_code.self_heating);
      printf("ptp_status : %u\n", status_info->status.status_code.lidar_error_code.ptp_status);
      printf("time_sync_status : %u\n", status_info->status.status_code.lidar_error_code.time_sync_status);
      printf("system_status : %u\n", status_info->status.status_code.lidar_error_code.system_status);
    }
  } else {
    printf("empty status info\n");
  }
}

int main(int argc, const char *argv[]) {
/** Set the program options. */
  SetProgramOption(argc, argv);

  if (!Init()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }
  /** Start livox sdk to receive lidar data */
  if (!Start()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  uint8_t slot = 0;
  SetPointDataCallback(slot, GetLidarData, nullptr);
  SetImuDataCallback(slot, GetImuData, nullptr);
  SetStatusDataCallback(slot, GetStatusData, nullptr);

  std::this_thread::sleep_for(std::chrono::seconds(100));

  printf("Livox lidar demo end!\n");

}
