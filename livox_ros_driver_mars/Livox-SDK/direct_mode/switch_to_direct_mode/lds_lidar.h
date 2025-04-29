//
// The MIT License (MIT)
//
// Copyright (c) 2021 Livox. All rights reserved.
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

/** Livox LiDAR data source, data from dependent lidar */

#ifndef LDS_LIDAR_H_
#define LDS_LIDAR_H_

#include <memory>
#include <vector>
#include <string>

#include "livox_def.h"
#include "livox_sdk.h"


typedef enum {
  kConnectStateOff = 0,
  kConnectStateOn = 1,
  kConnectStateConfig = 2,
  kConnectStateSampling = 3,
} LidarConnectState;

typedef struct {
  uint8_t handle;
  LidarConnectState connect_state;
  DeviceInfo info;
} LidarDevice;

/**
 * LiDAR data source, data from dependent lidar.
 */
class LdsLidar {
 public:

  static LdsLidar& GetInstance() {
    static LdsLidar lds_lidar;
    return lds_lidar;
  }

  int InitLdsLidar(std::vector<std::string>& broadcast_code_strs);
  int DeInitLdsLidar(void);

 private:
  LdsLidar();
  LdsLidar(const LdsLidar&) = delete;
  ~LdsLidar();
  LdsLidar& operator=(const LdsLidar&) = delete;

  static void OnDeviceBroadcast(const BroadcastDeviceInfo *info);
  static void OnDeviceChange(const DeviceInfo *info, DeviceEvent type);

  static void ConfigLidarDirectMode(uint8_t handle);
  static void LidarSwitchDirectModeCb(livox_status status, uint8_t handle,
                                      DeviceParameterResponse *response,
                                      void *client_data);
  static void SetLidarUnicastModeIpCb(livox_status status, uint8_t handle,
                                      DeviceParameterResponse *response,
                                      void *client_data);

  volatile bool is_initialized_;

  uint32_t lidar_count_;
  LidarDevice lidars_[kMaxLidarCount];
};

#endif
