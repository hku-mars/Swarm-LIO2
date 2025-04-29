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

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <thread>
#include <memory>

/** For callback use only */
LdsLidar* g_lidars = nullptr;

/** Lds lidar function ---------------------------------------------------------------------------*/
LdsLidar::LdsLidar() {
  is_initialized_    = false;

  lidar_count_       = 0;

  memset(lidars_, 0, sizeof(lidars_));
  for (uint32_t i=0; i<kMaxLidarCount; i++) {
    lidars_[i].handle = kMaxLidarCount;
    /** Unallocated state */
    lidars_[i].connect_state = kConnectStateOff;
  }
}

LdsLidar::~LdsLidar() {
}

int LdsLidar::InitLdsLidar(std::vector<std::string>& broadcast_code_strs) {

  if (is_initialized_) {
    printf("LiDAR data source is already inited!\n");
    return -1;
  }

  if (!Init()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

  SetBroadcastCallback(LdsLidar::OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(LdsLidar::OnDeviceChange);

  /** Start livox sdk to receive lidar data */
  if (!Start()) {
    Uninit();
    printf("Livox-SDK init fail!\n");
    return -1;
  }

  /** Add here, only for callback use */
  if (g_lidars == nullptr) {
    g_lidars = this;
  }
  is_initialized_= true;
  printf("Livox-SDK init success!\n");

  return 0;
}


int LdsLidar::DeInitLdsLidar(void) {

  if (!is_initialized_) {
    printf("LiDAR data source is not exit");
    return -1;
  }

  Uninit();
  printf("Livox SDK Deinit completely!\n");

  return 0;
}

/** Static function in LdsLidar for callback or event process ------------------------------------*/

void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr) {
    return;
  }

  if (info->dev_type == kDeviceTypeHub) {
    printf("In lidar mode, couldn't connect a hub : %s\n", info->broadcast_code);
    return;
  }

  livox_status result = kStatusFailure;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
    p_lidar->handle = handle;
    p_lidar->connect_state = kConnectStateOff;
  } else {
    printf("Add lidar to connect is failed : %d %d \n", result, handle);
  }
}

/** Callback function of changing of device state. */
void LdsLidar::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == nullptr) {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
  if (type == kEventConnect) {
    if (p_lidar->connect_state == kConnectStateOff) {
      p_lidar->connect_state = kConnectStateOn;
      p_lidar->info = *info;
    }
    printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
  } else if (type == kEventDisconnect) {
    p_lidar->connect_state = kConnectStateOff;
    printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
  } else if (type == kEventStateChange) {
    p_lidar->info = *info;
    printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
  }

  if (p_lidar->connect_state == kConnectStateOn) {
    printf("Device Working State %d\n", p_lidar->info.state);
    if (p_lidar->info.state == kLidarStateInit) {
      printf("Device State Change Progress %u\n", p_lidar->info.status.progress);
    } else {
      printf("Device State Error Code 0X%08x\n", p_lidar->info.status.status_code.error_code);
    }
    printf("Device feature %d\n", p_lidar->info.feature);

    /** Config lidar parameter */
    if (p_lidar->info.state == kLidarStateNormal) {
      ConfigLidarDirectMode(handle);
    }
  }
}

void LdsLidar::SetLidarUnicastModeIpCb(livox_status status, uint8_t handle,
                                      DeviceParameterResponse *response,
                                      void *client_data) {

  if (handle >= kMaxLidarCount) {
    return;
  }
  if (status != kStatusSuccess) {
    printf("Set Lidar Unicast Mode IP Failed\n");
    return;
  }

  LidarSwithDirectMode(handle,
                      ConnectedMode::kDirectModeUnicast,
                      LdsLidar::LidarSwitchDirectModeCb,
                      nullptr);

}

void LdsLidar::LidarSwitchDirectModeCb(livox_status status, uint8_t handle,
                                      DeviceParameterResponse *response,
                                      void *client_data) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  if (status != kStatusSuccess) {
    printf("Lidar Switch Direct Mode Failed\n");
  } else {
    printf("Lidar Switch Direct Mode Success\n");
  }
}

void LdsLidar::ConfigLidarDirectMode(uint8_t handle) {
  DeviceModeIpRequest req = {};
  uint8_t ip[4] = {192,168,1,55};
  memcpy(&req.ip_addr, ip, 4);
  req.point_port = 60000;
  req.imu_port = 60001;
  req.status_port = 60002;
  LidarSetUnicastModeIp(handle, &req, LdsLidar::SetLidarUnicastModeIpCb, (void *)g_lidars);
}


