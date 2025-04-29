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
void OnSwitchToNoramlModeCallback(livox_status status, uint8_t slot, uint8_t response, void *client_data);

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

void OnSwitchToNoramlModeCallback(livox_status status, uint8_t slot, SwithToNormalModeResponse* response, void *client_data) {
  if (status != kStatusSuccess || response->ret_code != 0) {
    printf("On Switch To Noraml Mode Callback ret code: %d\n", response->ret_code);
    printf("On Switch To Noraml Mode Callback sn:  %s\n", response->broadcast_code);
    //LidarSwithToNormalMode(slot, OnSwitchToNoramlModeCallback, nullptr);
  } else {
    printf("On Switch To Noraml Mode Callback Success\n");
  }
}

void OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == NULL || info->dev_type == kDeviceTypeHub) {
    return;
  }
  printf("Receive Broadcast Code %s\n", info->broadcast_code);
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
  SetBroadcastCallback(OnDeviceBroadcast);

  uint8_t slot = 0;
  livox_status status = kStatusFailure;
  while(status != kStatusSuccess) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    status = LidarSwithToNormalMode(slot, OnSwitchToNoramlModeCallback, nullptr);
  }

  std::this_thread::sleep_for(std::chrono::seconds(100));

  printf("Livox lidar demo end!\n");

}
