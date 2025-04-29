#ifndef TEAMMATE_HPP
#define TEAMMATE_HPP
#include <ros/ros.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include "fmt/color.h"
#include "udp_bridge/protocol.h"
#include "algorithm"
#include <string>
#include <fstream>

using namespace std;
using namespace udp_bridge;

class Teammate{
public:
    Teammate(const string &ip, const int &id, const double &rcv_time, const double &start_time){
        ip_ = ip;
        id_ = id;
        last_rcv_time_ = rcv_time;
        udp_send_fd_ptr_ = -1;
        offset_time_ = 0.0;
        offset_ts_.clear();
        delay_ts_.clear();
        sync_done_ = false;
        write_done_ = false;
        udp_start_time_ = start_time;
    }

    bool is_connect(double &cur_time){
    if(cur_time - last_rcv_time_ > 2.0)
        return false;
    else
        return true;
    }

    ~Teammate() = default;


    string ip_; //必须有ip_
    int id_;
    int udp_send_fd_ptr_;
    double offset_time_;
    vector<double> offset_ts_, delay_ts_;
    bool sync_done_, write_done_;
    double last_rcv_time_;
    sockaddr_in addr_udp_send_;
    double udp_start_time_;

private:

};
#endif