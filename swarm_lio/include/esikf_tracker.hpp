//
// Created by fangcheng on 2022/6/1.
//


#ifndef ESIKF_TRACKER_HPP
#define ESIKF_TRACKER_HPP

#include "common_lib.h"

#define init_cov 0.1;

class ESIKF{
public:
    ESIKF(const int &dim_x = 6, const int &dim_w = 3, const int &dim_z = 6){
        x_.resize(dim_x,1);
        x_.setZero();
        w_.resize(dim_w,1);
        w_.setZero();
        delta_x_.resize(dim_x,1);
        delta_x_.setZero();
        F_x_.resize(dim_x,dim_x);
        F_x_.setIdentity();
        F_w_.resize(dim_x,dim_w);
        F_w_.setZero();
        Q_.resize(dim_w,dim_w);
        Q_.setIdentity() * init_cov;
        R_.resize(dim_z,dim_z);
        R_.setIdentity() * init_cov;
        P_.resize(dim_x,dim_x);
        P_.setIdentity() * init_cov;
        K_.resize(dim_x,dim_z);
        K_.setZero();
        H_.resize(dim_z,dim_x);
        H_.setIdentity();
        z_.resize(dim_z,1);
        z_.setZero();
        iter_num_ = 1;
        last_predict_time_ = 0.0;
        last_update_time_ = 0.0;
        last_teammate_update_time_ = 0.0;
    }

    ~ESIKF(){};

    void init(const VectorXd &state, const double &lidar_end_time){
        x_ = state; //队友在自己世界系下的位置和速度
        w_ = Vector3d(0.01,0.01,0.01);//速度的噪声
        Q_ = w_.asDiagonal();
        F_x_ = Matrix<double,6,6>::Identity();
        F_x_.block<3,3>(0,3) = Matrix3d::Identity() * 0.001;
        F_w_.block<3,3>(0,0) = Matrix3d::Zero();
        F_w_.block<3,3>(3,0) = Matrix3d::Identity() * 0.001;
        H_ = Matrix<double,6,6>::Identity();
        last_predict_time_ = lidar_end_time - 0.001;
        last_update_time_ = lidar_end_time - 0.001;
        last_teammate_update_time_ = lidar_end_time - 0.001;
    }

    //重置滤波器，状态完全由观测决定，且协方差为0
    void reset( const VectorXd &meas, const double &lidar_end_time) {
        ROS_WARN("No update for too long time! EKF Tracker reset!");
        z_ = meas;
        x_ = z_;
        P_.setZero();
        last_predict_time_ = lidar_end_time;
    }

    void predict(const double &lidar_end_time){
        double dt = lidar_end_time - last_predict_time_;
        delta_x_.block<3,1>(0,0) = x_.block<3,1>(3,0) * dt;
        delta_x_.block<3,1>(3,0) = Zero3d;

        F_x_.block<3,3>(0,3) = Matrix3d::Identity() * dt;
        F_w_.block<3,3>(3,0) = Matrix3d::Identity() * dt;
        //State prediction
        x_ += delta_x_;
        //Covariance prediction
        P_ = F_x_ * P_ * F_x_.transpose() + F_w_ * Q_ * F_w_.transpose();
        last_predict_time_ = lidar_end_time;
    }

    void update(const VectorXd &meas, const int &iter_num, const double &lidar_end_time, const double &meas_noise) {
        z_ = meas;
        R_.setIdentity();
        R_ *= meas_noise;
        R_(2,2) = meas_noise * 100; //让Z轴尽量平滑，观测噪声大

        iter_num_ = iter_num;
        MatrixXd x_predicted = x_;

        for (int i = 0; i < iter_num; ++i) {
            MatrixXd residual = z_ - H_ * x_;
            //Kalman Gain
            K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
            //state update
            MatrixXd vec = x_ - x_predicted;
            delta_x_ = K_ * residual - vec + K_ * H_ * vec;
            x_ += delta_x_;

            //converge check
            if (delta_x_.norm() < 0.0001 || i == iter_num_ - 1){
                // Covariance update
                P_ = P_ - K_ * H_ * P_;
                break;
            }
        }
    }


    MatrixXd get_state_pos(){
        return x_.block<3,1>(0,0);
    }

    void set_H_vel(const Matrix3d &H_vel){
        H_.block<3,3>(3,3) = H_vel;
    }

    int iter_num_;
    double last_predict_time_, last_update_time_, last_teammate_update_time_;
private:
    MatrixXd x_, w_;
    MatrixXd delta_x_, F_x_, F_w_;
    MatrixXd Q_, R_;
    MatrixXd P_, K_, H_;
    MatrixXd z_;
};

#endif
