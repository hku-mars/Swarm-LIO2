//
// Created by fangcheng on 2022/3/31.
//
/*
Author: Fangcheng Zhu
email: zhufc@connect.hku.hk
*/
#include "MultiUAV.h"

template <class T>
string Multi_UAV::SetString(T &param_in) {
    stringstream ss;
    ss << setprecision(4) << param_in;
    string str = ss.str();
    return str;
}

Multi_UAV::Multi_UAV(const ros::NodeHandle &nh, const int & drone_id_) {
    nh_ = nh;
    drone_id = drone_id_;
    rot_world_to_gravity.setIdentity();
    //Load params
    LoadParam("preprocess/lidar_type", lidar_type, int(1));
    LoadParam("multiuav/cluster_extraction_in_predict_region", cluster_extraction_in_predict_region, bool(true));
    LoadParam("multiuav/inten_threshold", inten_threshold, int(100));
    LoadParam("multiuav/min_high_inten_cluster_size", min_high_inten_cluster_size, int(2));
    LoadParam("multiuav/min_cluster_size", min_cluster_size, int(50));
    LoadParam("multiuav/traj_matching_start_thresh", traj_matching_start_thresh, double(50));
    LoadParam("multiuav/ave_match_error_thresh", ave_match_error_thresh, double(0.5));
    LoadParam("multiuav/predict_region_radius", predict_region_radius, double(1.0));
    LoadParam("multiuav/temp_predict_region_radius", temp_predict_region_radius, double(1.0));
    LoadParam("multiuav/pos_num_in_traj", pos_num_in_traj, int(100));
    LoadParam("multiuav/reset_tracker_thresh", reset_tracker_thresh, double(5.0));
    LoadParam("multiuav/valid_cluster_size_thresh", valid_cluster_size_thresh, 0.6);
    LoadParam("multiuav/valid_cluster_dist_thresh", valid_cluster_dist_thresh, 0.35);
    LoadParam("multiuav/actual_uav_num", actual_uav_num, 4);

    string sub_quadstate_topic_name = "/quadstate_from_teammate";
    string sub_global_extrinsic_topic_name = "/global_extrinsic_from_teammate";


    same_obj_thresh = 0.3;
    valid_temp_cluster_dist_thresh = 0.4;
    topic_name_prefix = "";
    text_scale = 0.3;
    mesh_scale = 0.8;
    if (lidar_type == SIM) {
        //For Decentralized Simulation
        QuadState_subscriber_sim = nh_.subscribe(sub_quadstate_topic_name, 1000, &Multi_UAV::QuadstateCbk, this);
        GlobalExtrinsic_subscriber_sim = nh_.subscribe(sub_global_extrinsic_topic_name, 1000,
                                                       &Multi_UAV::GlobalExtrinsicCbk, this);
        same_obj_thresh = 0.5;
        valid_temp_cluster_dist_thresh = 0.8;
        sub_quadstate_topic_name = "/quadstate_to_teammate";
        sub_global_extrinsic_topic_name = "/global_extrinsic_to_teammate";
        topic_name_prefix = "quad" + SetString(drone_id) + "/";
        text_scale = 0.6;
        mesh_scale = 1.2;
    }

    QuadState_subscriber = nh_.subscribe(sub_quadstate_topic_name, 1000, &Multi_UAV::QuadstateCbk, this);
    QuadState_publisher = nh_.advertise<swarm_msgs::QuadStatePub>("/quadstate_to_teammate", 1000);
    GlobalExtrinsic_subscriber = nh_.subscribe(sub_global_extrinsic_topic_name, 1000,
                                               &Multi_UAV::GlobalExtrinsicCbk, this);
    GlobalExtrinsic_publisher = nh_.advertise<swarm_msgs::GlobalExtrinsicStatus>("/global_extrinsic_to_teammate",
                                                                                 1000);
    pubUAV = nh_.advertise<visualization_msgs::Marker>("/" + topic_name_prefix + "uav_visualization", 100);
    pubMeshUAV = nh_.advertise<visualization_msgs::Marker>("/" + topic_name_prefix + "uav_mesh_visualization", 100);
    pubCluster = nh_.advertise<visualization_msgs::Marker>("/" + topic_name_prefix + "cluster_visualization", 100);
    pubPredictRegionInput = nh_.advertise<sensor_msgs::PointCloud2>("/" + topic_name_prefix + "cluster_input",
                                                                    100);
    pubHighIntenInput = nh_.advertise<sensor_msgs::PointCloud2>("/" + topic_name_prefix + "high_intensity_input",
                                                                100);
    pubPredictRegion = nh_.advertise<visualization_msgs::Marker>("/" + topic_name_prefix + "predict_region", 100);
    pubTempTracker = nh_.advertise<visualization_msgs::Marker>("/" + topic_name_prefix + "temp_tracker", 100);
    pubTeammateList = nh_.advertise<swarm_msgs::ConnectedTeammateList>("/" + topic_name_prefix + "connected_teammate_list", 10);
    pubTeammateNum = nh_.advertise<std_msgs::Int8>("/" + topic_name_prefix + "connected_teammate_num", 10);
    pubTeammateIdTrajMatching = nh_.advertise<swarm_msgs::ConnectedTeammateList>("/" + topic_name_prefix + "teammate_id_with_traj_matching", 10);

    for (int i = 0; i < MAX_UAV_NUM; ++i) {
        pubTeammateOdom[i] = nh_.advertise<nav_msgs::Odometry>("/" + topic_name_prefix + "teammate_odom/UAV" +
                                                               SetString(i), 100);
        //Visualize Teammate Trajectory
        pubTeammateTraj[i] = nh_.advertise<visualization_msgs::MarkerArray>("/" + topic_name_prefix + "teammate_traj/UAV" +
                                                                            SetString(i), 100);
    }
    temp_tracker.clear();
    teammate_tracker.clear();
    reconnected_id.clear();
    extrinsic_infection.drone_id = drone_id;
    degenerated = false;
    teammate_id_by_traj_matching.clear();
}

Multi_UAV::~Multi_UAV(){
    teammate_tracker.clear();
    temp_tracker.clear();
    teammates.clear();
    cluster_pos_tag.clear();
    reconnected_id.clear();
}


void Multi_UAV::BuildMatrixWithUpperTriangular(const VD(12) &vec, M3D &rot_cov, M3D &pos_cov){
    rot_cov(0,0) = vec(0);
    rot_cov(0,1) = rot_cov(1,0) = vec(1);
    rot_cov(0,2) = rot_cov(2,0) = vec(2);
    rot_cov(1,1) = vec(3);
    rot_cov(1,2) = rot_cov(2,1) = vec(4);
    rot_cov(2,2) = vec(5);
    pos_cov(0,0) = vec(6);
    pos_cov(0,1) = pos_cov(1,0) = vec(7);
    pos_cov(0,2) = pos_cov(2,0) = vec(8);
    pos_cov(1,1) = vec(9);
    pos_cov(1,2) = pos_cov(2,1) = vec(10);
    pos_cov(2,2) = vec(11);
}

void Multi_UAV::QuadstateCbk(const swarm_msgs::QuadStatePub::ConstPtr &msg) {
    //判断队友飞机id
    int id = msg->drone_id;
    if (id == drone_id)
        return;

    auto iter = teammates.find(id);
    if (iter != teammates.end()) {
        auto &teammate_sub = iter->second;

        mtx_buffer_reconnectID.lock();
        //该飞机出了故障，重新开机加入队伍，swarmlio_start_time会变化
        if (abs(teammate_sub.teammate_state_temp.swarmlio_start_time - msg->swarmlio_start_time) > 0.1){
            reconnected_id.push_back(id);
        }
        mtx_buffer_reconnectID.unlock();


        teammate_sub.teammate_state_temp.sub_time = msg->header.stamp.toSec();
        teammate_sub.teammate_state_temp.rot =
        teammate_sub.teammate_state_temp.rot_end = Quaterniond(msg->pose.pose.orientation.w,
                                                               msg->pose.pose.orientation.x,
                                                               msg->pose.pose.orientation.y,
                                                               msg->pose.pose.orientation.z).matrix();
        teammate_sub.teammate_state_temp.pos =
        teammate_sub.teammate_state_temp.pos_end = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                                       msg->pose.pose.position.z);
        teammate_sub.teammate_state_temp.gyr = V3D(msg->gyr[0], msg->gyr[1], msg->gyr[2]);
        teammate_sub.teammate_state_temp.vel = V3D(msg->vel[0], msg->vel[1], msg->vel[2]);
        VD(12) pose_covariance;
        pose_covariance << msg->pose_cov[0], msg->pose_cov[1], msg->pose_cov[2],
                msg->pose_cov[3], msg->pose_cov[4], msg->pose_cov[5], msg->pose_cov[6], msg->pose_cov[7],
                msg->pose_cov[8], msg->pose_cov[9], msg->pose_cov[10], msg->pose_cov[11];
        BuildMatrixWithUpperTriangular(pose_covariance, teammate_sub.teammate_state_temp.rot_cov,
                                       teammate_sub.teammate_state_temp.pos_cov);


        teammate_sub.total_dist += (teammate_sub.teammate_state_temp.pos_end - teammate_sub.last_position).norm();
        teammate_sub.last_position = teammate_sub.teammate_state_temp.pos_end;
        teammate_sub.world_to_gravity_deg = V3D(msg->world_to_gravity_deg[0], msg->world_to_gravity_deg[1],
                                                msg->world_to_gravity_deg[2]);
        teammate_sub.teammate_state_temp.swarmlio_start_time = msg->swarmlio_start_time;
        teammate_sub.teammate_state_temp.degenerated = msg->degenerated;
        for (int i = 0; i < msg->teammate.size(); i++) {
            if (msg->teammate[i].teammate_id == drone_id) {
                //如果队友看到我了
                teammate_sub.teammate_state_temp.is_observed = true;
                V3D my_pos = V3D(msg->teammate[i].observed_pos[0], msg->teammate[i].observed_pos[1],
                                 msg->teammate[i].observed_pos[2]);
                teammate_sub.teammate_state_temp.my_pos_in_teammate = my_pos;
            }
        }
    } else {
        Teammate teammate_sub;
        teammate_sub.teammate_state_temp.sub_time = msg->header.stamp.toSec();
        teammate_sub.teammate_state_temp.rot =
        teammate_sub.teammate_state_temp.rot_end = Quaterniond(msg->pose.pose.orientation.w,
                                                               msg->pose.pose.orientation.x,
                                                               msg->pose.pose.orientation.y,
                                                               msg->pose.pose.orientation.z).matrix();
        teammate_sub.teammate_state_temp.pos =
        teammate_sub.teammate_state_temp.pos_end = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                                       msg->pose.pose.position.z);
        teammate_sub.teammate_state_temp.gyr = V3D(msg->gyr[0], msg->gyr[1], msg->gyr[2]);
        teammate_sub.teammate_state_temp.vel = V3D(msg->vel[0], msg->vel[1], msg->vel[2]);

        VD(12) pose_covariance;
        pose_covariance << msg->pose_cov[0], msg->pose_cov[1], msg->pose_cov[2],
                msg->pose_cov[3], msg->pose_cov[4], msg->pose_cov[5], msg->pose_cov[6], msg->pose_cov[7],
                msg->pose_cov[8], msg->pose_cov[9], msg->pose_cov[10], msg->pose_cov[11];
        BuildMatrixWithUpperTriangular(pose_covariance, teammate_sub.teammate_state_temp.rot_cov,
                                       teammate_sub.teammate_state_temp.pos_cov);

        teammate_sub.total_dist += (teammate_sub.teammate_state_temp.pos_end - teammate_sub.last_position).norm();
        teammate_sub.last_position = teammate_sub.teammate_state_temp.pos_end;
        teammate_sub.world_to_gravity_deg = V3D(msg->world_to_gravity_deg[0], msg->world_to_gravity_deg[1],
                                                msg->world_to_gravity_deg[2]);
        teammate_sub.teammate_state_temp.swarmlio_start_time = msg->swarmlio_start_time;
        teammate_sub.teammate_state_temp.degenerated = msg->degenerated;

        for (int i = 0; i < msg->teammate.size(); i++) {
            if (msg->teammate[i].teammate_id == drone_id) {
                //如果队友看到我了
                teammate_sub.teammate_state_temp.is_observed = true;
                V3D my_pos = V3D(msg->teammate[i].observed_pos[0], msg->teammate[i].observed_pos[1],
                                 msg->teammate[i].observed_pos[2]);
                teammate_sub.teammate_state_temp.my_pos_in_teammate = my_pos;
            }
        }
        teammates.insert(id2teammate(id, teammate_sub));
    }
}


void Multi_UAV::GlobalExtrinsicCbk(const swarm_msgs::GlobalExtrinsicStatus::ConstPtr &msg) {
    if (found_all_teammates || msg->drone_id == drone_id)
        return;

    //判断队友飞机id
    int id_j = msg->drone_id;

    //与队友建立通讯后等待两秒再开始接收世界系外参信息，这两秒内其他飞机会删除reconnect的队友的外参，即ResetReconnectedGlobalExtrinsic()函数
    auto iter = teammates.find(id_j);
    if(iter == teammates.end())
        return;
    else{
        double duration = msg->header.stamp.toSec() - iter->second.first_connect_time;
        if(duration < 2.0 || iter->second.first_connect_time < 0)
            return;
    }

    for (int k = 0; k < msg->extrinsic.size(); ++k) {
        int id_k = msg->extrinsic[k].teammate_id;
        V3D rot_jk_rad(msg->extrinsic[k].rot_deg[0], msg->extrinsic[k].rot_deg[1],
                       msg->extrinsic[k].rot_deg[2]);
        rot_jk_rad /= 57.3;
        M3D rot_jk = EulerToRotM(rot_jk_rad);
        V3D trans_jk = V3D(msg->extrinsic[k].trans[0], msg->extrinsic[k].trans[1], msg->extrinsic[k].trans[2]);

        mars::EdgeData edge;
        edge.from = id_j;
        edge.to = id_k;
        edge.rotation = rot_jk;
        edge.translation = trans_jk;
        extrinsic_infection.simple_graph.AddEdge(edge);
    }
}


void Multi_UAV::ResetReconnectedGlobalExtrinsic(StatesGroup &state_in, const double &lidar_end_time){
    mtx_buffer_reconnectID.lock();
    for (int i = 0; i < reconnected_id.size(); ++i) {
        int id = reconnected_id[i];
        auto tracker_ = teammate_tracker.find(id);
        if (tracker_ != teammate_tracker.end()) {
            teammate_tracker.erase(tracker_);
            found_all_teammates = false;
        }
        state_in.global_extrinsic_rot[id] = state.global_extrinsic_rot[id] = M3D::Identity();
        state_in.global_extrinsic_trans[id] = state.global_extrinsic_trans[id] = Zero3d;
        //Erase the edges of Reconneted UAV (of which the global extrinsic and start time changed)
        extrinsic_infection.simple_graph.RemoveNode(id);
    }
    reconnected_id.clear();
    mtx_buffer_reconnectID.unlock();
}

void Multi_UAV::UpdateFactorGraph(const bool &print_log) {
    if (found_all_teammates)
        return;

    TimeConsuming time_graph("Solve Graph");
    extrinsic_infection.SolveGraphIsam2(state);
    if (print_log)
        cout << " -- [Graph Solve Time] " << time_graph.stop() * 1000 << " ms"<< endl;
}

void Multi_UAV::UpdateGlobalExtrinsicAndCreateNewTeammateTracker(StatesGroup &state_in, const double &lidar_end_time) {
    double noise = 1e-6;
    for (auto iter = teammates.begin(); iter != teammates.end(); ++iter) {
        int id = iter->first;
        auto &teammate = iter->second;
        if (state_in.global_extrinsic_trans[id].norm() < 0.001 && state.global_extrinsic_trans[id].norm() > 0.001) {
            state_in.global_extrinsic_rot[id] = state.global_extrinsic_rot[id];
            state_in.global_extrinsic_trans[id] = state.global_extrinsic_trans[id];
            state_in.cov.block<6, 6>(18 + 6 * id, 18 + 6 * id) = noise * Matrix<double, 6, 6>::Identity();
            //Create New Tracker
            ESIKF drone_tracker(6, 3, 6);
            V3D teammate_pos_in_world =
                    state.global_extrinsic_rot[id] * teammate.teammate_state.pos_end +
                    state.global_extrinsic_trans[id];  //初始位置：cluster在本机世界系的位置
            V3D teammate_vel_in_world =
                    state.global_extrinsic_rot[id] * teammate.teammate_state.vel;; //初始速度：队友发来的速度转换到本机世界系
            Matrix<double, 6, 1> ekf_init_state;
            ekf_init_state.head(3) = teammate_pos_in_world;
            ekf_init_state.tail(3) = teammate_vel_in_world;
            drone_tracker.init(ekf_init_state, lidar_end_time);
            teammate_tracker.insert(id2ekf(id, drone_tracker));
            cout << BOLDBLUE << " -- [Infection Model] Find New Teammate, ID: " << id << RESET << endl;
        }
    }
}

void Multi_UAV::CopyTeammateState(Teammate &teammate) {
    teammate.teammate_state = teammate.teammate_state_temp;
}

//Reset each teammate's is_observed flag
void Multi_UAV::ResetTeammateState() {
    for (auto iter = teammates.begin(); iter != teammates.end(); ++iter) {
        iter->second.teammate_state_temp.is_observed = false;  //reset is_observed flag
        iter->second.teammate_state_temp.my_pos_in_teammate = Zero3d;
        iter->second.teammate_state_temp.degenerated = false;
        iter->second.is_observe_teammate = false;
        iter->second.teammate_pos_in_body = Zero3d;
    }
}

//可以从drone_id知道飞机编号
void Multi_UAV::PublishQuadstate(const V3D &unbiased_gyr, const double &lidar_end_time, const double &first_lidar_time) {
    quadstate_msg_pub.teammate.clear();
    quadstate_msg_pub.header.stamp = ros::Time().fromSec(lidar_end_time);
    quadstate_msg_pub.header.frame_id = topic_name_prefix + "world";
    if (lidar_type == SIM)
        quadstate_msg_pub.child_frame_id = "quad" + SetString(drone_id) + "_aft_mapped";
    else
        quadstate_msg_pub.child_frame_id = "aft_mapped";
    quadstate_msg_pub.drone_id = drone_id;
    SetPosestamp(quadstate_msg_pub.pose);
    V3D world_to_geavity_deg = RotMtoEuler(rot_world_to_gravity) * 57.3;
    for (int i = 0; i < 3; i++) {
        quadstate_msg_pub.vel[i] = state.vel_end(i);
        quadstate_msg_pub.gyr[i] = unbiased_gyr(i);
        quadstate_msg_pub.world_to_gravity_deg[i] = world_to_geavity_deg(i);
    }

    //Get Upper Triangular Matrix of pose covariance
    quadstate_msg_pub.pose_cov[0] = state.cov(0,0);
    quadstate_msg_pub.pose_cov[1] = state.cov(0,1);
    quadstate_msg_pub.pose_cov[2] = state.cov(0,2);
    quadstate_msg_pub.pose_cov[3] = state.cov(1,1);
    quadstate_msg_pub.pose_cov[4] = state.cov(1,2);
    quadstate_msg_pub.pose_cov[5] = state.cov(2,2);
    quadstate_msg_pub.pose_cov[6] = state.cov(3,3);
    quadstate_msg_pub.pose_cov[7] = state.cov(3,4);
    quadstate_msg_pub.pose_cov[8] = state.cov(3,5);
    quadstate_msg_pub.pose_cov[9] = state.cov(4,4);
    quadstate_msg_pub.pose_cov[10] = state.cov(4,5);
    quadstate_msg_pub.pose_cov[11] = state.cov(5,5);

    quadstate_msg_pub.swarmlio_start_time = first_lidar_time;
    quadstate_msg_pub.degenerated = degenerated;

    for (auto iter = teammates.begin(); iter != teammates.end(); ++iter) {
        //if observe teammate
        if (iter->second.is_observe_teammate) {
            swarm_msgs::ObserveTeammate obs_teammate;
            obs_teammate.is_observe = true;
            //drone id of observed teammate
            obs_teammate.teammate_id = iter->first;
            //observed teammate's position and timestamp
            obs_teammate.observed_pos[0] = iter->second.teammate_pos_in_body.x();
            obs_teammate.observed_pos[1] = iter->second.teammate_pos_in_body.y();
            obs_teammate.observed_pos[2] = iter->second.teammate_pos_in_body.z();
            quadstate_msg_pub.teammate.push_back(obs_teammate);
        }
    }
    QuadState_publisher.publish(quadstate_msg_pub);
}

void Multi_UAV::PublishGlobalExtrinsic(const double &lidar_end_time) {
    global_extrinsic_msg.extrinsic.clear();
    global_extrinsic_msg.header.stamp = ros::Time().fromSec(lidar_end_time);
    global_extrinsic_msg.header.frame_id = topic_name_prefix + "world";
    global_extrinsic_msg.drone_id = drone_id;
    V3D world_to_gravity_deg = RotMtoEuler(rot_world_to_gravity) * 57.3;
    global_extrinsic_msg.world_to_gravity_deg[0] = world_to_gravity_deg(0);
    global_extrinsic_msg.world_to_gravity_deg[1] = world_to_gravity_deg(1);
    global_extrinsic_msg.world_to_gravity_deg[2] = world_to_gravity_deg(2);

    for (auto iter = teammates.begin(); iter != teammates.end(); ++iter) {
        //Publish Effective Global Extrinsic with other drones
        swarm_msgs::GlobalExtrinsic global_extrinsic;
        global_extrinsic.teammate_id = iter->first;
        if (state.global_extrinsic_trans[iter->first].norm() < 0.001)
            continue;

        V3D global_rot_deg = RotMtoEuler(state.global_extrinsic_rot[iter->first]) * 57.3;
        for (int i = 0; i < 3; ++i) {
            global_extrinsic.rot_deg[i] = global_rot_deg(i);
            global_extrinsic.trans[i] = state.global_extrinsic_trans[iter->first](i);
            global_extrinsic.world_to_gravity_deg[i] = iter->second.world_to_gravity_deg(i);
        }
        global_extrinsic_msg.extrinsic.push_back(global_extrinsic);
    }
    GlobalExtrinsic_publisher.publish(global_extrinsic_msg);
}


template<class T>
bool Multi_UAV::LoadParam(string param_name, T &param_value, T default_value) {
    if (nh_.getParam(param_name, param_value)) {
        printf("\033[0;32m Load param %s success: \033[0;0m", param_name.c_str());
        cout << param_value << endl;
        return true;
    } else {
        printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
        param_value = default_value;
        cout << param_value << endl;
        return false;
    }
}

template<class T>
bool Multi_UAV::LoadParam(string param_name, vector<T> &param_value, vector<T> default_value) {
    if (nh_.getParam(param_name, param_value)) {
        printf("\033[0;32m Load param %s success: \033[0;0m", param_name.c_str());
        for (int i = 0; i < param_value.size(); i++) {
            cout << param_value[i] << " ";
        }
        cout << endl;
        return true;
    } else {
        printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
        param_value = default_value;
        for (int i = 0; i < param_value.size(); i++) {
            cout << param_value[i] << " ";
        }
        cout << endl;
        return false;
    }
}

template<typename T>
void Multi_UAV::SetPosestamp(T &out) {
    out.pose.position.x = state.pos_end(0);
    out.pose.position.y = state.pos_end(1);
    out.pose.position.z = state.pos_end(2);

    V3D euler_cur = RotMtoEuler(state.rot_end);
    geometry_msgs::Quaternion geoQuat;
    geoQuat = tf::createQuaternionMsgFromRollPitchYaw
            (euler_cur(0), euler_cur(1), euler_cur(2));
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}


//本机是否被队友(具体编号)观察到
bool Multi_UAV::IsObservedByTeammate(const Teammate &teammate) {
    return teammate.teammate_state.is_observed;
}

bool Multi_UAV::IsObserveTeammate(const Teammate &teammate) {
    return teammate.is_observe_teammate;
}

//队友发来的信息是否太陈旧，若是，则不满足匀速模型，无法使用互定位信息
bool Multi_UAV::IsDurationShort(const double &lidar_end_time, Teammate &teammate, const int &id) {
    double delta_t_quad = lidar_end_time - teammate.teammate_state.sub_time;
    if (abs(delta_t_quad) < 0.5) {
        teammate.last_connect_time = lidar_end_time;
        if(teammate.first_connect_time < 0)
            teammate.first_connect_time = lidar_end_time;
        return true;
    } else{
        cout << YELLOW << "[ WARN ] Message from drone " << id << " is out of time, delta_t = " << delta_t_quad << RESET << endl;
        return false;
    }
}

//Propagate teammate's pose in its own global frame at t_k.
void Multi_UAV::PropagateTeammateState(const double &lidar_end_time, Teammate &teammate) {
    double delta_t_quad = lidar_end_time - teammate.teammate_state.sub_time;
    teammate.teammate_state.pos_end =
            teammate.teammate_state.pos + teammate.teammate_state.vel * delta_t_quad;
    teammate.teammate_state.rot_end =
            teammate.teammate_state.rot * Exp(teammate.teammate_state.gyr, delta_t_quad);

    //Store teammates' odom and timestamp, used for trajectory matching
    if (teammate.teammate_odom_time.size() >= pos_num_in_traj)
        teammate.teammate_odom_time.pop_front();
    Vector4d pos_time;
    pos_time.block<3, 1>(0, 0) = teammate.teammate_state.pos_end;
    pos_time(3) = lidar_end_time;
    teammate.teammate_odom_time.push_back(pos_time);
}


Matrix<double, (3), (12)>
Multi_UAV::JacobianActiveObserve(V3D &active_observation_meas, const int &id, const Teammate &teammate) {
    Matrix<double, (3), (12)> Jacobian_case1;
    Jacobian_case1.setZero();
    M3D A_cross;
    V3D A = state.rot_end.transpose() *
            (state.global_extrinsic_rot[id] * teammate.teammate_state.pos_end +
             state.global_extrinsic_trans[id] -
             state.pos_end);
    A_cross << SKEW_SYM_MATRX(A);
    //Jacobian of Rotation
    Jacobian_case1.block<3, 3>(0, 0) = A_cross;
    //Jacobian of Translation
    Jacobian_case1.block<3, 3>(0, 3) = -state.rot_end.transpose();
    M3D B_cross;
    B_cross << SKEW_SYM_MATRX(teammate.teammate_state.pos_end);
    //Jacobian of Global Extrinsic Rotation
    Jacobian_case1.block<3, 3>(0, 6) =
            - state.rot_end.transpose() * state.global_extrinsic_rot[id] * B_cross;
    //Jacobian of Global Extrinsic Translation
    Jacobian_case1.block<3, 3>(0, 9) = state.rot_end.transpose();
    active_observation_meas = A - teammate.teammate_pos_in_body;
    return Jacobian_case1;
}


Matrix<double, (3), (12)>
Multi_UAV::JacobComputeCase2(V3D &passive_observation_meas, const int &id, const Teammate &teammate) {
    Matrix<double, (3), (12)> Jacobian_case2;
    Jacobian_case2.setZero();
    //Jacobian of Position
    M3D A = teammate.teammate_state.rot_end.transpose() * state.global_extrinsic_rot[id].transpose();
    Jacobian_case2.block<3, 3>(0, 3) = A;
    //Jacobian of Global Extrinsic Rotation
    M3D B_cross;
    V3D B = state.global_extrinsic_rot[id].transpose() *
            (state.pos_end - state.global_extrinsic_trans[id]);
    B_cross << SKEW_SYM_MATRX(B);
    Jacobian_case2.block<3, 3>(0, 6) = teammate.teammate_state.rot_end.transpose() * B_cross;
    //Jacobian of Global Extrinsic Translation
    Jacobian_case2.block<3, 3>(0, 9) = -A;
    V3D C = A * (state.pos_end - state.global_extrinsic_trans[id])
            - teammate.teammate_state.rot_end.transpose() * teammate.teammate_state.pos_end;
    //Residual at t_k
    passive_observation_meas = C - teammate.teammate_state.my_pos_in_teammate;
    return Jacobian_case2;
}


Matrix<double, (3), (12)>
Multi_UAV::JacobianPassiveObserve(V3D &passive_observation_meas, const int &id, const Teammate &teammate,
                                  const double &lidar_end_time) {
    Matrix<double, (3), (12)> Jacobian_case2;
    Jacobian_case2.setZero();
    double delta_ts_tk = teammate.teammate_state.sub_time - lidar_end_time;
    M3D A = teammate.teammate_state.rot.transpose() * state.global_extrinsic_rot[id].transpose();
    //Jacobian of Velocity
    Jacobian_case2.block<3, 3>(0, 0) = A * delta_ts_tk;
    //Jacobian of Position
    Jacobian_case2.block<3, 3>(0, 3) = A;
    //Jacobian of Global Extrinsic Rotation
    M3D B_cross;
    V3D B = state.global_extrinsic_rot[id].transpose() *
            (state.pos_end + state.vel_end * delta_ts_tk - state.global_extrinsic_trans[id]);
    B_cross << SKEW_SYM_MATRX(B);
    Jacobian_case2.block<3, 3>(0, 6) = teammate.teammate_state.rot.transpose() * B_cross;
    //Jacobian of Global Extrinsic Translation
    Jacobian_case2.block<3, 3>(0, 9) = -A;
    V3D C = A * (state.pos_end + state.vel_end * delta_ts_tk - state.global_extrinsic_trans[id])
            - teammate.teammate_state.rot.transpose() * teammate.teammate_state.pos;
    //Residual at t_s
    passive_observation_meas = C - teammate.teammate_state.my_pos_in_teammate;
    return Jacobian_case2;
}

M3D Multi_UAV::GetActiveMutualObserveMeasurementNoise(const bool &degenerated, const StatesGroup &state_, const int &id, const Teammate &teammate, const double &mutual_observe_noise){
    M3D ActiveObserveNoise; ActiveObserveNoise.setZero();
    if(degenerated){
        MD(3,12) Jacobian_noise;Jacobian_noise.setZero();
        //Noise of Global Extrinsic Rotation
        M3D teammate_pos_cross;
        teammate_pos_cross << SKEW_SYM_MATRX(teammate.teammate_state.pos_end);
        Jacobian_noise.block<3,3>(0,0) = - state_.rot_end.transpose()
                                         * state_.global_extrinsic_rot[id].transpose() * teammate_pos_cross;
        //Noise of Teammate's Position
        Jacobian_noise.block<3,3>(0,3) = state_.rot_end.transpose()
                                         * state_.global_extrinsic_rot[id].transpose();
        //Noise of Global Extrinsic Translation
        Jacobian_noise.block<3,3>(0,6) = state_.rot_end.transpose();
        //Noise of mutual observe measurement in body frame
        Jacobian_noise.block<3,3>(0,9) = - M3D::Identity();

        ActiveObserveNoise = Jacobian_noise.block<3,3>(0,0) * state_.cov.block<3,3>(18 + id * 6,18 + id * 6) * Jacobian_noise.block<3,3>(0,0).transpose()
                             + Jacobian_noise.block<3,3>(0,3) * teammate.teammate_state.pos_cov * Jacobian_noise.block<3,3>(0,3).transpose()
                             + Jacobian_noise.block<3,3>(0,6) * state_.cov.block<3,3>(18 + id * 6 + 3,18 + id * 6 + 3) * Jacobian_noise.block<3,3>(0,6).transpose()
                             + Jacobian_noise.block<3,3>(0,9) * (M3D::Identity() * mutual_observe_noise) * Jacobian_noise.block<3,3>(0,9).transpose();
        return ActiveObserveNoise;
    }else{
        MD(3,6) Jacobian_noise;
        Jacobian_noise.setZero();
        //Noise of Teammate's Position
        Jacobian_noise.block<3,3>(0,0) = state_.rot_end.transpose()
                                         * state_.global_extrinsic_rot[id].transpose();
        //Noise of mutual observe measurement in body frame
        Jacobian_noise.block<3,3>(0,3) = - M3D::Identity();
        ActiveObserveNoise = Jacobian_noise.block<3,3>(0,0) * teammate.teammate_state.pos_cov * Jacobian_noise.block<3,3>(0,0).transpose()
                             + Jacobian_noise.block<3,3>(0,3) * (M3D::Identity() * mutual_observe_noise) * Jacobian_noise.block<3,3>(0,3).transpose();
        return ActiveObserveNoise;
    }
}

M3D Multi_UAV::GetPassiveMutualObserveMeasurementNoise(const bool &degenerated, const StatesGroup &state_, const int &id, const Teammate &teammate, const double & lidar_end_time, const double &mutual_observe_noise){
    M3D PassiveObserveNoise; PassiveObserveNoise.setZero();
    if(degenerated){
        MD(3,15) Jacobian_noise;Jacobian_noise.setZero();
        //Noise of Teammate's Rotation
        double delta_ts_tk = teammate.teammate_state.sub_time - lidar_end_time;
        V3D temp0 = state.global_extrinsic_rot[id].transpose()
                    * (state.pos_end + state.vel_end * delta_ts_tk - state.global_extrinsic_trans[id]);
        V3D temp1 = teammate.teammate_state.rot.transpose() * (temp0 - teammate.teammate_state.pos)
                    - teammate.teammate_state.my_pos_in_teammate;
        M3D temp1_cross; temp1_cross << SKEW_SYM_MATRX(temp1);
        Jacobian_noise.block<3,3>(0,0) = temp1_cross;
        //Noise of Global Extrinsic Rotation
        M3D temp0_cross; temp0_cross << SKEW_SYM_MATRX(temp0);
        Jacobian_noise.block<3,3>(0,3) = teammate.teammate_state.rot.transpose() * temp0_cross;
        //Noise of Global Extrinsic Translation
        Jacobian_noise.block<3,3>(0,6) = teammate.teammate_state.rot.transpose() * state.global_extrinsic_rot[id].transpose();
        //Noise of Teammate's Position
        Jacobian_noise.block<3,3>(0,9) = teammate.teammate_state.rot.transpose();
        //Noise of mutual observe measurement in body frame
        Jacobian_noise.block<3,3>(0,12) = - M3D::Identity();

        PassiveObserveNoise = Jacobian_noise.block<3,3>(0,0) * teammate.teammate_state.rot_cov * Jacobian_noise.block<3,3>(0,0).transpose()
                              + Jacobian_noise.block<3,3>(0,3) * state_.cov.block<3,3>(18 + id * 6,18 + id * 6) * Jacobian_noise.block<3,3>(0,3).transpose()
                              + Jacobian_noise.block<3,3>(0,6) * state_.cov.block<3,3>(18 + id * 6 + 3,18 + id * 6 + 3) * Jacobian_noise.block<3,3>(0,6).transpose()
                              + Jacobian_noise.block<3,3>(0,9) * teammate.teammate_state.pos_cov * Jacobian_noise.block<3,3>(0,9).transpose()
                              + Jacobian_noise.block<3,3>(0,12) * (M3D::Identity() * mutual_observe_noise) * Jacobian_noise.block<3,3>(0,12).transpose();
        return PassiveObserveNoise;
    }else{
        MD(3,9) Jacobian_noise;Jacobian_noise.setZero();
        //Noise of Teammate's Rotation
        double delta_ts_tk = teammate.teammate_state.sub_time - lidar_end_time;
        V3D temp0 = state.global_extrinsic_rot[id].transpose()
                    * (state.pos_end + state.vel_end * delta_ts_tk - state.global_extrinsic_trans[id]);
        V3D temp1 = teammate.teammate_state.rot.transpose() * (temp0 - teammate.teammate_state.pos)
                    - teammate.teammate_state.my_pos_in_teammate;
        M3D temp1_cross; temp1_cross << SKEW_SYM_MATRX(temp1);
        Jacobian_noise.block<3,3>(0,0) = temp1_cross;

        //Noise of Teammate's Position
        Jacobian_noise.block<3,3>(0,3) = teammate.teammate_state.rot.transpose();
        //Noise of mutual observe measurement in body frame
        Jacobian_noise.block<3,3>(0,6) = - M3D::Identity();
        PassiveObserveNoise = Jacobian_noise.block<3,3>(0,0) * teammate.teammate_state.rot_cov * Jacobian_noise.block<3,3>(0,0).transpose()
                              + Jacobian_noise.block<3,3>(0,3) * teammate.teammate_state.pos_cov * Jacobian_noise.block<3,3>(0,3).transpose()
                              + Jacobian_noise.block<3,3>(0,6) * (M3D::Identity() * mutual_observe_noise) * Jacobian_noise.block<3,3>(0,6).transpose();
        return PassiveObserveNoise;
    }
}

//Euclidean Cluster Extraction
void Multi_UAV::ClusterExtractPredictRegion(const double &lidar_end_time, const PointCloudXYZI::Ptr cur_pcl_undistort) {
    cluster_pos_tag.clear();
    if (teammate_tracker.empty() && temp_tracker.empty()) {
        //Delete All Cluster Visualization
        VisualizeDeleteAllCluster(lidar_end_time);
        cluster_id = 0;
        // cout << YELLOW << " -- [Warn] No Predict Region! No Tracker Yet!" << RESET << endl;
        return;
    }

    if(!cluster_extraction_in_predict_region)
        return;


    PointCloudXYZI::Ptr cur_pcl_undistort_remove_far_points(new PointCloudXYZI(cur_pcl_undistort->size(), 1));
    cur_pcl_undistort_remove_far_points->clear(); cur_pcl_undistort_remove_far_points->reserve(cur_pcl_undistort->size());


    //Remove Far Points
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < cur_pcl_undistort->size(); i++) {
        auto it_pcl = cur_pcl_undistort->points.begin() + i;
        V3D pt(it_pcl->x, it_pcl->y, it_pcl->z);
        if(pt.norm() < 20)
            cur_pcl_undistort_remove_far_points->push_back(*it_pcl);
    }


    //Nearest Search
    pcl::search::KdTree<pcl::PointXYZINormal>::Ptr kdtree_all(new pcl::search::KdTree<pcl::PointXYZINormal>);
    //KD-TREE建树耗时较大
    kdtree_all->setInputCloud(cur_pcl_undistort_remove_far_points);


    //Publish Predict Region of Temporary Tracker
    VisualizeTemporaryPredictRegion(lidar_end_time);

    //Point Cloud in Predict Region
    pcl::PointCloud<pcl::PointXYZ> teammate_predict_region_cloud, temp_predict_region_cloud, all_predict_region_cloud;
    teammate_predict_region_cloud.clear();teammate_predict_region_cloud.reserve(cur_pcl_undistort_remove_far_points->size());
    temp_predict_region_cloud.clear();temp_predict_region_cloud.reserve(cur_pcl_undistort_remove_far_points->size());
    all_predict_region_cloud.clear();all_predict_region_cloud.reserve(cur_pcl_undistort_remove_far_points->size());


    for (int i = 0; i < temp_tracker.size(); ++i) {
        V3D predict_pos_in_body =
                state.rot_end.transpose() * (temp_tracker[i].dyn_tracker.get_state_pos() - state.pos_end);
        PointType searchPoint;
        searchPoint.x = predict_pos_in_body(0);
        searchPoint.y = predict_pos_in_body(1);
        searchPoint.z = predict_pos_in_body(2);
        //基于半径R内的最近邻搜索
        vector<int> pointIdxRadiusSearch;//下标
        vector<float> pointRadiusSquaredDistance;//距离
        if(kdtree_all->radiusSearch(searchPoint, temp_predict_region_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, 150) > 0){
            for (int j = 0; j < pointIdxRadiusSearch.size(); ++j){
                pcl::PointXYZ pt_add;
                pt_add.x = cur_pcl_undistort_remove_far_points->points[pointIdxRadiusSearch[j]].x;
                pt_add.y = cur_pcl_undistort_remove_far_points->points[pointIdxRadiusSearch[j]].y;
                pt_add.z = cur_pcl_undistort_remove_far_points->points[pointIdxRadiusSearch[j]].z;
                temp_predict_region_cloud.push_back(pt_add);
            }
        }
    }
    for (auto iter = teammate_tracker.begin(); iter != teammate_tracker.end(); ++iter) {
        //V3D predict_pos_in_body = state.rot_end.transpose() * (iter->second.get_state_pos() - state.pos_end);
        int id = iter->first;
        auto teammate_iter = teammates.find(id);
        V3D predict_pos_in_body = state.rot_end.transpose() * (state.global_extrinsic_rot[id] * teammate_iter->second.teammate_state.pos_end + state.global_extrinsic_trans[id] - state.pos_end); //预测的队友位置

        PointType searchPoint;
        searchPoint.x = predict_pos_in_body(0);
        searchPoint.y = predict_pos_in_body(1);
        searchPoint.z = predict_pos_in_body(2);
        //基于半径R内的最近邻搜索
        vector<int> pointIdxRadiusSearch;//下标
        vector<float> pointRadiusSquaredDistance;//距离
        if(kdtree_all->radiusSearch(searchPoint, predict_region_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance, 60) > 0){
            for (int j = 0; j < pointIdxRadiusSearch.size(); ++j){
                pcl::PointXYZ pt_add;
                pt_add.x = cur_pcl_undistort_remove_far_points->points[pointIdxRadiusSearch[j]].x;
                pt_add.y = cur_pcl_undistort_remove_far_points->points[pointIdxRadiusSearch[j]].y;
                pt_add.z = cur_pcl_undistort_remove_far_points->points[pointIdxRadiusSearch[j]].z;
                teammate_predict_region_cloud.push_back(pt_add);
            }
        }
    }


    all_predict_region_cloud = teammate_predict_region_cloud + temp_predict_region_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(
            new pcl::PointCloud<pcl::PointXYZ>(all_predict_region_cloud.size(), 1));
    *cluster_cloud = all_predict_region_cloud;

    if (pubPredictRegionInput.getNumSubscribers() > 0) {
        //publish cluster input in world frame
        sensor_msgs::PointCloud2 cluster_input_msg;
        Matrix4d body_to_gravity = Matrix4d::Identity();
        body_to_gravity.block<3, 3>(0, 0) = rot_world_to_gravity * state.rot_end;
        body_to_gravity.block<3, 1>(0, 3) = rot_world_to_gravity * state.pos_end;
        pcl::PointCloud<pcl::PointXYZ>::Ptr all_predict_region_cloud_world(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(all_predict_region_cloud, *all_predict_region_cloud_world, body_to_gravity);
        pcl::toROSMsg(*all_predict_region_cloud_world, cluster_input_msg);
        cluster_input_msg.header.stamp = ros::Time().fromSec(lidar_end_time);
        cluster_input_msg.header.frame_id = topic_name_prefix + "world";
        pubPredictRegionInput.publish(cluster_input_msg);
    }

    if (all_predict_region_cloud.empty()) {
        // cout << YELLOW << " -- [Warn] No Object in Predict Region!" << RESET << endl;
        return;
    }

    //FEC: Fast Euclidean Clustering for Point Cloud Segmentation
    vector<pcl::PointIndices> clusters;
    clusters.clear();
    clusters = FEC::FastEuclideanClustering(cluster_cloud, min_high_inten_cluster_size, 0.3, 200);


    //Delete All Cluster Visualization
    VisualizeDeleteAllCluster(lidar_end_time);
    // Reset cluster ID
    cluster_id = 0;
    for (auto iter = clusters.begin(); iter != clusters.end(); iter++) {
        //For each point in current cluster...
        int cnt = 0;
        V3D cluster_pos = Zero3d;
        V3D max_value = V3D(INT_MIN, INT_MIN, INT_MIN), min_value = V3D(INT_MAX, INT_MAX, INT_MAX);
        for (auto index = iter->indices.begin(); index != iter->indices.end(); index++) {
            cnt++;
            V3D point_cur = V3D(cluster_cloud->points[*index].x, cluster_cloud->points[*index].y,
                                cluster_cloud->points[*index].z);
            //Position of the cluster center
            cluster_pos += (point_cur - cluster_pos) / cnt;
            //Compute max and min xyz value
            for (int i = 0; i < 3; ++i) {
                max_value(i) = max(max_value(i), point_cur(i));
                min_value(i) = min(min_value(i), point_cur(i));
            }
        }
        double max_dist = (max_value - min_value).norm();
        if(max_dist > valid_cluster_size_thresh)
            continue;
        cluster_pos_tag.push_back(Cluster(cluster_pos, false, max_dist));
        VisualizeCluster(lidar_end_time, cluster_pos, cluster_id, V3D(max_value - min_value));
        cluster_id++;
    }
}


void Multi_UAV::ClusterExtractHighIntensity(const double &lidar_end_time, const PointCloudXYZI::Ptr cur_pcl_undistort) {
    pcl::PointCloud<pcl::PointXYZ> cluster_input_cloud;
    cluster_input_cloud.clear();
    cluster_input_cloud.reserve(cur_pcl_undistort->size());

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < cur_pcl_undistort->size(); i++) {
        if (cur_pcl_undistort->points[i].intensity > inten_threshold) {
            pcl::PointXYZ pt_add;
            pt_add.x = cur_pcl_undistort->points[i].x;
            pt_add.y = cur_pcl_undistort->points[i].y;
            pt_add.z = cur_pcl_undistort->points[i].z;
            V3D dist(pt_add.x, pt_add.y, pt_add.z);

            //距离小于20m的高反点，才用于聚类
            if(lidar_type != SIM && dist.norm() > 20)
                continue;

            cluster_input_cloud.push_back(pt_add);
        }
    }


    if (cluster_input_cloud.empty()) {
        // cout << YELLOW << " -- [Warn] No Object with High Intensity!" << RESET << endl;
        return;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree <pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(
            new pcl::PointCloud<pcl::PointXYZ>(cluster_input_cloud.size(), 1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_world(
            new pcl::PointCloud<pcl::PointXYZ>(cluster_input_cloud.size(), 1));
    *cluster_cloud = cluster_input_cloud;


    if (pubHighIntenInput.getNumSubscribers() > 0) {
        //publish cluster input in world frame
        sensor_msgs::PointCloud2 cluster_input_msg;
        Matrix4d body_to_gravity = Matrix4d::Identity();
        body_to_gravity.block<3, 3>(0, 0) = rot_world_to_gravity * state.rot_end;
        body_to_gravity.block<3, 1>(0, 3) = rot_world_to_gravity * state.pos_end;
        pcl::transformPointCloud(*cluster_cloud, *cluster_cloud_world, body_to_gravity);
        pcl::toROSMsg(*cluster_cloud_world, cluster_input_msg);
        cluster_input_msg.header.stamp = ros::Time().fromSec(lidar_end_time);
        cluster_input_msg.header.frame_id = topic_name_prefix + "world";
        pubHighIntenInput.publish(cluster_input_msg);
    }


    //FEC: Fast Euclidean Clustering for Point Cloud Segmentation
    vector<pcl::PointIndices> clusters;
    clusters.clear();
    clusters = FEC::FastEuclideanClustering(cluster_cloud, min_high_inten_cluster_size, 0.3, 1000);
    // For every cluster...
    for (auto iter = clusters.begin(); iter != clusters.end(); iter++) {
        //For each point in current cluster...
        int cnt = 0;
        V3D cluster_pos = Zero3d;
        V3D max_value = V3D(INT_MIN, INT_MIN, INT_MIN), min_value = V3D(INT_MAX, INT_MAX, INT_MAX);
        for (auto index = iter->indices.begin(); index != iter->indices.end(); index++) {
            cnt++;
            V3D point_cur = V3D(cluster_cloud->points[*index].x, cluster_cloud->points[*index].y,
                                cluster_cloud->points[*index].z);
            //Position of the cluster center
            cluster_pos += (point_cur - cluster_pos) / cnt;
            //Compute max and min xyz value
            for (int i = 0; i < 3; ++i) {
                max_value(i) = max(max_value(i), point_cur(i));
                min_value(i) = min(min_value(i), point_cur(i));
            }
        }
        double max_dist = (max_value - min_value).norm();

        bool same_object = false;
        //如果从teammate tracker的predicted region中分割出的object 和 highly-reflective object足够近，则认为是同一个物体，物体的位置更新为高反点的位置
        for (int i = 0; i < cluster_pos_tag.size(); ++i) {
            V3D dist_vec = cluster_pos_tag[i].pos_in_body - cluster_pos;
            if (dist_vec.norm() < same_obj_thresh) {
                cluster_pos_tag[i].pos_in_body = cluster_pos;
                cluster_pos_tag[i].is_high_intensity = false; //如果该高反物体和某个tracker是同个物体，则不会被用于创建新的temp tracker，可将其属性设置为“非高反”
                cluster_pos_tag[i].max_dist = max_dist;
                same_object = true;
                break;
            }
        }

        if (!same_object) {
            cluster_pos_tag.push_back(Cluster(cluster_pos, true, max_dist));
            VisualizeCluster(lidar_end_time, cluster_pos, cluster_id, V3D(max_value - min_value));
            cluster_id++;
        }
    }
}



void Multi_UAV::CheckClusterValidation(const int &id, Teammate &teammate) {
    //Reset observe status
    teammate.is_observe_teammate = false;
    teammate.teammate_pos_in_body = Zero3d;
    auto iter = teammate_tracker.find(id);
    if (iter == teammate_tracker.end())
        return;
    else {
        auto &tracker = iter->second;
        //V3D pos_predict = state.rot_end.transpose() * (tracker.get_state_pos() - state.pos_end); //EKF的预测
        auto teammate_iter = teammates.find(id);
        V3D pos_predict = state.rot_end.transpose() * (state.global_extrinsic_rot[id] * teammate_iter->second.teammate_state.pos_end + state.global_extrinsic_trans[id] - state.pos_end); //EKF的预测

        double min_dist = DBL_MAX;  //找到最近的一个cluster
        for (int j = 0; j < cluster_pos_tag.size(); j++) {
            V3D resi_vec = cluster_pos_tag[j].pos_in_body - pos_predict;
            if (resi_vec.norm() < valid_cluster_dist_thresh && resi_vec.norm() < min_dist) {
                min_dist = resi_vec.norm();
                teammate.is_observe_teammate = true;
                teammate.teammate_pos_in_body = cluster_pos_tag[j].pos_in_body;
                cluster_pos_tag.erase(cluster_pos_tag.begin() + j); //删除匹配成功的cluster position，减少后续计算
                j--;
            }
        }
    }
}


void Multi_UAV::PredictTracker(const double &lidar_end_time, const int &id) {
    auto iter = teammate_tracker.find(id);
    if (iter == teammate_tracker.end())
        return;
    else {
        double connect_interval = lidar_end_time - iter->second.last_teammate_update_time_;
        if (connect_interval < 2.0) //失联少于两秒，才propagation
            iter->second.predict(lidar_end_time);
    }
}

void Multi_UAV::UpdateTracker(const double &lidar_end_time, const int &id, const Teammate &teammate,
                              const bool &cluster_meas, const bool &print_log) {
    auto iter = teammate_tracker.find(id);
    if (iter == teammate_tracker.end())
        return;

    auto &tracker = iter->second;
    //Update EKF tracker
    Matrix<double, 6, 1> ekf_meas;

    if (cluster_meas) {
        if (teammate.teammate_pos_in_body.norm() < 1e-3)
            return;
        ekf_meas.head(3) = state.rot_end * teammate.teammate_pos_in_body + state.pos_end;
        ekf_meas.tail(3) = Zero3d;
        double cluster_update_dt = lidar_end_time - tracker.last_update_time_;
        if (cluster_update_dt > reset_tracker_thresh) {
            //Reset EKF Tracker
            tracker.reset(ekf_meas, lidar_end_time);
            tracker.last_update_time_ = lidar_end_time;
            return;
        }
        tracker.set_H_vel(Matrix3d::Zero());
        tracker.update(ekf_meas, 1, lidar_end_time, 0.01);
        tracker.last_update_time_ = lidar_end_time;
    } else {
        //失联少于两秒，才用队友odom值来update
        double connect_interval = lidar_end_time - teammate.last_connect_time;
        if(connect_interval > 2.0)
            return;

        ekf_meas.head(3) =
                state.global_extrinsic_rot[id] * teammate.teammate_state.pos_end + state.global_extrinsic_trans[id];
        ekf_meas.tail(3) = state.global_extrinsic_rot[id] * teammate.teammate_state.vel;

        double teammate_update_dt = lidar_end_time - tracker.last_teammate_update_time_;

        if (teammate_update_dt > reset_tracker_thresh) {
            //Reset EKF Tracker
            tracker.reset(ekf_meas, lidar_end_time);
            tracker.last_teammate_update_time_ = lidar_end_time;
            return;
        }

        tracker.set_H_vel(Matrix3d::Identity());
        tracker.update(ekf_meas, 1, lidar_end_time, 0.001);
//        if (print_log)
//            cout << " -- [Teammate Tracker " << id << " Update] " << GREEN << "Teammate LIO." << RESET << endl;
        tracker.last_teammate_update_time_ = lidar_end_time;
    }
}


bool Multi_UAV::DeleteInvalidTemporaryTracker(const double &lidar_end_time, const int &index) {
    double update_dt = lidar_end_time - temp_tracker[index].dyn_tracker.last_update_time_;
    double exist_dt = lidar_end_time - temp_tracker[index].create_time;
    if (update_dt > 2.0) {   //2s没有更新，删除
        cout << YELLOW << " -- [Delete Temporary Tracker " << index << "] This object is NOT teammate!" << RESET << endl;
        VisualizeTempTrackerDelete(lidar_end_time, index);
        temp_tracker.erase(temp_tracker.begin() + index);
        return true;
    }

    for (auto iter = teammate_tracker.begin(); iter != teammate_tracker.end(); ++iter) {
        double dist = (temp_tracker[index].dyn_tracker.get_state_pos() - iter->second.get_state_pos()).norm();
        if (dist < same_obj_thresh) {
            VisualizeTempTrackerDelete(lidar_end_time, index);
            temp_tracker.erase(temp_tracker.begin() + index);
            return true;
        }
    }
    return false;
}


bool Multi_UAV::TrajMatching(vector<Vector3d> &dyn_positions,
                             vector<Vector3d> &uav_positions,
                             Matrix3d &Rot,
                             Vector3d &trans,
                             double &Coeff,
                             const int &id) {
    // 相互定位计算两台无人机的世界坐标系转换关系
    // dyn_positions是本机观测到到的动态物体在自己世界系下的运动轨迹，
    // 将与本机收到的其他无人机传过来的LIO（在队友自己的世界坐标系下）轨迹uav_positions进行匹配
    // 算法原理就是SVD分解求R,t
    V3D dyn_mean = Zero3d;
    V3D uav_mean = Zero3d;
    for (size_t i = 0; i < dyn_positions.size(); ++i) {
        dyn_mean += dyn_positions[i];
        uav_mean += uav_positions[i];
    }

    dyn_mean /= double(uav_positions.size());
    uav_mean /= double(uav_positions.size());


    M3D H = M3D::Zero();
    for (size_t i = 0; i < dyn_positions.size(); ++i) {
        H += (uav_positions[i] - uav_mean) * (dyn_positions[i] - dyn_mean).transpose();
    }

    JacobiSVD<MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);

    //反射修正
    M3D mirror = M3D::Identity();
    mirror(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant();
    Rot = svd.matrixV() * mirror * svd.matrixU().transpose();
    trans = dyn_mean - Rot * uav_mean;

    double total_match_err = 0.0;
    for (size_t i = 0; i < dyn_positions.size(); ++i) {
        total_match_err += (dyn_positions[i] - Rot * uav_positions[i] - trans).norm();
    }

    double ave_match_err = total_match_err / double(dyn_positions.size());
    cout << " -- [Trajectory Match] Average Matching Error: " << ave_match_err << endl;

    if (ave_match_err > ave_match_error_thresh)
        return false;

    Coeff = ave_match_err / (svd.singularValues()(0) * svd.singularValues()(1)) * 1000;
    return true;
}

bool Multi_UAV::CreateTeammateTracker(const double &lidar_end_time, const int &index, StatesGroup &state_in,
                                      StatesGroup &state_prop, const bool &print_log) {
    auto dyn_pos_time = temp_tracker[index].dyn_pos_time;
    //Assess if the tracker's trajectory is excited enough
    V3D dyn_mean = Zero3d;
    for (size_t i = 0; i < dyn_pos_time.size(); ++i)
        dyn_mean += dyn_pos_time[i].block<3, 1>(0, 0);
    dyn_mean /= double(dyn_pos_time.size());

    M3D H = M3D::Zero();
    for (size_t i = 0; i < dyn_pos_time.size(); ++i) {
        H += (dyn_pos_time[i].block<3, 1>(0, 0) - dyn_mean) *
             (dyn_pos_time[i].block<3, 1>(0, 0) - dyn_mean).transpose();
    }

    JacobiSVD<MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double second_large_singular_value = svd.singularValues()(1);


    if(print_log && second_large_singular_value > 30)
        cout << " -- [Temp " << temp_tracker[index].id << "] Trajectory Matching Threshold = " << BOLDYELLOW
             << second_large_singular_value
             << RESET << endl;

    if (second_large_singular_value > traj_matching_start_thresh) {
        for (auto iter = teammates.begin(); iter != teammates.end(); ++iter) {   //每个队友飞机
            int id = iter->first;
            auto iter_tracker = teammate_tracker.find(id);
            if (iter->second.teammate_odom_time.empty() ||
                iter_tracker != teammate_tracker.end())  //如果队友的LIO还是空的，或者已经被Track了
                continue;
            else {
                vector<V3D> dyn_pos, uav_pos;
                auto temp_dyn_pose = temp_tracker[index].dyn_pos_time;
                dyn_pos.clear();
                uav_pos.clear();
                for (int i = 0; i < iter->second.teammate_odom_time.size(); ++i) {  //队友的每个odom (200)
                    while (!temp_dyn_pose.empty() &&
                           temp_dyn_pose.front()(3) < iter->second.teammate_odom_time[i](3)) {
                        temp_dyn_pose.pop_front();
                    }
                    //找到与收到的队友轨迹时间戳最接近的观测轨迹
                    if (abs(temp_dyn_pose.front()(3) - iter->second.teammate_odom_time[i](3)) < 1e-5) {
                        dyn_pos.push_back(temp_dyn_pose.front().head(3));
                        uav_pos.push_back(iter->second.teammate_odom_time[i].head(3));
                        continue;
                    }
                }

                if (dyn_pos.size() < 50)
                    break;
                //Trajectory Matching
                M3D rot(M3D::Identity());
                V3D trans = Zero3d;
                double noise = 1e-5;
                cout << BOLDBLUE << " -- [Trajectory Matching] with Drone " << id << RESET << endl;


                if (TrajMatching(dyn_pos, uav_pos, rot, trans, noise, id)) {
                    cout << BOLDMAGENTA << "R: " << RotMtoEuler(rot).transpose() * 57.3 << " deg" << endl
                         << "t: " << trans.transpose() << " m" << RESET << endl;
                    state.global_extrinsic_rot[id] = state_prop.global_extrinsic_rot[id] = rot;
                    state.global_extrinsic_trans[id] = state_prop.global_extrinsic_trans[id] = trans;

                    state.cov.block<6, 6>(18 + 6 * id, 18 + 6 * id) = noise * Matrix<double, 6, 6>::Identity();
                    state_prop.cov.block<6, 6>(18 + 6 * id, 18 + 6 * id) = noise * Matrix<double, 6, 6>::Identity();
                    state_in = state;

                    teammate_tracker.insert(id2ekf(id, temp_tracker[index].dyn_tracker));
                    cout << BOLDGREEN << " -- [Trajectory Matching] Find New Teammate, ID: " << id << RESET << endl;
                    VisualizeTempTrackerDelete(lidar_end_time, index);
                    temp_tracker.erase(temp_tracker.begin() + index);

                    //Extrinsic Infection Model
                    mars::EdgeData edge;
                    edge.from = drone_id;
                    edge.to = id;
                    edge.rotation = rot;
                    edge.translation = trans;
                    extrinsic_infection.simple_graph.AddEdge(edge);

                    //Add to list
                    teammate_id_by_traj_matching.push_back(id);

                    return true; //匹配成功
                }
            }
        }
    }
    return false;
}

void Multi_UAV::CreateTempTrackerByHighIntensity(const double &lidar_end_time) {
    if(found_all_teammates)
        return;


    if (teammate_tracker.size() >= actual_uav_num - 1) {  //找到了所有的队友，并且创建了teammate tracker，则不会再尝试发现新队友
        for (int i = 0; i < temp_tracker.size(); ++i)
            VisualizeTempTrackerDelete(lidar_end_time, i);
        temp_tracker.clear();
        found_all_teammates = true;
        cout << BOLDGREEN << " -- [Initialization] Found all teammates !!!!!!" <<  RESET << endl;
        return;
    }
    for (int i = 0; i < cluster_pos_tag.size(); i++) {
        if (cluster_pos_tag[i].is_high_intensity) {
            //Create New Tracker
            ESIKF drone_tracker(6, 3, 6);
            V3D teammate_pos_in_world =
                    state.rot_end * cluster_pos_tag[i].pos_in_body + state.pos_end;  //初始位置：cluster在本机世界系的位置
            V3D teammate_vel_in_world = Zero3d; //初始速度：0
            Matrix<double, 6, 1> ekf_init_state;
            ekf_init_state.head(3) = teammate_pos_in_world;
            ekf_init_state.tail(3) = teammate_vel_in_world;
            drone_tracker.init(ekf_init_state, lidar_end_time);
            int new_id = 0;
            if (!temp_tracker.empty())
                new_id = temp_tracker.back().id + 1;
            cout << BOLDYELLOW << " -- [Temp Tracker] Create New Temporary Tracker: " << new_id << RESET << endl;
            temp_tracker.push_back(TemporaryTracker(drone_tracker, lidar_end_time, new_id));
        }
    }
}

void Multi_UAV::CheckTempClusterValidation(const int &index) {
    //RESET 观测状态
    temp_tracker[index].exist_meas = false;
    temp_tracker[index].meas_of_tracker = Zero3d;
    auto &tracker = temp_tracker[index].dyn_tracker;
    V3D pos_predict = state.rot_end.transpose() * (tracker.get_state_pos() - state.pos_end); //EKF的预测
    double min_dist = DBL_MAX;  //找到最近的一个cluster
    for (int j = 0; j < cluster_pos_tag.size(); j++) {
        V3D resi_vec = cluster_pos_tag[j].pos_in_body - pos_predict;
        if (resi_vec.norm() < valid_temp_cluster_dist_thresh && resi_vec.norm() < min_dist) {
            min_dist = resi_vec.norm();
            temp_tracker[index].exist_meas = true;
            temp_tracker[index].meas_of_tracker = cluster_pos_tag[j].pos_in_body;
            cluster_pos_tag.erase(cluster_pos_tag.begin() + j); //删除匹配成功的cluster pose，减少后续计算
            j--;
        }
    }
}

void Multi_UAV::PredictTemporaryTracker(const double &lidar_end_time, const int &index) {
    temp_tracker[index].dyn_tracker.predict(lidar_end_time);
}

void Multi_UAV::UpdateTemporaryTracker(const double &lidar_end_time, const int &index, const bool &print_log) {
    double cluster_update_dt = lidar_end_time - temp_tracker[index].dyn_tracker.last_update_time_;
    if (temp_tracker[index].exist_meas) {
        //Update EKF tracker
        Matrix<double, 6, 1> ekf_meas;
        ekf_meas.head(3) = state.rot_end * temp_tracker[index].meas_of_tracker + state.pos_end;
        ekf_meas.tail(3) = Zero3d;
        if (cluster_update_dt > 1.5) {
            //Reset EKF Tracker
            temp_tracker[index].dyn_tracker.reset(ekf_meas, lidar_end_time);
            return;
        }
        temp_tracker[index].dyn_tracker.set_H_vel(Matrix3d::Zero());
        temp_tracker[index].dyn_tracker.update(ekf_meas, 1, lidar_end_time, 0.001);
        temp_tracker[index].dyn_tracker.last_update_time_ = lidar_end_time;
    }
    if (temp_tracker[index].dyn_pos_time.size() >= pos_num_in_traj)
        temp_tracker[index].dyn_pos_time.pop_front();
    Vector4d pos_time;
    pos_time.block<3, 1>(0, 0) = temp_tracker[index].dyn_tracker.get_state_pos();
    pos_time(3) = lidar_end_time;
    temp_tracker[index].dyn_pos_time.push_back(pos_time);
}

void Multi_UAV::VisualizeText(const ros::Publisher &pub, const double &time, const int &id, const double &scale, const V3D &pos, const string &text, const V3D &color){
    //Publish Text
    visualization_msgs::Marker vis_text;
    vis_text.header.stamp = ros::Time().fromSec(time);
    vis_text.header.frame_id = topic_name_prefix + "world";
    vis_text.action = visualization_msgs::Marker::ADD;
    vis_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    vis_text.id = id;
    vis_text.color.a = 1.0; // Don't forget to set the alpha!
    vis_text.color.r = color(0);
    vis_text.color.g = color(1);
    vis_text.color.b = color(2);
    vis_text.scale.x = scale;
    vis_text.scale.y = scale;
    vis_text.scale.z = scale;

    vis_text.pose.position.x = pos(0);
    vis_text.pose.position.y = pos(1);
    vis_text.pose.position.z = pos(2);
    vis_text.pose.orientation.x = 0.0;
    vis_text.pose.orientation.y = 0.0;
    vis_text.pose.orientation.z = 0.0;
    vis_text.pose.orientation.w = 1.0;
    vis_text.text = text;
    pub.publish(vis_text);
}

void Multi_UAV::VisualizeBoundingBox(const ros::Publisher &pub, const double &time, const int &id, const V3D &color, const V3D &pos, const double &size){
    //Publish Bounding box
    visualization_msgs::Marker line_strip;
    line_strip.header.stamp = ros::Time().fromSec(time);
    line_strip.header.frame_id = topic_name_prefix + "world";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = id; //unique id, useful when multiple markers exist.
    line_strip.type = visualization_msgs::Marker::LINE_STRIP; //marker type
    line_strip.scale.x = 0.1;

    line_strip.color.r = color.x();
    line_strip.color.g = color.y();
    line_strip.color.b = color.z();

    line_strip.color.a = 1.0; //不透明度，设0则全透明
    geometry_msgs::Point p[8];
    double length = size, width = size, hight = size;
    //vis_pos_world是目标物的坐标
    p[0].x = pos(0) - width;
    p[0].y = pos(1) + length;
    p[0].z = pos(2) + hight;
    p[1].x = pos(0) - width;
    p[1].y = pos(1) - length;
    p[1].z = pos(2) + hight;
    p[2].x = pos(0) - width;
    p[2].y = pos(1) - length;
    p[2].z = pos(2) - hight;
    p[3].x = pos(0) - width;
    p[3].y = pos(1) + length;
    p[3].z = pos(2) - hight;
    p[4].x = pos(0) + width;
    p[4].y = pos(1) + length;
    p[4].z = pos(2) - hight;
    p[5].x = pos(0) + width;
    p[5].y = pos(1) - length;
    p[5].z = pos(2) - hight;
    p[6].x = pos(0) + width;
    p[6].y = pos(1) - length;
    p[6].z = pos(2) + hight;
    p[7].x = pos(0) + width;
    p[7].y = pos(1) + length;
    p[7].z = pos(2) + hight;
    //LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
    for (int i = 0; i < 8; i++) {
        line_strip.points.push_back(p[i]);
    }
    //为了保证矩形框的八条边都存在：
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[3]);
    line_strip.points.push_back(p[2]);
    line_strip.points.push_back(p[5]);
    line_strip.points.push_back(p[6]);
    line_strip.points.push_back(p[1]);
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[7]);
    line_strip.points.push_back(p[4]);
    pub.publish(line_strip);
}


void Multi_UAV::VisualizeDeleteAllCluster(const double &time) {
    if (pubCluster.getNumSubscribers() < 1)
        return;
    visualization_msgs::Marker vis_cluster_delete;
    vis_cluster_delete.header.stamp = ros::Time().fromSec(time);
    vis_cluster_delete.header.frame_id = topic_name_prefix + "world";
    vis_cluster_delete.action = visualization_msgs::Marker::DELETEALL;
    pubCluster.publish(vis_cluster_delete);
}

void Multi_UAV::PublishTeammateOdom(const double &lidar_end_time){
    cout << BOLDREDPURPLE << " -- [Teammate List] ";
    for (auto iter = teammate_tracker.begin(); iter != teammate_tracker.end(); ++iter){
        int teammate_id = iter->first;
        cout << teammate_id << "  ";
        if(pubTeammateOdom[teammate_id].getNumSubscribers() < 1)
            continue;

        auto iter_teammate = teammates.find(teammate_id);
        //V3D teammate_pos_gravity = rot_world_to_gravity * iter->second.get_state_pos();
        //2024.3.14
        //Use global extrinsic to transform teammate position into self gravity frame
        V3D teammate_pos_gravity = rot_world_to_gravity * (state.global_extrinsic_rot[teammate_id] * iter_teammate->second.teammate_state.pos_end + state.global_extrinsic_trans[teammate_id]);
        TeammateOdom.header.frame_id = topic_name_prefix + "world";
        TeammateOdom.child_frame_id = "quad" + SetString(teammate_id) + "_aft_mapped";
        TeammateOdom.header.stamp = ros::Time().fromSec(lidar_end_time);
        TeammateOdom.pose.pose.position.x = teammate_pos_gravity(0);
        TeammateOdom.pose.pose.position.y = teammate_pos_gravity(1);
        TeammateOdom.pose.pose.position.z = teammate_pos_gravity(2);


        M3D teammate_rot_gravity = rot_world_to_gravity * state.global_extrinsic_rot[teammate_id] * iter_teammate->second.teammate_state.rot_end;
        geometry_msgs::Quaternion geoQuat;
        V3D euler_cur = RotMtoEuler(teammate_rot_gravity);
        geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                (euler_cur(0), euler_cur(1), euler_cur(2));
        TeammateOdom.pose.pose.orientation.x = geoQuat.x;
        TeammateOdom.pose.pose.orientation.y = geoQuat.y;
        TeammateOdom.pose.pose.orientation.z = geoQuat.z;
        TeammateOdom.pose.pose.orientation.w = geoQuat.w;


        V3D teammate_vel_gravity = rot_world_to_gravity * state.global_extrinsic_rot[teammate_id] * iter_teammate->second.teammate_state.vel;
        TeammateOdom.twist.twist.linear.x = teammate_vel_gravity(0);
        TeammateOdom.twist.twist.linear.y = teammate_vel_gravity(1);
        TeammateOdom.twist.twist.linear.z = teammate_vel_gravity(2);
        pubTeammateOdom[teammate_id].publish(TeammateOdom);
    }
    cout << RESET << endl;
}

void Multi_UAV::PublishConnectedTeammateList(const double &lidar_end_time){
    swarm_msgs::ConnectedTeammateList msg;
    msg.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg.drone_id = drone_id;
    msg.connected_teammate_id.clear();
    for (auto iter = teammate_tracker.begin(); iter != teammate_tracker.end(); ++iter){
        int teammate_id = iter->first;
        auto teammate = teammates.find(teammate_id);
        double connect_interval = lidar_end_time - teammate->second.last_connect_time;
        if (connect_interval < 2.0)
            msg.connected_teammate_id.push_back(teammate_id);
    }
    sort(msg.connected_teammate_id.begin(), msg.connected_teammate_id.end());
    pubTeammateList.publish(msg);

    std_msgs::Int8 msg_int;
    msg_int.data = -1;
    for (auto iter = teammates.begin(); iter != teammates.end(); ++iter) {
        double connect_interval = lidar_end_time - iter->second.last_connect_time;
        if (connect_interval < 2.0){
            msg_int.data = 0;
            break;
        }
    }
    if(msg_int.data == 0 && teammate_tracker.size() > 0)
        msg_int.data = teammate_tracker.size();
    
    pubTeammateNum.publish(msg_int);

    //Pub Teammate List with Trajectory Matching
    swarm_msgs::ConnectedTeammateList list;
    list.header.stamp = ros::Time().fromSec(lidar_end_time);
    list.drone_id = drone_id;
    list.connected_teammate_id.clear();
    for (int i = 0; i < teammate_id_by_traj_matching.size(); ++i) {
        list.connected_teammate_id.push_back(teammate_id_by_traj_matching[i]);
    }
    sort(list.connected_teammate_id.begin(), list.connected_teammate_id.end());
    pubTeammateIdTrajMatching.publish(list);
}


void Multi_UAV::VisualizeTeammateTracker(const double &lidar_end_time, const int &teammate_id) {
    auto iter = teammate_tracker.find(teammate_id);
    if (iter != teammate_tracker.end()) {
        auto &tracker = iter->second;

        V3D vis_pos_gravity = rot_world_to_gravity * tracker.get_state_pos();
        auto iter_teammate = teammates.find(teammate_id);
        M3D vis_rot_gravity = rot_world_to_gravity * state.global_extrinsic_rot[teammate_id] * iter_teammate->second.teammate_state.rot_end;

        if(pubUAV.getNumSubscribers() < 1)
            return;

        //连续0.5s没有互观测就变黄
        V3D vis_color = Zero3d;
        if (lidar_end_time - tracker.last_update_time_ > 0.5)
            vis_color = V3D(1.0, 1.0, 0.0);
        else
            vis_color = V3D(0.0, 1.0, 0.0);

        //Publish Text
        string vis_text = "UAV" + SetString(teammate_id);
        VisualizeText(pubUAV, lidar_end_time, teammate_id, text_scale,  rot_world_to_gravity * state.rot_end * V3D(0.9, 0, 0.3) + vis_pos_gravity, vis_text, V3D(1,1,1));

        //Publish Bounding box
        VisualizeBoundingBox(pubUAV, lidar_end_time, teammate_id + 100, vis_color, vis_pos_gravity, predict_region_radius - 0.1);
    } else {
        visualization_msgs::Marker vis_uav_delete;
        vis_uav_delete.header.stamp = ros::Time().fromSec(lidar_end_time);
        vis_uav_delete.header.frame_id = topic_name_prefix + "world";
        vis_uav_delete.type = visualization_msgs::Marker::DELETE;
        vis_uav_delete.action = visualization_msgs::Marker::DELETE;
        vis_uav_delete.id = teammate_id;
        pubUAV.publish(vis_uav_delete);
        vis_uav_delete.id = teammate_id + 100;
        pubUAV.publish(vis_uav_delete);
    }
}

void Multi_UAV::VisualizeDisconnectedTracker(const double &lidar_end_time, const int &teammate_id, const Teammate &teammate) {
    auto iter = teammate_tracker.find(teammate_id);

    if (iter != teammate_tracker.end()) {
        double connect_interval = lidar_end_time - teammate.last_connect_time;
        if (connect_interval > 2.0) //队友失联2.0秒以上，标出该teammate tracker为失联状态
        {
            V3D vis_pos_gravity = rot_world_to_gravity * iter->second.get_state_pos();
            string vis_text = "UAV" + SetString(teammate_id) + " LOST";
            VisualizeText(pubMeshUAV, lidar_end_time, teammate_id, text_scale, rot_world_to_gravity * state.rot_end * V3D(0.5, 0, 0.5) + vis_pos_gravity, vis_text, V3D(0.8,0.2,0.2));
            VisualizeText(pubUAV, lidar_end_time, teammate_id, text_scale, rot_world_to_gravity * state.rot_end * V3D(0.5, 0, 0.5) + vis_pos_gravity, vis_text, V3D(0.8,0.2,0.2));
        }
    }
}


void Multi_UAV::VisualizeTeammateTrajectory(const ros::Publisher pub_, deque<Vector4d> &traj,
                                            const V3D position,
                                            const string namespace_,
                                            const double mkr_size,
                                            const double &timestamp,
                                            const int teammate_id,
                                            const int has_observation
) {

    //Visualize Teammate Trajectory
    V3D teammate_traj_color1 = V3D(0.5, 0.5, 0.5); //gray
    V3D teammate_traj_color2 = V3D(0.5, 0.5, 0.5);

    switch (teammate_id) {
        case 1: teammate_traj_color2 = V3D(1.0, 0.25, 0.25); break; //red
        case 2: teammate_traj_color2 = V3D(0, 1.0, 0.5); break; //green
        case 3: teammate_traj_color2 = V3D(0, 0.75, 1); break; //blue
        case 4: teammate_traj_color2 = V3D(1, 1, 0); break; //yellow
        case 5: teammate_traj_color2 = V3D(0.42, 0.35, 1.0); break; //purple
        default: teammate_traj_color2 = V3D(0.5, 0.5, 0.5); break; //gray
    }

    V3D last_pos, cur_pos;
    visualization_msgs::MarkerArray mrkarr;
    mrkarr.markers.clear();

//    traj.push_back(Vector4d(position.x(), position.y(), position.z(), has_observation));
    traj.push_back(Vector4d(position.x(), position.y(), position.z(), 1.0));
    cout << " Teammate traj.size() "<< traj.size() << endl;
    last_pos = traj[0].head(3);
    int cnt = 0;
    for (auto cur_index: traj) {
        V3D cur_pos = cur_index.head(3);
        visualization_msgs::Marker line_list;
        // publish lines
        line_list.header.frame_id = topic_name_prefix + "world";
        line_list.header.stamp = ros::Time().fromSec(timestamp);
        line_list.ns = namespace_;
        line_list.id = cnt++;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.type = visualization_msgs::Marker::ARROW;
        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_list.scale.x = mkr_size;
        line_list.scale.y = 0.0000001;
        line_list.scale.z = 0.0000001;
        if(int(cur_index(3)) == 0) {
            line_list.color.r = teammate_traj_color1.x();
            line_list.color.g = teammate_traj_color1.y();
            line_list.color.b = teammate_traj_color1.z();
        } else {
            line_list.color.r = teammate_traj_color2.x();
            line_list.color.g = teammate_traj_color2.y();
            line_list.color.b = teammate_traj_color2.z();
        }
        line_list.color.a = 1;
        // Create the vertices for the points and lines
        geometry_msgs::Point p;
        p.x = last_pos.x();
        p.y = last_pos.y();
        p.z = last_pos.z();
        // The line list needs two points for each line
        line_list.points.push_back(p);
        p.x = cur_pos.x();
        p.y = cur_pos.y();
        p.z = cur_pos.z();
        // The line list needs
        line_list.points.push_back(p);
        mrkarr.markers.push_back(line_list);
        last_pos = cur_pos;
    }
    pub_.publish(mrkarr);
}


void Multi_UAV::VisualizeTeammateTrajectorySphere(const ros::Publisher pub_, deque<Vector4d> &traj,
                                            const V3D position,
                                            const string namespace_,
                                            const double mkr_size,
                                            const double &timestamp,
                                            const int teammate_id,
                                            const int has_observation
) {
    //Visualize Teammate Trajectory
    V3D teammate_traj_color1 = V3D(0.5, 0.5, 0.5); //gray
    V3D teammate_traj_color2 = V3D(0.5, 0.5, 0.5);

    switch (teammate_id) {
        case 1: teammate_traj_color2 = V3D(1.0, 0.25, 0.25); break; //red
        case 2: teammate_traj_color2 = V3D(0, 1.0, 0.5); break; //green
//        case 3: teammate_traj_color2 = V3D(1, 0.65, 0.0); break; //orange
        case 3: teammate_traj_color2 = V3D(0, 0.75, 1); break; //blue
        case 4: teammate_traj_color2 = V3D(1, 1, 0); break; //yellow
        case 5: teammate_traj_color2 = V3D(0.42, 0.35, 1.0); break; //purple
        default: teammate_traj_color2 = V3D(0.5, 0.5, 0.5); break; //gray
    }

    V3D last_pos, cur_pos;
    visualization_msgs::MarkerArray mrkarr;
    mrkarr.markers.clear();

//    traj.push_back(Vector4d(position.x(), position.y(), position.z(), has_observation));
    traj.push_back(Vector4d(position.x(), position.y(), position.z(), 1.0));
    cout << " Teammate traj.size() "<< traj.size() << endl;
    last_pos = traj[0].head(3);
    int cnt = 0;
    for (auto cur_index: traj) {
        V3D cur_pos = cur_index.head(3);
        visualization_msgs::Marker line_list;
        // publish lines
        line_list.header.frame_id = topic_name_prefix + "world";
        line_list.header.stamp = ros::Time().fromSec(timestamp);
        line_list.ns = namespace_;
        line_list.id = cnt++;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.type = visualization_msgs::Marker::SPHERE;
        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_list.scale.x = mkr_size;
        line_list.scale.y = mkr_size;
        line_list.scale.z = mkr_size;
        if(int(cur_index(3)) == 0) {
            line_list.color.r = teammate_traj_color1.x();
            line_list.color.g = teammate_traj_color1.y();
            line_list.color.b = teammate_traj_color1.z();
        } else {
            line_list.color.r = teammate_traj_color2.x();
            line_list.color.g = teammate_traj_color2.y();
            line_list.color.b = teammate_traj_color2.z();
        }
        line_list.color.a = 1;
        line_list.pose.position.x = cur_pos.x();
        line_list.pose.position.y = cur_pos.y();
        line_list.pose.position.z = cur_pos.z();
        mrkarr.markers.push_back(line_list);
        last_pos = cur_pos;
    }
    pub_.publish(mrkarr);
}


void Multi_UAV::VisualizeMeshUAV(const double &lidar_end_time, const int &teammate_id, const Teammate &teammate){
    auto iter = teammate_tracker.find(teammate_id);
    if (iter != teammate_tracker.end()) {
        auto &tracker = iter->second;

        if(pubMeshUAV.getNumSubscribers() < 1)
            return;

        V3D vis_pos_gravity = rot_world_to_gravity * tracker.get_state_pos();
        M3D vis_rot_gravity = rot_world_to_gravity * state.global_extrinsic_rot[teammate_id] * teammate.teammate_state.rot_end;

        //Publish Text
        string vis_text = "UAV" + SetString(teammate_id);
        VisualizeText(pubMeshUAV, lidar_end_time, teammate_id, text_scale, rot_world_to_gravity * state.rot_end * V3D(0.5, 0, 0.5) + vis_pos_gravity, vis_text, V3D(1,1,1));

        //连续0.5s没有互观测就变黄
        V3D vis_color = Zero3d;
        int has_observation = 0;
        if (lidar_end_time - tracker.last_update_time_ > 0.2)
            vis_color = V3D(1.0, 1.0, 0.0);
        else{
            int has_observation = 1;
            vis_color = V3D(1.0, 1.0, 1.0);
        }

        vis_color = V3D(1.0, 1.0, 1.0);
        //Publish Mesh UAV
        visualization_msgs::Marker meshROS;

        // Mesh model
        meshROS.header.frame_id = topic_name_prefix + "world";
        meshROS.header.stamp = ros::Time().fromSec(lidar_end_time);
        meshROS.ns = "mesh";
        meshROS.id = teammate_id + 100;
        meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
        meshROS.action = visualization_msgs::Marker::ADD;
        meshROS.pose.position.x = vis_pos_gravity.x();
        meshROS.pose.position.y = vis_pos_gravity.y();
        meshROS.pose.position.z = vis_pos_gravity.z();

        Quaterniond quat(vis_rot_gravity);
        meshROS.pose.orientation.w = quat.w();
        meshROS.pose.orientation.x = quat.x();
        meshROS.pose.orientation.y = quat.y();
        meshROS.pose.orientation.z = quat.z();
        meshROS.scale.x = mesh_scale;
        meshROS.scale.y = mesh_scale;
        meshROS.scale.z = mesh_scale;
        meshROS.color.a = 1;
        meshROS.color.r = vis_color.x();
        meshROS.color.g = vis_color.y();
        meshROS.color.b = vis_color.z();
        meshROS.mesh_resource = "package://swarm_lio/mesh/yunque-M.dae";
        meshROS.mesh_use_embedded_materials = true;
        pubMeshUAV.publish(meshROS);

//        VisualizeTeammateTrajectory(pubTeammateTraj[teammate_id], teammate_traj[teammate_id],
//                                    vis_pos_gravity,"Teammate" + SetString(teammate_id), 0.15, lidar_end_time, teammate_id, has_observation);

    } else {
        visualization_msgs::Marker vis_uav_delete;
        vis_uav_delete.header.stamp = ros::Time().fromSec(lidar_end_time);
        vis_uav_delete.header.frame_id = topic_name_prefix + "world";
        vis_uav_delete.type = visualization_msgs::Marker::DELETE;
        vis_uav_delete.action = visualization_msgs::Marker::DELETE;
        vis_uav_delete.id = teammate_id;
        pubMeshUAV.publish(vis_uav_delete);
        vis_uav_delete.ns = "mesh";
        vis_uav_delete.id = teammate_id + 100;
        pubMeshUAV.publish(vis_uav_delete);
    }
}

void Multi_UAV::VisualizeTempTracker(const double &lidar_end_time) {
    if(pubTempTracker.getNumSubscribers() < 1)
        return;
    for (int index = 0; index < temp_tracker.size(); ++index) {
        V3D vis_pos_gravity = rot_world_to_gravity * temp_tracker[index].dyn_tracker.get_state_pos();
        VisualizeBoundingBox(pubTempTracker, lidar_end_time, temp_tracker[index].id + 100, V3D(1,0,0), vis_pos_gravity, temp_predict_region_radius - 0.1);
    }
}

void Multi_UAV::VisualizeTemporaryPredictRegion(const double &lidar_end_time) {
    if(pubTempTracker.getNumSubscribers() < 1)
        return;
    visualization_msgs::Marker sphere;
    sphere.header.stamp = ros::Time().fromSec(lidar_end_time);
    sphere.header.frame_id = topic_name_prefix + "world";
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose.orientation.w = 1.0;
    sphere.type = visualization_msgs::Marker::SPHERE; //marker type
    sphere.scale.x = temp_predict_region_radius * 2; //直径
    sphere.scale.y = temp_predict_region_radius * 2;
    sphere.scale.z = temp_predict_region_radius * 2;
    sphere.color.r = 1.0;
    sphere.color.g = 0.0;
    sphere.color.b = 0.0;
    sphere.color.a = 0.5;
    for (int i = 0; i < temp_tracker.size(); i++) {
        V3D predict_pos_in_gravity = rot_world_to_gravity * temp_tracker[i].dyn_tracker.get_state_pos();
        sphere.pose.position.x = predict_pos_in_gravity.x();
        sphere.pose.position.y = predict_pos_in_gravity.y();
        sphere.pose.position.z = predict_pos_in_gravity.z();
        sphere.id = temp_tracker[i].id + 200;
        pubTempTracker.publish(sphere);
    }
}

void Multi_UAV::VisualizeTempTrackerDelete(const double &lidar_end_time, const int &index) {
    if(pubTempTracker.getNumSubscribers() < 1)
        return;
    visualization_msgs::Marker vis_uav_delete;
    vis_uav_delete.header.stamp = ros::Time().fromSec(lidar_end_time);
    vis_uav_delete.header.frame_id = topic_name_prefix + "world";
    vis_uav_delete.type = visualization_msgs::Marker::DELETE;
    vis_uav_delete.action = visualization_msgs::Marker::DELETE;
    vis_uav_delete.id = temp_tracker[index].id;
    pubTempTracker.publish(vis_uav_delete);
    vis_uav_delete.id = temp_tracker[index].id + 100;
    pubTempTracker.publish(vis_uav_delete);
    vis_uav_delete.id = temp_tracker[index].id + 200;
    pubTempTracker.publish(vis_uav_delete);
}

void Multi_UAV::VisualizeCluster(const double &lidar_end_time, const V3D &pos, const int &cluster_index, const V3D &size) {
    if (pubCluster.getNumSubscribers() < 1)
        return;

    V3D cluster_pos_gravity = rot_world_to_gravity* (state.rot_end * pos + state.pos_end);
    string vis_text = "Cluster " + SetString(cluster_index);
    VisualizeText(pubCluster, lidar_end_time, cluster_index, 0.4, cluster_pos_gravity, vis_text, V3D(1,1,1));
    VisualizeBoundingBox(pubCluster, lidar_end_time, cluster_index + 100, V3D(1,1,1), cluster_pos_gravity, size.x());
}


void Multi_UAV::VisualizePredictRegion(const double &lidar_end_time, const int &id) {
    if(pubPredictRegion.getNumSubscribers() < 1)
        return;

    auto iter = teammate_tracker.find(id);
    if (iter != teammate_tracker.end()) {
        visualization_msgs::Marker sphere;
        sphere.header.stamp = ros::Time().fromSec(lidar_end_time);
        sphere.header.frame_id = topic_name_prefix + "world";
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.pose.orientation.w = 1.0;
        sphere.type = visualization_msgs::Marker::SPHERE; //marker type
        sphere.scale.x = predict_region_radius * 2; //直径
        sphere.scale.y = predict_region_radius * 2;
        sphere.scale.z = predict_region_radius * 2;

        //连续1.0s没有互观测就变黄
        if (lidar_end_time - iter->second.last_update_time_ > 0.1) {
            sphere.color.r = 1.0;
            sphere.color.g = 1.0;
            sphere.color.b = 0.0;
        } else {
            sphere.color.r = 0.0;
            sphere.color.g = 1.0;
            sphere.color.b = 0.0;
        }
        sphere.color.a = 0.6;

        V3D predict_pos_in_gravity = rot_world_to_gravity * iter->second.get_state_pos();
        sphere.pose.position.x = predict_pos_in_gravity.x();
        sphere.pose.position.y = predict_pos_in_gravity.y();
        sphere.pose.position.z = predict_pos_in_gravity.z();
        sphere.id = iter->first;
        pubPredictRegion.publish(sphere);
    }else{
        visualization_msgs::Marker vis_uav_delete;
        vis_uav_delete.header.stamp = ros::Time().fromSec(lidar_end_time);
        vis_uav_delete.header.frame_id = topic_name_prefix + "world";
        vis_uav_delete.type = visualization_msgs::Marker::DELETE;
        vis_uav_delete.action = visualization_msgs::Marker::DELETE;
        vis_uav_delete.id = id;
        pubPredictRegion.publish(vis_uav_delete);
    }
}

void Multi_UAV::VisualizeRectangle(const ros::Publisher &pub_rect, const double &lidar_end_time, const int &rect_id, const V3D &position, const V3D rect_size) {
    visualization_msgs::Marker line_strip;
    line_strip.header.stamp = ros::Time().fromSec(lidar_end_time);
    line_strip.header.frame_id = topic_name_prefix + "world";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = rect_id;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.1;
    line_strip.color.r = 1.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    geometry_msgs::Point p[8];
    double length = rect_size.y()/2, width = rect_size.x()/2, hight =
            rect_size.z()/2;
    //vis_pos_world是目标物的坐标
    p[0].x = position(0) - width;
    p[0].y = position(1) + length;
    p[0].z = position(2) + hight;
    p[1].x = position(0) - width;
    p[1].y = position(1) - length;
    p[1].z = position(2) + hight;
    p[2].x = position(0) - width;
    p[2].y = position(1) - length;
    p[2].z = position(2) - hight;
    p[3].x = position(0) - width;
    p[3].y = position(1) + length;
    p[3].z = position(2) - hight;
    p[4].x = position(0) + width;
    p[4].y = position(1) + length;
    p[4].z = position(2) - hight;
    p[5].x = position(0) + width;
    p[5].y = position(1) - length;
    p[5].z = position(2) - hight;
    p[6].x = position(0) + width;
    p[6].y = position(1) - length;
    p[6].z = position(2) + hight;
    p[7].x = position(0) + width;
    p[7].y = position(1) + length;
    p[7].z = position(2) + hight;
    //LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
    for (int i = 0; i < 8; i++) {
        line_strip.points.push_back(p[i]);
    }
    //为了保证矩形框的八条边都存在：
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[3]);
    line_strip.points.push_back(p[2]);
    line_strip.points.push_back(p[5]);
    line_strip.points.push_back(p[6]);
    line_strip.points.push_back(p[1]);
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[7]);
    line_strip.points.push_back(p[4]);
    pub_rect.publish(line_strip);
}

