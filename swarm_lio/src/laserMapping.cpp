#include <omp.h>
#include "IMUProcessing.hpp"
#include <unistd.h>
#include <Python.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include "MultiUAV.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <ifaddrs.h>
#include "cpu_memory_query.h"
#include "malloc.h"

//#define BACKWARD_HAS_DW 1
//#include "backward.hpp"
//namespace backward {
//    backward::SignalHandling sh;
//}

#ifndef DEPLOY
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#endif

#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer_lidar, mtx_buffer_imu, mtx_buffer_imu_prop, mtx_buffer_gt;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, \
 effect_meas_num = 0, effect_feat_num = 0, scan_count = 0, point_filter_num = 1;

double gyr_cov = 0.1, acc_cov = 0.1, grav_cov = 0.0001, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double last_timestamp_lidar = 0, last_timestamp_imu = 0.0;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0;
double cube_len = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = - 1.0;
ros::Publisher pubDrone;

// Time Log Variables
double ave_pre_processing_time = 0.0, ave_frame_time = 0.0;
int kdtree_delete_counter = 0, kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0;

int lidar_type, pcd_save_interval = -1, pcd_index = 0;
bool lidar_pushed, flg_reset, flg_exit = false, flg_EKF_inited = true;
bool imu_en = true;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool runtime_pos_log = false, pcd_save_en = false, path_en = true;
double pcd_resolution = 0.03;;
// LiDAR-IMU Parameters
bool cut_frame_en{false}, accumulate_frame_en{false};
int cut_frame_num = 1, original_frequency = 10, frame_num = 0, imu_num = 0;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
M3D offset_R_L_I(M3D::Identity());
V3D offset_T_L_I(Zero3d);
double time_offset_lidar_imu = 0.0;
//Gravity Alignment Parameters
bool gravity_align_en{false}, gravity_align_finished{false};
double gravity_align_dur;
Matrix4d G_T_PX4, PX4_T_I, G_T_I0, Horizon_T_PX4;
V3D unbiased_gyr;

//Multi-UAV Parameters
ofstream fout_pose;
string sub_gt_pose_topic;
deque<VD(7)> ground_truth;
deque<VD(4)> LIO_pose;
int drone_id, actual_uav_num;
double mutual_observe_noise, mutual_observe_noise_;
shared_ptr<ImuProcess> p_imu;
string topic_name_prefix;
int frame_num_in_sliding_window;
double text_scale, mesh_scale;
int degeneration_thresh;

//IMU propagation Parameters
StatesGroup imu_propagate, latest_ekf_state;
bool new_imu{false}, state_update_flg{false}, imu_prop_enable{true}, ekf_finish_once{false};
deque<sensor_msgs::Imu> prop_imu_buffer;
sensor_msgs::Imu newest_imu;
double latest_ekf_time;
nav_msgs::Odometry imu_prop_odom;
ros::Publisher pubImuPropOdom;
string imu_prop_topic;

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
PointCloudXYZI::Ptr voxel_downsampled_map(new PointCloudXYZI());
pcl::PCDWriter pcd_writer;
string all_points_dir;

vector<BoxPointType> cub_needrm;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<double> time_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
vector<vector<int>> pointSearchInd_surf;
vector<PointVector> Nearest_Points;
bool point_selected_surf[100000] = {0};
float res_last[100000] = {0.0};
int points_cache_size = 0;

//Frame merge for cluster extraction
deque<PointCloudXYZI::Ptr> feats_world_sliding_window;

//surf feature in map
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort_lidar(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_body_dense_filter_dynamic(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort_orig_lidar(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_lidar(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterPCD;


//KD_TREE ikdtree;
KD_TREE<PointType> ikdtree;
V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);


//estimator inputs and output;
MeasureGroup Measures;
StatesGroup state;
nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;
shared_ptr<Preprocess> p_pre(new Preprocess());

Matrix4d T_inverse(const Matrix4d T) {
    Matrix4d T_out = Matrix4d::Identity();
    T_out.col(3).head(3) = -T.block<3, 3>(0, 0).transpose() * T.col(3).head(3);
    T_out.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();
    return T_out;
}

void ConvertTwistToGravFrame(V3D &L0_vel_L) {
    M3D omg_hat;
    omg_hat << SKEW_SYM_MATRX(unbiased_gyr);
    V3D PX4_p_I = PX4_T_I.col(3).head(3);
    V3D L0_vel_I = L0_vel_L - omg_hat * Horizon_T_PX4.block<3,3>(0,0) * PX4_T_I.block<3,3>(0,0) * PX4_p_I;
    L0_vel_L = G_T_I0.block<3,3>(0,0) * L0_vel_I;
}

void ConvertPoseToGravFrame(V3D &pos, Quaterniond &q) {
    M3D rot = q.toRotationMatrix();
    Matrix4d I0_T_I = Matrix4d::Identity();
    I0_T_I.block<3, 3>(0, 0) = rot;
    I0_T_I.col(3).head(3) = pos;
    Matrix4d T_out = G_T_I0 * I0_T_I * T_inverse(PX4_T_I) * T_inverse(Horizon_T_PX4);
    pos = T_out.col(3).head(3);
    rot = T_out.block<3, 3>(0, 0);
    q = Quaterniond(rot);
}


void prop_imu_once(StatesGroup & imu_prop_state,
                   const double dt,
                   V3D acc_avr,
                   V3D angvel_avr) {
    double mean_acc_norm = p_imu->IMU_mean_acc_norm;
    acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
    angvel_avr -= imu_prop_state.bias_g;
    unbiased_gyr = angvel_avr;
    M3D Exp_f = Exp(angvel_avr, dt);
    /* propogation of IMU attitude */
    imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

    /* Specific acceleration (global frame) of IMU */
    V3D acc_imu = imu_prop_state.rot_end * acc_avr +
                  V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

    /* propogation of IMU */
    imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

    /* velocity of IMU */
    imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void VisualizeDrone(ros::Publisher &pub) {
    Quaterniond quat;
    V3D cluster_pos_world;
    if(lidar_type == SIM){
        quat = G_T_I0.block<3,3>(0,0) * state.rot_end;
        cluster_pos_world = G_T_I0.block<3,3>(0,0) * state.pos_end;
    }else{
        quat.w() = imu_prop_odom.pose.pose.orientation.w;
        quat.x() = imu_prop_odom.pose.pose.orientation.x;
        quat.y() = imu_prop_odom.pose.pose.orientation.y;
        quat.z() = imu_prop_odom.pose.pose.orientation.z;
        cluster_pos_world = V3D(imu_prop_odom.pose.pose.position.x, imu_prop_odom.pose.pose.position.y,
                                imu_prop_odom.pose.pose.position.z);
    }
    V3D text_pos_world =  G_T_I0.block<3,3>(0,0) * (state.rot_end * V3D(0.4, 0.0, 0.3) + state.pos_end);
    visualization_msgs::Marker vis_drone;
    vis_drone.header.stamp = ros::Time().fromSec(lidar_end_time);
    vis_drone.header.frame_id = topic_name_prefix + "world";
    vis_drone.action = visualization_msgs::Marker::ADD;
    vis_drone.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    vis_drone.id = drone_id;
    vis_drone.color.a = 1.0; // Don't forget to set the alpha!
    vis_drone.color.r = 1.0;
    vis_drone.color.g = 1.0;
    vis_drone.color.b = 1.0;
    vis_drone.scale.x = text_scale;
    vis_drone.scale.y = text_scale;
    vis_drone.scale.z = text_scale;
    vis_drone.pose.position.x = text_pos_world(0);
    vis_drone.pose.position.y = text_pos_world(1);
    vis_drone.pose.position.z = text_pos_world(2);
    vis_drone.pose.orientation.x = 0.0;
    vis_drone.pose.orientation.y = 0.0;
    vis_drone.pose.orientation.z = 0.0;
    vis_drone.pose.orientation.w = 1.0;

    vis_drone.text = "UAV" + SetString(drone_id);
    pub.publish(vis_drone);

    visualization_msgs::Marker meshROS;
    // Mesh model
    meshROS.header.frame_id = topic_name_prefix + "world";
    meshROS.header.stamp = ros::Time().fromSec(lidar_end_time);
    meshROS.ns = "mesh";
    meshROS.id = drone_id + 100;
    meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = cluster_pos_world.x();
    meshROS.pose.position.y = cluster_pos_world.y();
    meshROS.pose.position.z = cluster_pos_world.z();
    meshROS.pose.orientation.w = quat.w();
    meshROS.pose.orientation.x = quat.x();
    meshROS.pose.orientation.y = quat.y();
    meshROS.pose.orientation.z = quat.z();
    meshROS.scale.x = mesh_scale;
    meshROS.scale.y = mesh_scale;
    meshROS.scale.z = mesh_scale;
    meshROS.color.a = 1;
    meshROS.color.r = 1.0;
    meshROS.color.g = 1.0;
    meshROS.color.b = 1.0;
    meshROS.mesh_resource = "package://swarm_lio/mesh/yunque-M.dae";
    meshROS.mesh_use_embedded_materials = true;
    pub.publish(meshROS);
}

void imu_prop_callback(const ros::TimerEvent &e) {
    if (p_imu->imu_need_init_ || !new_imu || !ekf_finish_once) {
        return;
    }
    mtx_buffer_imu_prop.lock();
    new_imu = false; //控制propagate频率和IMU频率一致
    if (imu_prop_enable && !prop_imu_buffer.empty()) {
        static double last_t_from_lidar_end_time = 0;
        if (state_update_flg) {
            imu_propagate = latest_ekf_state;
            // drop all useless imu pkg
            while ((!prop_imu_buffer.empty() && prop_imu_buffer.front().header.stamp.toSec() < latest_ekf_time)) {
                prop_imu_buffer.pop_front();
            }
            last_t_from_lidar_end_time = 0;
            for (int i = 0; i < prop_imu_buffer.size(); i++) {
                double t_from_lidar_end_time = prop_imu_buffer[i].header.stamp.toSec() - latest_ekf_time;
                double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
                //cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
                V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x,
                            prop_imu_buffer[i].linear_acceleration.y,
                            prop_imu_buffer[i].linear_acceleration.z);
                V3D omg_imu(prop_imu_buffer[i].angular_velocity.x,
                            prop_imu_buffer[i].angular_velocity.y,
                            prop_imu_buffer[i].angular_velocity.z);
                prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
                last_t_from_lidar_end_time = t_from_lidar_end_time;
            }
            state_update_flg = false;
        }
        else
        {
            V3D acc_imu(newest_imu.linear_acceleration.x,
                        newest_imu.linear_acceleration.y,
                        newest_imu.linear_acceleration.z);
            V3D omg_imu(newest_imu.angular_velocity.x,
                        newest_imu.angular_velocity.y,
                        newest_imu.angular_velocity.z);
            double t_from_lidar_end_time = newest_imu.header.stamp.toSec() - latest_ekf_time;
            double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
            prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
            last_t_from_lidar_end_time = t_from_lidar_end_time;
        }

        V3D posi, vel_i;
        Eigen::Quaterniond q;
        posi = imu_propagate.pos_end;
        vel_i = imu_propagate.vel_end;
        q = Eigen::Quaterniond(imu_propagate.rot_end);
        ConvertPoseToGravFrame(posi, q);
        ConvertTwistToGravFrame(vel_i);
        imu_prop_odom.header.frame_id = "world";
        imu_prop_odom.header.stamp = newest_imu.header.stamp;
        imu_prop_odom.pose.pose.position.x = posi.x();
        imu_prop_odom.pose.pose.position.y = posi.y();
        imu_prop_odom.pose.pose.position.z = posi.z();
        imu_prop_odom.pose.pose.orientation.w = q.w();
        imu_prop_odom.pose.pose.orientation.x = q.x();
        imu_prop_odom.pose.pose.orientation.y = q.y();
        imu_prop_odom.pose.pose.orientation.z = q.z();
        imu_prop_odom.twist.twist.linear.x = vel_i.x();
        imu_prop_odom.twist.twist.linear.y = vel_i.y();
        imu_prop_odom.twist.twist.linear.z = vel_i.z();
        pubImuPropOdom.publish(imu_prop_odom);

        static tf::TransformBroadcaster br1;
        tf::Transform transform;
        tf::Quaternion q1;
        transform.setOrigin(tf::Vector3(imu_prop_odom.pose.pose.position.x, \
                                imu_prop_odom.pose.pose.position.y, \
                                imu_prop_odom.pose.pose.position.z));
        q1.setW(imu_prop_odom.pose.pose.orientation.w);
        q1.setX(imu_prop_odom.pose.pose.orientation.x);
        q1.setY(imu_prop_odom.pose.pose.orientation.y);
        q1.setZ(imu_prop_odom.pose.pose.orientation.z);
        transform.setRotation(q1);
        br1.sendTransform(
                tf::StampedTransform(transform, imu_prop_odom.header.stamp, topic_name_prefix + "world",
                                     "quad" + SetString(drone_id) + "_imu_propagation"));
    }
    mtx_buffer_imu_prop.unlock();
}


float calc_dist(PointType p1, PointType p2) {
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

void calcBodyVar(Eigen::Vector3d &pb, const float range_inc,
                 const float degree_inc, Eigen::Matrix3d &var) {
    float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
    float range_var = range_inc * range_inc;
    Eigen::Matrix2d direction_var;
    direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0,
            pow(sin(DEG2RAD(degree_inc)), 2);
    Eigen::Vector3d direction(pb);
    direction.normalize();
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1), direction(2), 0,
            -direction(0), -direction(1), direction(0), 0;
    Eigen::Vector3d base_vector1(1, 1,
                                 -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
            base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
    var = direction * range_var * direction.transpose() +
          A * direction_var * A.transpose();
}

void SigHandle(int sig) {
    if (pcd_save_en && pcd_save_interval < 0) {
        all_points_dir = string(root_dir + "/PCD/PCD_all" + SetString(drone_id)) + string(".pcd");
        downSizeFilterPCD.setInputCloud(pcl_wait_save);
        downSizeFilterPCD.filter(*voxel_downsampled_map);
        pcd_writer.writeBinary(all_points_dir, *voxel_downsampled_map);
    }
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

void pointBodyToWorld(PointType const *const pi, PointType *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state.rot_end * (offset_R_L_I * p_body + offset_T_L_I) + state.pos_end);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->normal_x = pi->normal_x;
    po->normal_y = pi->normal_y;
    po->normal_z = pi->normal_z;
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po) {
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state.rot_end * (offset_R_L_I * p_body + offset_T_L_I) + state.pos_end);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointTypeRGB *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state.rot_end * (offset_R_L_I * p_body + offset_T_L_I) + state.pos_end);
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->r = pi->normal_x;
    po->g = pi->normal_y;
    po->b = pi->normal_z;
    float intensity = pi->intensity;
    intensity = intensity - floor(intensity);
    int reflection_map = intensity * 10000;
}

void points_cache_collect() {
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
    for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;

void lasermap_fov_segment() {
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = state.pos_end;
    if (!Localmap_Initialized) {
        //if (cube_len <= 2.0 * MOV_THRESHOLD * DET_RANGE) throw std::invalid_argument("[Error]: Local Map Size is too small! Please change parameter \"cube_side_length\" to larger than %d in the launch file.\n");
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    // printf("Local Map is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", LocalMap_Points.vertex_min[0],LocalMap_Points.vertex_max[0],LocalMap_Points.vertex_min[1],LocalMap_Points.vertex_max[1],LocalMap_Points.vertex_min[2],LocalMap_Points.vertex_max[2]);
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                         double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
            // printf("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
            // printf("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);
        }
    }
    LocalMap_Points = New_LocalMap_Points;
    points_cache_collect();
    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    // printf("Delete Box: %d\n",int(cub_needrm.size()));
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    scan_count++;
    TimeConsuming t1("Preprocess time");
    mtx_buffer_lidar.lock();
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        ROS_WARN("lidar loop back, clear buffer");
        lidar_buffer.clear();
        time_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if(accumulate_frame_en && !cut_frame_en){
        static PointCloudXYZI::Ptr ptr_accumulate(new PointCloudXYZI());
        static double first_lidar_head_time = msg->header.stamp.toSec();
        //合帧2s，合成后的每一帧的点云数目和10Hz点云相同
        if(scan_count < 2 * original_frequency){
            if(scan_count % (original_frequency/10) == 0)
                first_lidar_head_time = msg->header.stamp.toSec();

            p_pre->process_accumulate_frame_livox(msg, ptr_accumulate, first_lidar_head_time);
            if(scan_count % (original_frequency/10) != 0){
                mtx_buffer_lidar.unlock();
                sig_buffer.notify_all();
                return;
            }
            else
            {
                PointCloudXYZI::Ptr pcl_temp(new PointCloudXYZI());
                PointCloudXYZI pcl_temp1;
                pcl_temp1.clear();
                pcl_temp1 = *ptr_accumulate;
                *pcl_temp = pcl_temp1;
                lidar_buffer.push_back(pcl_temp);
                time_buffer.push_back(first_lidar_head_time);
                ptr_accumulate->clear();
            }
        }else{
            PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
            p_pre->process(msg, ptr);
            lidar_buffer.push_back(ptr);
            time_buffer.push_back(last_timestamp_lidar);
        }
    }else if(cut_frame_en && !accumulate_frame_en){
        deque<PointCloudXYZI::Ptr> ptr;//存储切割后的每一帧点云，每个点的curvature是相对于新的帧头的偏移时间
        deque<double> timestamp_lidar; //存储切割后每一帧的帧头时间（原绝对时间轴上的值）
        //输入原始的一帧点云数据，输出切割后的10帧点云和每一帧的帧头时间戳
        p_pre->process_cut_frame_livox(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
        //将切割后的点云和时间戳存入lidar_buffer和time_buffer，同时弹出ptr和timestamp_lidar
        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000));//单位:s
            timestamp_lidar.pop_front();
        }
    }else{
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(last_timestamp_lidar);
    }
    double t1_time = t1.stop() * 1000;
    ave_pre_processing_time += (t1_time - ave_pre_processing_time) / scan_count;
    mtx_buffer_lidar.unlock();
    sig_buffer.notify_all();
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    scan_count++;
    mtx_buffer_lidar.lock();
    if (msg->header.stamp.toSec() < last_timestamp_lidar) {
        ROS_ERROR("lidar loop back, clear Lidar buffer.");
        lidar_buffer.clear();
        time_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if ((lidar_type == VELO || lidar_type == OUSTER || lidar_type == PANDAR) && cut_frame_en) {
        deque<PointCloudXYZI::Ptr> ptr;
        deque<double> timestamp_lidar;
        p_pre->process_cut_frame_pcl2(msg, ptr, timestamp_lidar, cut_frame_num, scan_count);
        while (!ptr.empty() && !timestamp_lidar.empty()) {
            lidar_buffer.push_back(ptr.front());
            ptr.pop_front();
            time_buffer.push_back(timestamp_lidar.front() / double(1000));//单位:s
            timestamp_lidar.pop_front();
        }
    } else {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(msg->header.stamp.toSec());
    }
    mtx_buffer_lidar.unlock();
    sig_buffer.notify_all();
}

ros::Publisher pubFilteredImu;
V3D acc_raw[2] = {Zero3d, Zero3d};
V3D acc_filtered = Zero3d;
bool filter_acc_en{false};

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_offset_lidar_imu);
    double timestamp = msg->header.stamp.toSec();

    //Low-pass filter
    if(filter_acc_en){
        acc_raw[0] = acc_raw[1];
        acc_raw[1] = V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        V3D acc_filtered_now(Zero3d);
        acc_filtered_now = 0.25 * acc_raw[0] + 0.25 * acc_raw[1] + 0.5 * acc_filtered;
        acc_filtered = acc_filtered_now;
        msg->linear_acceleration.x = acc_filtered_now.x();
        msg->linear_acceleration.y = acc_filtered_now.y();
        msg->linear_acceleration.z = acc_filtered_now.z();
        pubFilteredImu.publish(msg);
    }


    mtx_buffer_imu.lock();
    if (timestamp < last_timestamp_imu) {
        ROS_WARN("IMU loop back, clear IMU buffer.");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);
    mtx_buffer_imu.unlock();
    //IMU propagation

    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init_) {
        prop_imu_buffer.push_back(*msg);
    }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
    sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas) {
    if (lidar_buffer.empty() || imu_buffer.empty())
        return false;

    /** push a lidar scan **/
    if (!lidar_pushed) {
        PointCloudXYZI lidar_sorted = *(lidar_buffer.front());
        /*** sort point clouds by offset time ***/
        sort(lidar_sorted.points.begin(), lidar_sorted.points.end(), time_list);
        *(meas.lidar) = lidar_sorted;
        if (meas.lidar->points.size() <= 1) {
            ROS_WARN("Too few input point cloud!\n");
            mtx_buffer_lidar.lock();
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            mtx_buffer_lidar.unlock();
            return false;
        }
        meas.lidar_beg_time = time_buffer.front(); //单位s
        if (lidar_type == SIM)
            lidar_end_time = meas.lidar_beg_time;
        else
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000); //单位:s
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
        return false;


    /** push imu data, and pop from imu buffer **/
    mtx_buffer_imu.lock();
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }
    mtx_buffer_imu.unlock();

    //LiDAR pop_front
    mtx_buffer_lidar.lock();
    lidar_buffer.pop_front();
    time_buffer.pop_front();
    mtx_buffer_lidar.unlock();
    lidar_pushed = false;
    return true;
}


void map_incremental() {
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++) {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_lidar->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited) {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min +
                          0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min &&
                fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        } else {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}



PointCloudXYZI::Ptr downsampled_map(new PointCloudXYZI());
PointCloudXYZI::Ptr downsampled_before_filter(new PointCloudXYZI());
pcl::VoxelGrid<PointType> downsample_filter_map;
ros::Publisher pub_map;
void publish_frame_world(const ros::Publisher &pubCloudRegistered, const ros::Publisher &pubCloudRegisteredSparse) {
    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_body_dense_filter_dynamic : feats_down_lidar);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
        PointCloudXYZI::Ptr laserCloudWorldSparse(new PointCloudXYZI(feats_down_lidar->points.size(), 1));

        //Pointcloud Transform
        Matrix4d lidar_to_world = Matrix4d::Identity();
        lidar_to_world.block<3, 3>(0, 0) = state.rot_end * offset_R_L_I;
        lidar_to_world.block<3, 1>(0, 3) = state.rot_end * offset_T_L_I + state.pos_end;
        Matrix4d lidar_to_gravity = G_T_I0 * lidar_to_world;
        pcl::transformPointCloud(*laserCloudFullRes, *laserCloudWorld, lidar_to_gravity);


        //Convert all original points into gravity-aligned world frame
        pcl::transformPointCloud(*feats_down_lidar, *laserCloudWorldSparse, lidar_to_gravity);


        sensor_msgs::PointCloud2 laserCloudmsg;
        sensor_msgs::PointCloud2 laserCloudSparsemsg;

        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        pcl::toROSMsg(*laserCloudWorldSparse, laserCloudSparsemsg);

        //Publish
        laserCloudmsg.header.stamp = laserCloudSparsemsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = laserCloudSparsemsg.header.frame_id = topic_name_prefix + "world";
        pubCloudRegisteredSparse.publish(laserCloudSparsemsg);

        if(pubCloudRegistered.getNumSubscribers() > 0)
            pubCloudRegistered.publish(laserCloudmsg);
    }
    /**************** save map ****************/
    /* 1. make sure you have enough memories
       2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en) {
        boost::filesystem::create_directories(root_dir + "/PCD");
        int size = feats_undistort_lidar->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        Matrix4d lidar_to_world;
        lidar_to_world.block<3, 3>(0, 0) = state.rot_end * offset_R_L_I;
        lidar_to_world.block<3, 1>(0, 3) = state.rot_end * offset_T_L_I + state.pos_end;
        Matrix4d body_to_gravity = G_T_I0 * lidar_to_world;
        pcl::transformPointCloud(*feats_down_lidar, *laserCloudWorld, body_to_gravity);

        *pcl_wait_save += *laserCloudWorld;
        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            all_points_dir = string(root_dir + "/PCD/PCD") + to_string(pcd_index) + string(".pcd");
            cout << "current scan saved to " << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher &pubCloudRegisteredBody_) {
    PointCloudXYZI::Ptr laserCloudFullResBody(feats_undistort_lidar);
    Matrix4d lidar_to_body = Matrix4d::Identity();
    lidar_to_body.block<3,3>(0,0) = offset_R_L_I;
    lidar_to_body.block<3,1>(0,3) = offset_T_L_I;
    pcl::transformPointCloud(*feats_undistort_lidar, *laserCloudFullResBody, lidar_to_body);
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudFullResBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = topic_name_prefix + "world";
    pubCloudRegisteredBody_.publish(laserCloudmsg);
}

template<typename T>
void set_posestamp(T &out) {
    Quaterniond q(geoQuat.w, geoQuat.x, geoQuat.y, geoQuat.z);
    V3D position_in_grav = state.pos_end;
    ConvertPoseToGravFrame(position_in_grav, q);

    out.position.x = position_in_grav(0);
    out.position.y = position_in_grav(1);
    out.position.z = position_in_grav(2);
    out.orientation.x = q.x();
    out.orientation.y = q.y();
    out.orientation.z = q.z();
    out.orientation.w = q.w();
}

template<typename T>
void set_twist(T &out) {
    Vector3d vel =  state.vel_end;
    ConvertTwistToGravFrame(vel);
    out.linear.x = vel(0);
    out.linear.y = vel(1);
    out.linear.z = vel(2);
}

void publish_odometry(const ros::Publisher &pubLidarSlamOdom) {
    odomAftMapped.header.frame_id = topic_name_prefix + "world";
    odomAftMapped.child_frame_id = "quad" + SetString(drone_id) + "_aft_mapped";


    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose.pose);
    set_twist(odomAftMapped.twist.twist);

    pubLidarSlamOdom.publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(
            tf::StampedTransform(transform, odomAftMapped.header.stamp, topic_name_prefix + "world",
                                 "quad" + SetString(drone_id) + "_aft_mapped"));
}

void publish_mavros(const ros::Publisher &mavros_pose_publisher) {
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);

    msg_body_pose.header.frame_id = topic_name_prefix + "world";
    set_posestamp(msg_body_pose.pose);
    mavros_pose_publisher.publish(msg_body_pose);
}

void publish_path(const ros::Publisher pubPath) {
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = topic_name_prefix + "world";

    //for short path
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
    //for long path
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) // if path is too large, the rvis will crash
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

void point_downsample(PointCloudXYZI &orig_pcl, PointCloudXYZI &pcl_out) {
    pcl_out.clear();
    pcl_out.reserve(orig_pcl.size());
    if (point_filter_num != 1) {
        for (int i = 0; i < orig_pcl.size(); i += point_filter_num) {
            pcl_out.push_back(orig_pcl.points[i]);
        }
    } else
        pcl_out = orig_pcl;
}

void point_filter_teammate(const PointCloudXYZI &orig_pcl, PointCloudXYZI &pcl_out, const shared_ptr<Multi_UAV> &swarm_in) {
    pcl_out.clear();
    pcl_out.reserve(orig_pcl.size());
    for (int i = 0; i < orig_pcl.points.size(); i++) {
        V3D pt(orig_pcl.points[i].x, orig_pcl.points[i].y, orig_pcl.points[i].z);
        pt = offset_R_L_I * pt + offset_T_L_I; //lidar_to_body
        bool is_dynamic = false;
        for (auto iter = swarm_in->teammate_tracker.begin(); iter != swarm_in->teammate_tracker.end(); ++iter) {
            V3D dist = pt - state.rot_end.transpose() * (iter->second.get_state_pos() - state.pos_end);
            if (dist.norm() < 0.7) {
                is_dynamic = true;
                continue;
            }
        }

        if (!is_dynamic) {
            for (int j = 0; j < swarm_in->temp_tracker.size(); ++j) {
                V3D dist = pt - state.rot_end.transpose() * (swarm_in->temp_tracker[j].dyn_tracker.get_state_pos() - state.pos_end);
                if (dist.norm() < 0.7)  {
                    is_dynamic = true;
                    continue;
                }
            }
        }

        if (!is_dynamic){
            pcl_out.push_back(orig_pcl.points[i]);
        }
    }
}


void VisualizeRectangle(const ros::Publisher &pub_rect, const V3D &head, const V3D rect_size, const int &rect_id) {
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
            rect_size.z();
    //vis_pos_world是目标物的坐标
    p[0].x = head(0) - width;
    p[0].y = head(1) + length;
    p[0].z = head(2) + 0.3;
    p[1].x = head(0) - width;
    p[1].y = head(1) - length;
    p[1].z = head(2) + 0.3;
    p[2].x = head(0) - width;
    p[2].y = head(1) - length;
    p[2].z = head(2) - hight;
    p[3].x = head(0) - width;
    p[3].y = head(1) + length;
    p[3].z = head(2) - hight;
    p[4].x = head(0) + width;
    p[4].y = head(1) + length;
    p[4].z = head(2) - hight;
    p[5].x = head(0) + width;
    p[5].y = head(1) - length;
    p[5].z = head(2) - hight;
    p[6].x = head(0) + width;
    p[6].y = head(1) - length;
    p[6].z = head(2) + 0.3;
    p[7].x = head(0) + width;
    p[7].y = head(1) + length;
    p[7].z = head(2) + 0.3;
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

void point_downsample_filt_rectangle(PointCloudXYZI &orig_pcl, PointCloudXYZI &pcl_out,
                                     const shared_ptr<Multi_UAV> &swarm_in, const ros::Publisher &pub_rect) {
    V3D rect_size(1.2,1.2,1.5);
    PointCloudXYZI feats_undistort_orig_copy = orig_pcl;

    pcl_out.clear();
    pcl_out.reserve(feats_undistort_orig_copy.size());

    orig_pcl.clear();
    orig_pcl.reserve(feats_undistort_orig_copy.size());

    for (int i = 0; i < feats_undistort_orig_copy.points.size(); i++) {
        V3D pt(feats_undistort_orig_copy.points[i].x, feats_undistort_orig_copy.points[i].y, feats_undistort_orig_copy.points[i].z);
        pt = offset_R_L_I * pt+ offset_T_L_I; //lidar_to_body
        bool is_dynamic = false;
        for (int j = 0; j < swarm_in->temp_tracker.size(); ++j) {
            V3D dist = pt - state.rot_end.transpose() * (swarm_in->temp_tracker[j].dyn_tracker.get_state_pos() - state.pos_end);
            if ( - rect_size.z() < dist.z() && dist.z() < 0.3 && (sqrt(dist.x()*dist.x() + dist.y()*dist.y()) < rect_size.x()/2)) {
                is_dynamic = true;
                continue;
            }
        }

        if (!is_dynamic) {
            for (auto iter = swarm_in->teammate_tracker.begin(); iter != swarm_in->teammate_tracker.end(); ++iter) {
                V3D dist = pt - state.rot_end.transpose() * (iter->second.get_state_pos() - state.pos_end);
                if ( - rect_size.z() < dist.z() && dist.z() < 0.3 && (sqrt(dist.x()*dist.x() + dist.y()*dist.y()) < rect_size.x()/2)) {
                    is_dynamic = true;
                    continue;
                }
            }
        }

        if (!is_dynamic){
            orig_pcl.push_back(feats_undistort_orig_copy.points[i]);
            if(i % point_filter_num == 0)
                pcl_out.push_back(feats_undistort_orig_copy.points[i]);
        }
    }

    if(pub_rect.getNumSubscribers() < 1)
        return;

    visualization_msgs::Marker vis_cluster_delete;
    vis_cluster_delete.header.stamp = ros::Time().fromSec(lidar_end_time);
    vis_cluster_delete.header.frame_id = topic_name_prefix + "world";
    vis_cluster_delete.action = visualization_msgs::Marker::DELETEALL;
    pub_rect.publish(vis_cluster_delete);

    for (int i = 0; i < swarm_in->temp_tracker.size(); ++i) {
        VisualizeRectangle(pub_rect, G_T_I0.block<3,3>(0,0) * swarm_in->temp_tracker[i].dyn_tracker.get_state_pos(),
                            rect_size, swarm_in->temp_tracker[i].id);
    }
    for (auto iter = swarm_in->teammate_tracker.begin(); iter != swarm_in->teammate_tracker.end(); ++iter) {
        VisualizeRectangle(pub_rect, G_T_I0.block<3,3>(0,0) * iter->second.get_state_pos(),
                            rect_size, iter->first + 100);
    }
}


void gt_pos_cbk_sim(const nav_msgs::Odometry::ConstPtr &msg) {
    VD(7) gt;
    gt(6) = msg->header.stamp.toSec();
    gt.block<3, 1>(3, 0) = V3D(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    M3D gt_rot = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).matrix();
    gt.block<3, 1>(0, 0) = RotMtoEuler(gt_rot);
    ground_truth.push_back(gt);
}

void gt_pos_cbk_real(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if(msg->pose.position.x > 9990){
        sig_buffer.notify_all();
        return;
    }
    VD(7) gt;
    gt(6) = msg->header.stamp.toSec();
    gt.block<3, 1>(3, 0) = V3D(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    M3D gt_rot = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                    msg->pose.orientation.y, msg->pose.orientation.z).matrix();
    gt.block<3, 1>(0, 0) = RotMtoEuler(gt_rot);
    mtx_buffer_gt.lock();
    ground_truth.push_back(gt);
    mtx_buffer_gt.unlock();
    sig_buffer.notify_all();
}

void TimerCbk(const ros::TimerEvent &e){
    if(LIO_pose.empty())
        return;
    while ((ground_truth[1](6) <= LIO_pose.front()(3)) && ground_truth.size() > 2) {
        mtx_buffer_gt.lock();
        ground_truth.pop_front();
        mtx_buffer_gt.unlock();
    }
    if(ground_truth[0](6) <= LIO_pose.front()(3) && ground_truth[1](6) > LIO_pose.front()(3)){
        //Linear Interpolate
        double DT = ground_truth[1](6) - ground_truth[0](6);
        double dt = LIO_pose.front()(3) - ground_truth[0](6);
        double s = dt / DT;
        V3D gt_rot_euler = s * ground_truth[1].block<3, 1>(0, 0) + (1 - s) * ground_truth[0].block<3, 1>(0, 0);
        V3D gt_position = s * ground_truth[1].block<3, 1>(3, 0) + (1 - s) * ground_truth[0].block<3, 1>(3, 0);
        fout_pose << setiosflags(ios::fixed) << setprecision(12)  << LIO_pose.front().block<3,1>(0,0).transpose() << " " << gt_position.transpose() << " " << LIO_pose.front()(3) << endl;
        LIO_pose.pop_front();
    }
}

bool get_local_ip(string &local_ip) {
    char ip[16];
    struct ifaddrs *ifAddrStruct;
    void *tmpAddrPtr=NULL;
    getifaddrs(&ifAddrStruct);
    while (ifAddrStruct != NULL) {
        if (ifAddrStruct->ifa_addr->sa_family==AF_INET) {
            tmpAddrPtr = &((struct sockaddr_in *) ifAddrStruct->ifa_addr)->sin_addr;
            inet_ntop(AF_INET, tmpAddrPtr, ip, INET_ADDRSTRLEN);
            if ((ip[0] == '1' && ip[1] == '0' && ip[3] == '0')||
                (ip[8] == '2' && ip[9] == '3' && ip[10] == '4' && ip[12] == '1')) {
                printf("%s IP Address: %s\n", ifAddrStruct->ifa_name, ip);
                local_ip = ip;
                // freeifaddrs(ifAddrStruct);
                return true;
            }
        }
        ifAddrStruct=ifAddrStruct->ifa_next;
    }
    //free ifaddrs
    freeifaddrs(ifAddrStruct);
    return false;
}

void StringIp2CharIp(string &str_ip, uint8_t *ch_ip) {
    std::stringstream s(str_ip);
    int data[4];
    char ch; //to temporarily store the '.'
    s >> data[0] >> ch >> data[1] >> ch >> data[2] >> ch >> data[3];
    for (int i = 0; i < 4; i++) {
        ch_ip[i] = data[i];
    }
}


deque<V3D> uav_traj;
ros::Publisher traj_pub;
void VisualizeTrajectory(const ros::Publisher pub_, deque<V3D> &traj,
                         const V3D position,
                         const string namespace_,
                         const double mkr_size,
                         const double &timestamp,
                         const V3D color
) {
    V3D last_pos, cur_pos;
    visualization_msgs::MarkerArray mrkarr;
    mrkarr.markers.clear();

    traj.push_back(position);
    cout << "traj.size() "<< traj.size() << endl;
    last_pos = traj[0];
    int cnt = 0;
    for (auto cur_pos: traj) {
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
        // Line list is blue
        line_list.color.r = color.x();
        line_list.color.g = color.y();
        line_list.color.b = color.z();
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


void VisualizeTrajectorySphere(const ros::Publisher pub_, deque<V3D> &traj,
                               const V3D position,
                               const string namespace_,
                               const double mkr_size,
                               const double &timestamp,
                               const V3D color
) {
    V3D last_pos, cur_pos;
    visualization_msgs::MarkerArray mrkarr;
    mrkarr.markers.clear();

    traj.push_back(position);
    cout << "traj.size() "<< traj.size() << endl;
    last_pos = traj[0];
    int cnt = 0;
    for (auto cur_pos: traj) {
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
        // Line list is blue
        line_list.color.r = color.x();
        line_list.color.g = color.y();
        line_list.color.b = color.z();
        line_list.color.a = 1;
        line_list.pose.position.x = 0.5 * (cur_pos.x() + last_pos.x());
        line_list.pose.position.y = 0.5 * (cur_pos.y() + last_pos.y());
        line_list.pose.position.z = 0.5 * (cur_pos.z() + last_pos.z());
        mrkarr.markers.push_back(line_list);
        last_pos = cur_pos;
    }
    pub_.publish(mrkarr);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh("~");
    p_imu.reset(new ImuProcess());


    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    nh.param<int>("mapping/max_iteration", NUM_MAX_ITERATIONS, 4);
    nh.param<int>("mapping/point_filter_num", point_filter_num, 2);
    nh.param<string>("map_file_path", map_file_path, "");
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
    nh.param<double>("common/time_offset_lidar_imu", time_offset_lidar_imu, 0.0);
    nh.param<int>("common/drone_id", drone_id, 0);
    nh.param<double>("filter_size_corner", filter_size_corner_min, 0.5);
    nh.param<double>("mapping/filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("mapping/filter_size_map", filter_size_map_min, 0.5);
    nh.param<double>("mapping/cube_side_length", cube_len, 200);
    nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
    nh.param<bool>("mapping/imu_en", imu_en, true);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/grav_cov", grav_cov, 0.001);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 1.0);
    nh.param<int>("preprocess/lidar_type", lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<bool>("preprocess/feature_extract_en", p_pre->feature_enabled, false);
    nh.param<bool>("calibration/cut_frame_en", cut_frame_en, false);
    nh.param<int>("calibration/cut_frame_num", cut_frame_num, 1);
    nh.param<bool>("calibration/accumulate_frame_en", accumulate_frame_en, false);
    nh.param<int>("calibration/original_frequency", original_frequency, 10);
    nh.param<bool>("calibration/gravity_align_enable", gravity_align_en, true);
    nh.param<double>("calibration/gravity_align_time", gravity_align_dur, 50.0);
    nh.param<bool>("publish/path_en", path_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, false);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<double>("pcd_save/pcd_resolution", pcd_resolution, 0.03);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/LI_extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/LI_extrinsic_R", extrinR, vector<double>());
    nh.param<string>("sub_gt_pose_topic", sub_gt_pose_topic, "");

    nh.param<int>("multiuav/actual_uav_num", actual_uav_num, 5);
    nh.param<int>("multiuav/frame_num_in_sliding_window", frame_num_in_sliding_window, original_frequency/10);
    nh.param<double>("multiuav/mutual_observe_noise", mutual_observe_noise_, 0.001);
    nh.param<int>("multiuav/degeneration_thresh", degeneration_thresh, 30);
    nh.param<bool>("imu_propagate/enable", imu_prop_enable, false);
    nh.param<string>("imu_propagate/topic", imu_prop_topic, "lidar_slam/imu_propagate");
    nh.param<bool>("imu_propagate/filter_acc_en", filter_acc_en, false);

    //Automatically Acquire IP
//    string local_ip;
//    if(lidar_type != SIM && get_local_ip(local_ip)){
//        //Set Drone ID
//        uint8_t* ip_c = new uint8_t[4];
//        StringIp2CharIp(local_ip, ip_c);
//        drone_id =  ip_c[3] - 100;
//    }
    printf("Drone ID: %d\n", drone_id);
    cout << "Run Swarm LIO" << endl;
    //Swarm LIO
    shared_ptr<Multi_UAV> swarm(new Multi_UAV(nh, drone_id));

    //Gravity Alignment
    G_T_PX4.setIdentity();
    G_T_I0.setIdentity();
    vector<double> PX4_R_I_deg, PX4_p_I, level_horizon_deg;
    nh.param<vector<double>>("calibration/PX4_R_I", PX4_R_I_deg, vector<double>());
    nh.param<vector<double>>("calibration/PX4_p_I", PX4_p_I, vector<double>());
    V3D PX4_R_I_rad = V3D(PX4_R_I_deg[0], PX4_R_I_deg[1], PX4_R_I_deg[2]); PX4_R_I_rad /= 57.3;
    //Extrinsic from FAST-LIO IMU to PX4
    PX4_T_I.setIdentity();
    PX4_T_I.block<3,3>(0,0) = EulerToRotM(PX4_R_I_rad);
    PX4_T_I.block<3,1>(0,3) = V3D(PX4_p_I[0], PX4_p_I[1], PX4_p_I[2]);
    nh.param<vector<double>>("calibration/level_horizon", level_horizon_deg, vector<double>());
    V3D level_horizon_rad = V3D(level_horizon_deg[0], level_horizon_deg[1], level_horizon_deg[2])/57.3;
    Horizon_T_PX4.setIdentity();
    Horizon_T_PX4.block<3,3>(0,0) = EulerToRotM(level_horizon_rad);
    //yaw angle from IO to PX4 IMU
    Eigen::AngleAxisd PX4_R_I_yaw(AngleAxisd(PX4_R_I_rad(2), Vector3d::UnitZ()));


    topic_name_prefix = "";
    if(lidar_type == SIM)
        topic_name_prefix = "quad" + SetString(drone_id) + "/";

    path.header.stamp = ros::Time().fromSec(lidar_end_time);
    path.header.frame_id = topic_name_prefix + "world";

    offset_R_L_I << MAT_FROM_ARRAY(extrinR);
    offset_T_L_I << VEC_FROM_ARRAY(extrinT);

    /*** variables definition ***/
    VD(DIM_STATE) solution;
    V3D rot_add, T_add;
    StatesGroup state_propagat;


    bool flg_EKF_converged, EKF_stop_flg = 0;

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterPCD.setLeafSize(pcd_resolution, pcd_resolution, pcd_resolution);
    downsample_filter_map.setLeafSize(0.05, 0.05, 0.05);


    p_pre->point_filter_num = point_filter_num;
    p_pre->original_freq = original_frequency;
    p_imu->set_extrinsic(offset_R_L_I, offset_T_L_I);
    p_imu->set_global_extrinsic_cov(Zero3d, Zero3d);
    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_pre->DET_RANGE = DET_RANGE;
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));


    /*** debug record ***/
    ofstream fout_out, fout_global_extrin, fout_solve_time, fout_cpu_memory, fout_cov;
    fout_out.open(DEBUG_FILE_DIR("quad" + SetString(drone_id) + "_state.txt"), ios::out);
    fout_pose.open(DEBUG_FILE_DIR("quad" + SetString(drone_id) + "_pose.txt"), ios::out);
//    fout_global_extrin.open(DEBUG_FILE_DIR("quad" + SetString(drone_id) + "_global_extrinsic.txt"), ios::out);
    fout_solve_time.open(DEBUG_FILE_DIR("quad" + SetString(drone_id) + "_solve_time.txt"), ios::out);
    fout_cpu_memory.open(DEBUG_FILE_DIR("quad" + SetString(drone_id) + "_cpu_memory.txt"), ios::out);
//    fout_cov.open(DEBUG_FILE_DIR("quad" + SetString(drone_id) + "_cov.txt"), ios::out);
    if (fout_out)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;


    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
         nh.subscribe(lid_topic, 200000, standard_pcl_cbk);



    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 20000, imu_cbk);

    ros::Publisher pubCloudRegistered = nh.advertise<sensor_msgs::PointCloud2>
            ("/" + topic_name_prefix + "cloud_registered", 1000);        //去畸变后原始点云在world系下的坐标
    ros::Publisher pubCloudRegisteredSparse = nh.advertise<sensor_msgs::PointCloud2>
            ("/" + topic_name_prefix + "cloud_registered_sparse", 1000);        //去畸变后降采样后的点云在world系下的坐标
    ros::Publisher pubCloudRegisteredBody = nh.advertise<sensor_msgs::PointCloud2>
            ("/" + topic_name_prefix + "cloud_registered_body", 1000);   //去畸变后点云在body系下的坐标
    pub_map = nh.advertise<sensor_msgs::PointCloud2>
            ("/" + topic_name_prefix + "downsampled_map", 1000);   //去畸变后点云在body系下的坐标
    ros::Publisher pubLidarSlamOdom = nh.advertise<nav_msgs::Odometry>
            ("/" + topic_name_prefix + "lidar_slam/odom", 1000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>
            ("/" + topic_name_prefix + "path", 1000);
    pubDrone = nh.advertise<visualization_msgs::Marker>
            ("/" + topic_name_prefix + "drone_id", 1000);
    ros::Publisher pubRectangle = nh.advertise<visualization_msgs::Marker>
            ("/" + topic_name_prefix + "target_rectangle", 1000);
    traj_pub = nh.advertise<visualization_msgs::MarkerArray>("/" + topic_name_prefix + "traj", 100);

    pubImuPropOdom = nh.advertise<nav_msgs::Odometry> ("/" + topic_name_prefix + imu_prop_topic, 1000);

    ros::Timer imu_prop_timer = nh.createTimer(ros::Duration(0.004), imu_prop_callback);

    ros::Timer timer = nh.createTimer(ros::Duration(0.01), TimerCbk);

    pubFilteredImu = nh.advertise<sensor_msgs::Imu>("/" + topic_name_prefix + "imu/data_filtered", 1000);


    ros::Subscriber groundtruth_subscriber;
    //Multi-UAV subscriber & publisher
    if(lidar_type == SIM){
        text_scale = 0.5;
        mesh_scale = 1.2;
        groundtruth_subscriber = nh.subscribe<nav_msgs::Odometry>
                (sub_gt_pose_topic, 10000, gt_pos_cbk_sim);
    }else{
        text_scale = 0.3;
        mesh_scale = 0.8;
        groundtruth_subscriber = nh.subscribe<geometry_msgs::PoseStamped>
                (sub_gt_pose_topic, 10000, gt_pos_cbk_real);
    }
    ground_truth.clear();


    //Clear sliding window
    feats_world_sliding_window.clear();

    ros::Publisher mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    ros::Time main_loop_start_time = ros::Time::now();
    bool print_log{false};

    //异步回调，防止线程占用
    ros::AsyncSpinner spinner(0);
    spinner.start();

    fout_cpu_memory << "lidar_end_time" << "     " << "cpu(%)" <<  "     " << "memory(GB)" << endl;
    fout_solve_time << "lidar_end_time" << "     " << "cur_frame_time(s) " <<  "     " << "ekf_consuming_time(s)" << endl;

    //CPU Status
    int current_pid = CpuMemoryQuery::GetCurrentPid(); // or you can set a outside program pid
    boost::filesystem::create_directories(root_dir + "/PCD");

    while (status) {
        if (flg_exit) break;
        //ros::spinOnce();
        if (sync_packages(Measures)) {
            double start_time = ros::Time::now().toSec();
            TimeConsuming time_all("Total time per scan");
            print_log = false;
            //1s 10次 print log
            if(frame_num % (original_frequency / 10) == 0){
                print_log = true;
                cout << endl << endl << endl << CYAN << " -- [NEW SCAN " << frame_num << ", Drone ID " << drone_id << "]" << RESET << endl;
            }

            //1s一次print cpu status
            if(frame_num % original_frequency == 0){
                float cpu_usage_ratio = CpuMemoryQuery::GetCpuUsageRatio(current_pid);
                float memory_usage = CpuMemoryQuery::GetMemoryUsage(current_pid);
//                cout << MAGENTA;
//                cout << "---------------------------------------" << endl;
//                cout << "    CPU Usage Ratio: " << cpu_usage_ratio * 100 << "%" << endl;
//                cout << "    Memory Usage:    " << memory_usage/1024 << "GB" << endl;
//                cout << "---------------------------------------" <<  RESET <<  endl;
                fout_cpu_memory << setiosflags(ios::fixed) << setprecision(2) << lidar_end_time << "     "
                                << cpu_usage_ratio * 100 << "%        " <<  memory_usage/1024 << "GB" << setprecision(6) << endl;
            }
            if(first_lidar_time < 0)
                first_lidar_time = lidar_end_time;

            if (flg_reset) {
                ROS_WARN("reset when rosbag play back.");
                p_imu->Reset();
                flg_reset = false;
                continue;
            }

            //Gravity Alignment Starts
            if(gravity_align_en){
                if (p_imu->imu_need_init_) {
                    main_loop_start_time = ros::Time::now();
                }
                double wait_duration = (ros::Time::now() - main_loop_start_time).toSec();
                if (!gravity_align_finished && print_log) {
                    cout << BOLDYELLOW << " -- [Initializing Gravity]: "
                         << (wait_duration > gravity_align_dur ? gravity_align_dur : wait_duration) /
                            gravity_align_dur * 100 << "%" << RESET  << endl;
                }
                if (!p_imu->imu_need_init_ && !gravity_align_finished) {
                    if (wait_duration >= gravity_align_dur){
                        gravity_align_finished = true;
                        cout << BOLDGREEN << " -- [Initializing Gravity Finished] " << RESET << endl;
                    }
                    V3D ez(0, 0, -1), gz(state.gravity);
                    // Rotation: FAST-LIO original world frame to gravity-aligned world frame
                    V3D euler_ = RotMtoEuler(Quaterniond::FromTwoVectors(gz, ez).toRotationMatrix());
                    euler_.z() = 0.0;
                    // The first PX4 IMU frame to gravity-aligned world frame
                    G_T_PX4.block<3,3>(0,0) = Quaterniond(PX4_R_I_yaw).toRotationMatrix() * EulerToRotM(euler_) * PX4_T_I.block<3,3>(0,0).transpose();
                    // FAST-LIO original world frame to gravity-aligned world frame
                    G_T_I0 = G_T_PX4 * PX4_T_I;
                    swarm->rot_world_to_gravity = G_T_I0.block<3,3>(0,0);
                    if (print_log)
                        cout << setiosflags(ios::fixed) << setprecision(4) << " -- [Gravity Calibration] " << RotMtoEuler(swarm->rot_world_to_gravity).transpose() * 57.3 << " deg" << endl;
                }
            }

            p_imu->Process(Measures, state, feats_undistort_orig_lidar);
            unbiased_gyr = p_imu->unbiased_gyr;
            //feats_undistort_orig_lidar是L_k系中无畸变的点云坐标 (原始点云数目)
            if (feats_undistort_orig_lidar->empty() || (feats_undistort_orig_lidar == NULL)) {
                ROS_WARN("FAST-LIO not ready, no points stored.");
                continue;
            }

            //清除NaN点
            std::vector<int> indices_all;
            feats_undistort_orig_lidar->is_dense = false; //必须设置为false，才会剔除NaN点
            pcl::removeNaNFromPointCloud(*feats_undistort_orig_lidar, *feats_undistort_orig_lidar, indices_all);


            //Delete Reconnected UAV (of which the swarm start time changed)
            swarm->ResetReconnectedGlobalExtrinsic(state, lidar_end_time);

            //Update Factor Graph
            swarm->UpdateFactorGraph(print_log);
            swarm->UpdateGlobalExtrinsicAndCreateNewTeammateTracker(state, lidar_end_time);
            swarm->state = state;
            state_propagat = state;

            PointCloudXYZI::Ptr feats_merged_body(new PointCloudXYZI());
            //Transform last several frames to current body frame
            if(feats_world_sliding_window.size() >= frame_num_in_sliding_window)
                feats_world_sliding_window.pop_front();

            PointCloudXYZI::Ptr feats_world_temp(new PointCloudXYZI());
            Matrix4d lidar_to_world = Matrix4d::Identity();
            lidar_to_world.block<3, 3>(0, 0) = state_propagat.rot_end * offset_R_L_I;
            lidar_to_world.block<3, 1>(0, 3) = state_propagat.rot_end * offset_T_L_I + state_propagat.pos_end;
            pcl::transformPointCloud(*feats_undistort_orig_lidar, *feats_world_temp, lidar_to_world);
            feats_world_sliding_window.push_back(feats_world_temp);

            PointCloudXYZI::Ptr feats_all_in_window(new PointCloudXYZI());
            for (int i = 0; i < feats_world_sliding_window.size(); ++i) {
                *feats_all_in_window += *feats_world_sliding_window[i];
            }

            Matrix4d world_to_body = Matrix4d::Identity();
            world_to_body.block<3, 3>(0, 0) = state_propagat.rot_end.transpose();
            world_to_body.block<3, 1>(0, 3) = - state_propagat.rot_end.transpose() * state_propagat.pos_end;
            pcl::transformPointCloud(*feats_all_in_window, *feats_merged_body, world_to_body);


            //Predict Temporary Tracker
            for (int i = 0; i < swarm->temp_tracker.size(); ++i) {
                swarm->PredictTemporaryTracker(lidar_end_time, i);
            }

            //Predict Teammate Tracker
            for (auto iter = swarm->teammates.begin(); iter != swarm->teammates.end(); ++iter) {
                int id = iter->first;
                //Copy teammate_state from callback function
                swarm->CopyTeammateState(iter->second);
                //If lost communication for too long time with this teammate
                swarm->VisualizeDisconnectedTracker(lidar_end_time, id, iter->second);

                if (swarm->IsDurationShort(lidar_end_time, iter->second, id)) {
                    //propagate teammate's state to lidar_end_time.
                    swarm->PropagateTeammateState(lidar_end_time, iter->second);
                    //EKF tracker prediction
                    swarm->PredictTracker(lidar_end_time, id);
                }
            }

            //Cluster Extraction
            TimeConsuming time_cluster1("Clustering1");
            swarm->ClusterExtractPredictRegion(lidar_end_time, feats_merged_body);
            double cluster_time1 = time_cluster1.stop() * 1000;

            //Detect and segment highly-reflective points
            TimeConsuming time_cluster2("Clustering2");
            swarm->ClusterExtractHighIntensity(lidar_end_time, feats_merged_body);
            double cluster_time2 = time_cluster2.stop() * 1000;

            //Update Teammate Tracker
            TimeConsuming time_tracking("Tracking");
            for (auto iter = swarm->teammates.begin(); iter != swarm->teammates.end(); ++iter) {
                int id = iter->first;
                //Multi-UAV
                if (swarm->IsDurationShort(lidar_end_time, iter->second, id)) {
                    //Chack clusters' validation, and update observe status
                    swarm->CheckClusterValidation(id, iter->second);

                    //Observe teammate
                    if (swarm->IsObserveTeammate(iter->second)) {
                        swarm->UpdateTracker(lidar_end_time, id, iter->second, true, print_log);
                    }

                    //Update Tracker Using Teammate Odom.
                    swarm->UpdateTracker(lidar_end_time, id, iter->second, false, print_log);

                    //Visualize Teammate Trackers and Predicted Regions
                    swarm->VisualizeMeshUAV(lidar_end_time, id, iter->second);
                    swarm->VisualizePredictRegion(lidar_end_time,id);
                    swarm->VisualizeTeammateTracker(lidar_end_time, id);

                }
            }

            //Update Temporary Trackers for Potential Teammates
            for (int i = 0; i < swarm->temp_tracker.size(); ++i) {

                //Check validation of clusters in temporary trackers' predict region, to find out if there is a valid measurement
                swarm->CheckTempClusterValidation(i);

                //Update Temporary Tracker and push_back dyn_positions for trajectory matching
                swarm->UpdateTemporaryTracker(lidar_end_time, i, print_log);

                //Try Trajectory Matching and Create New Teammate Tracker
                bool find_new_teammate = false;
                if (swarm->temp_tracker[i].dyn_pos_time.size() >= 0.1 * swarm->pos_num_in_traj)
                    find_new_teammate = swarm->CreateTeammateTracker(lidar_end_time, i, state, state_propagat, print_log);

                if (find_new_teammate)
                    i--;
                else
                    i -= swarm->DeleteInvalidTemporaryTracker(lidar_end_time, i);
            }


            //If there is NEW highly reflective object
            swarm->CreateTempTrackerByHighIntensity(lidar_end_time);

            //Visualize Temporary Tracker
            swarm->VisualizeTempTracker(lidar_end_time);
            double tracking_time = time_tracking.stop() * 1000;



            point_filter_teammate(*feats_undistort_orig_lidar, *feats_body_dense_filter_dynamic, swarm);
            point_downsample(*feats_body_dense_filter_dynamic, *feats_undistort_lidar);
//            point_downsample_filt_rectangle(*feats_undistort_orig_lidar, *feats_undistort_lidar, swarm, pubRectangle);


            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort_lidar);// L_k系的无畸变点云坐标
            downSizeFilterSurf.filter(*feats_down_lidar);         //降采样后L_k系的无畸变点云坐标

            if(print_log){
//                cout << " -- [feats_undistort_lidar size]   " << feats_undistort_lidar->points.size() << endl;
                cout  <<  " -- [Total Point Size for ICP]  " << GREEN << feats_down_lidar->points.size() << RESET << endl;
                cout  <<  " -- [Cluster Predict Region Time] " << YELLOW << cluster_time1 << " ms" << RESET<< endl;
                cout  <<  " -- [Cluster High Itensity Time] " << YELLOW << cluster_time2 << " ms" << RESET<< endl;
                cout  <<  " -- [Tracking Time] " << YELLOW << tracking_time << " ms" << RESET<< endl;
            }

            feats_down_size = feats_down_lidar->points.size();
            /*** initialize the map kdtree ***/
            if (ikdtree.Root_Node == nullptr) {
                if (feats_down_size > 5) {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    pcl::transformPointCloud(*feats_undistort_lidar, *feats_down_world, lidar_to_world);
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }

            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();


            /*** ICP and iterated Kalman filter update ***/
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);


            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int rematch_num = 0;
            bool nearest_search_en = true;

            /*** iterated state estimation ***/
            vector<int> teammateID_with_observation;
            vector<int> teammateID_with_active_observation;
            vector<int> teammateID_with_passive_observation;
            teammateID_with_observation.clear();
            teammateID_with_active_observation.clear();
            teammateID_with_passive_observation.clear();


            for (auto iter = swarm->teammates.begin(); iter != swarm->teammates.end(); ++iter){
                int id = iter->first;
                //If the teammate drone is degenerated, do not use the mutual observation with it
                // && !iter->second.teammate_state.degenerated
                if (swarm->IsDurationShort(lidar_end_time, iter->second, id)){
                    bool active_observation_ = false;
                    if (swarm->IsObserveTeammate(iter->second)) {
                        active_observation_ = true;
                        teammateID_with_active_observation.push_back(id);
                        teammateID_with_observation.push_back(id);
                    }

                    if (swarm->IsObservedByTeammate(iter->second) && state.global_extrinsic_trans[id].norm() > 0.001){
                        teammateID_with_passive_observation.push_back(id);
                        if(!active_observation_)
                            teammateID_with_observation.push_back(id);
                    }
                }
            }
            sort(teammateID_with_observation.begin(), teammateID_with_observation.end());
            int teammate_num_with_observation = teammateID_with_observation.size();
            if(print_log){
                cout << BOLDGREEN << " -- [Observation] ";
                for (int i = 0; i < teammate_num_with_observation; ++i) {
                    cout << teammateID_with_observation[i] << " ";
                }
                cout  << RESET << endl;
            }

            //ESIKF迭代开始
            bool degeneration_detected = false;
            mutual_observe_noise = mutual_observe_noise_;
            TimeConsuming time_ekf("EKF time per scan");
            for (iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++) {
                laserCloudOri->clear();
                corr_normvect->clear();

                /** closest surface search and residual computation **/
        #ifdef MP_EN
                omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
        #endif
                for (int i = 0; i < feats_down_size; i++) {
                    PointType &point_lidar = feats_down_lidar->points[i];  //L_k系
                    PointType &point_world = feats_down_world->points[i];  //World系
                    V3D p_body(point_lidar.x, point_lidar.y, point_lidar.z);
                    /// transform to world frame
                    pointBodyToWorld(&point_lidar, &point_world); // L_k系转换到World系 (中间还有一步雷达IMU外参的转换)
                    vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
                    auto &points_near = Nearest_Points[i];
                    uint8_t search_flag = 0;
                    if (nearest_search_en) {
                        /** Find the closest surfaces in the map **/
                        ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis,
                                               2.236);
                        if (points_near.size() < NUM_MATCH_POINTS)
                            point_selected_surf[i] = false;
                        else
                            point_selected_surf[i] = pointSearchSqDis[NUM_MATCH_POINTS - 1] <= 5;

                    }

                    res_last[i] = -1000.0f;

                    if (!point_selected_surf[i] || points_near.size() < NUM_MATCH_POINTS) {
                        point_selected_surf[i] = false;
                        continue;
                    }

                    point_selected_surf[i] = false;
                    VD(4) pabcd;
                    pabcd.setZero();
                    if (esti_plane(pabcd, points_near, 0.1)) //(planeValid)
                    {
                        float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                        float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                        if (s > 0.9) {
                            point_selected_surf[i] = true;
                            //每一个点对应平面的法向量
                            normvec->points[i].x = pabcd(0);
                            normvec->points[i].y = pabcd(1);
                            normvec->points[i].z = pabcd(2);
                            normvec->points[i].intensity = pd2;
                            res_last[i] = abs(pd2);
                        }
                    }
                }


                //Degeneration Detection
                M3D DegenerationDetectionMat = M3D::Zero();
                effect_feat_num = 0;
                for (int i = 0; i < feats_down_size; i++) {
                    if (point_selected_surf[i]) {
                        laserCloudOri->points[effect_feat_num] = feats_down_lidar->points[i];
                        corr_normvect->points[effect_feat_num] = normvec->points[i];
                        effect_feat_num++;

                        //Degeneration Detection
                        V3D normal_vector(normvec->points[i].x, normvec->points[i].y, normvec->points[i].z);
                        DegenerationDetectionMat += normal_vector * normal_vector.transpose();
                    }
                }

                JacobiSVD<MatrixXd> svd(DegenerationDetectionMat, Eigen::ComputeThinU | Eigen::ComputeThinV);
                double smallest_singular_value = svd.singularValues()(2);
//                cout << svd.singularValues().transpose() << endl;
                if (smallest_singular_value < degeneration_thresh) {
                    cout << BOLDYELLOW << "LiDAR Degeneration Detected!" << RESET << endl;
                    mutual_observe_noise = 0.00005;
                    degeneration_detected = true;
                }


                /*** Computation of Measurement Jacobian matrix H and measurents vector ***/
                vector<Triplet<double>> tripletsHsub;
                vector<Triplet<double>> tripletsHsubT;
                vector<Triplet<double>> triplets_meas_vec;

                for (int i = 0; i < effect_feat_num; i++) {
                    const PointType &laser_p = laserCloudOri->points[i];
                    V3D point_this_L(laser_p.x, laser_p.y, laser_p.z);
                    V3D point_this = offset_R_L_I * point_this_L + offset_T_L_I;   //L_k系下的点投影到I_k坐标系
                    M3D point_crossmat;
                    point_crossmat << SKEW_SYM_MATRX(point_this);

                    /*** get the normal vector of closest surface/corner ***/
                    const PointType &norm_p = corr_normvect->points[i];
                    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
                    laserCloudOri->points[i].intensity = sqrt(1000);

                    /*** calculate the Measurement Jacobian matrix H ***/
                    M3D point_this_L_cross;
                    point_this_L_cross << SKEW_SYM_MATRX(point_this_L);
                    V3D A(point_crossmat * state.rot_end.transpose() * norm_vec);

                    VectorXd Hrow(6);
                    Hrow << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;
                    for (int j = 0; j < 6; ++j) {
                        tripletsHsub.push_back(Triplet<double>(i, j, Hrow(j)));
                        tripletsHsubT.push_back(Triplet<double>(j, i, Hrow(j) * 1000));
                    }
                    /*** Measurement: distance to the closest plane ***/
                    triplets_meas_vec.push_back(Triplet<double>(i, 0, -norm_p.intensity));
                }


                for (int index = 0; index < teammate_num_with_observation; ++index) {
                    int id = teammateID_with_observation[index];
                    auto iter_teammate = swarm->teammates.find(id);
                    int start_col = 18 + index * 6;
                    auto iter_id = find(teammateID_with_active_observation.begin(), teammateID_with_active_observation.end(), id);
                    if(iter_id != teammateID_with_active_observation.end()){
                        //Active Observation: 自身观察到队友
                        V3D active_observation_meas = Zero3d;
                        int start_row = effect_feat_num + index * 6;
                        //Jacobian matrix of Active Observation Measurements
                        MD(3, 12) Jacob_case1 = swarm->JacobianActiveObserve(active_observation_meas,
                                                                             id, iter_teammate->second);


                        M3D R_ao_ij = swarm->GetActiveMutualObserveMeasurementNoise(degeneration_detected, state, id, iter_teammate->second, mutual_observe_noise);
                        MD(12, 3) HsubT_R_inv_;
                        for (int i = 0; i < 4; ++i) {
                            HsubT_R_inv_.block<3,3>(3 * i,0) = Jacob_case1.block<3,3>(0,3 * i).transpose() * R_ao_ij.inverse();
                        }

                        for (int i = 0; i < 3; ++i) {
                            for (int j = 0; j < 3; ++j) {
                                //Jacobian of Attitude
                                tripletsHsub.push_back(Triplet<double>
                                                               (start_row + i, j, Jacob_case1(i, j)));
                                //Jacobian of Position
                                tripletsHsub.push_back(Triplet<double>
                                                               (start_row + i, j + 3, Jacob_case1(i, j + 3)));
                                //Jacobian of Attitude
                                tripletsHsubT.push_back(Triplet<double>
                                                                (j, start_row + i, HsubT_R_inv_(j,i)));
                                //Jacobian of Position
                                tripletsHsubT.push_back(Triplet<double>
                                                                ( j + 3,start_row + i, HsubT_R_inv_(j + 3,i)));

                                if(!degeneration_detected){
                                    //Jacobian of Global Extrinsic Rotation
                                    tripletsHsub.push_back(Triplet<double>
                                                                   (start_row + i, start_col + j, Jacob_case1(i, j + 6)));
                                    //Jacobian of Global Extrinsic Translation
                                    tripletsHsub.push_back(Triplet<double>
                                                                   (start_row + i, start_col + j + 3, Jacob_case1(i, j + 9)));
                                    //Jacobian of Global Extrinsic Rotation
                                    tripletsHsubT.push_back(Triplet<double>
                                                                    ( start_col + j, start_row + i, HsubT_R_inv_(j + 6,i)));
                                    //Jacobian of Global Extrinsic Translation
                                    tripletsHsubT.push_back(Triplet<double>
                                                                    ( start_col + j + 3, start_row + i, HsubT_R_inv_(j + 9,i)));
                                }
                            }
                        }

                        for (int k = 0; k < 3; ++k) {
                            triplets_meas_vec.push_back(Triplet<double>(start_row + k, 0, - active_observation_meas(k)));
                        }
                    }

                    //Passive Observation: 队友观察到自己
                    iter_id = find(teammateID_with_passive_observation.begin(), teammateID_with_passive_observation.end(), id);
                    if(iter_id != teammateID_with_passive_observation.end()){
                        V3D passive_observation_meas = Zero3d;
                        int start_row = effect_feat_num + index * 6 + 3;
                        //Jacobian matrix of Passive Observation Measurements
                        MD(3, 12) Jacob_case2 = swarm->JacobianPassiveObserve(passive_observation_meas,
                                                                              id, iter_teammate->second, lidar_end_time);
                        M3D R_po_ij = swarm->GetPassiveMutualObserveMeasurementNoise(degeneration_detected, state, id, iter_teammate->second,lidar_end_time,mutual_observe_noise);
                        MD(12, 3) HsubT_R_inv_;
                        for (int i = 0; i < 4; ++i) {
                            HsubT_R_inv_.block<3,3>(3 * i,0) = Jacob_case2.block<3,3>(0,3 * i).transpose() * R_po_ij.inverse();
                        }
                        for (int i = 0; i < 3; ++i) {
                            for (int j = 0; j < 3; ++j) {
                                //Jacobian of Velocity
                                tripletsHsub.push_back(Triplet<double>
                                                               (start_row + i, j + 6, Jacob_case2(i, j)));
                                tripletsHsubT.push_back(Triplet<double>
                                                                (j + 6, start_row + i,HsubT_R_inv_(j,i)));
                                //Jacobian of Position
                                tripletsHsub.push_back(Triplet<double>
                                                               (start_row + i, j + 3, Jacob_case2(i, j + 3)));
                                tripletsHsubT.push_back(Triplet<double>
                                                                (j + 3, start_row + i, HsubT_R_inv_(j + 3,i)));

                                if(!degeneration_detected){
                                    //Jacobian of Global Extrinsic Rotation
                                    tripletsHsub.push_back(Triplet<double>
                                                                   (start_row + i, start_col + j,Jacob_case2(i, j + 6)));
                                    tripletsHsubT.push_back(Triplet<double>
                                                                    (start_col + j, start_row + i, HsubT_R_inv_(j + 6,i)));
                                    //Jacobian of Global Extrinsic Translation
                                    tripletsHsub.push_back(Triplet<double>
                                                                   (start_row + i, start_col + j + 3,Jacob_case2(i, j + 9)));
                                    tripletsHsubT.push_back(Triplet<double>
                                                                    (start_col + j + 3, start_row + i,HsubT_R_inv_(j + 9,i)));
                                }
                            }
                        }
                        for (int k = 0; k < 3; ++k) {
                            triplets_meas_vec.push_back(Triplet<double>(start_row + k, 0, - passive_observation_meas(k)));
                        }
                    }
                }


                effect_meas_num = effect_feat_num + 6 * teammate_num_with_observation;
                int NEW_DIM_STATE = 18 + 6 * teammate_num_with_observation;
                if(degeneration_detected)
                    NEW_DIM_STATE = 18;

                SparseMatrix<double> Hsub, Hsub_T_R_inv, meas_vec, H_T_H;
                Hsub.resize(effect_meas_num, NEW_DIM_STATE);
                Hsub_T_R_inv.resize(NEW_DIM_STATE, effect_meas_num);
                meas_vec.resize(effect_meas_num, 1);
                H_T_H.resize(NEW_DIM_STATE, NEW_DIM_STATE);
                Hsub.setZero();
                Hsub_T_R_inv.setZero();
                meas_vec.setZero();
                H_T_H.setZero();

                Hsub.setFromTriplets(tripletsHsub.begin(), tripletsHsub.end());
                Hsub_T_R_inv.setFromTriplets(tripletsHsubT.begin(), tripletsHsubT.end());
                meas_vec.setFromTriplets(triplets_meas_vec.begin(), triplets_meas_vec.end());

                SparseMatrix<double> K;
                K.resize(NEW_DIM_STATE, effect_meas_num);
                K.setZero();

                EKF_stop_flg = false;
                flg_EKF_converged = false;

                /*** Iterative Kalman Filter Update ***/
                H_T_H = Hsub_T_R_inv * Hsub;

                //Calculation of (H_T_H+P).inverse(): Sparse LU composition


                //Marginalize Covariance
                MatrixXd state_cov;
                state_cov.resize(NEW_DIM_STATE, NEW_DIM_STATE);
                state_cov.setZero();
                state_cov.block<18,18>(0,0) = state.cov.block<18,18>(0,0);
                if(!degeneration_detected){
                    for (int i = 0; i < teammate_num_with_observation; ++i) {
                        int id1 = teammateID_with_observation[i];
                        state_cov.block<18,6>(0,18 + i * 6) = state.cov.block<18,6>(0,18 + id1 * 6);
                        state_cov.block<6,18>(18 + i * 6,0) = state.cov.block<6,18>(18 + id1 * 6,0);
                        state_cov.block<6,6>(18 + i * 6,18 + i * 6) = state.cov.block<6,6>(18 + id1 * 6,18 + id1 * 6);
                        for (int j = i + 1; j < teammate_num_with_observation; ++j) {
                            int id2 = teammateID_with_observation[j];
                            state_cov.block<6,6>(18 + i * 6,18 + j * 6) = state.cov.block<6,6>(18 + id1 * 6,18 + id2 * 6);
                            state_cov.block<6,6>(18 + j * 6,18 + i * 6) = state.cov.block<6,6>(18 + id2 * 6,18 + id1 * 6);
                        }
                    }
                }

                SparseMatrix<double> I_mat;
                I_mat.resize(NEW_DIM_STATE, NEW_DIM_STATE);
                I_mat.setIdentity();
                SparseLU<SparseMatrix<double>> solver2;
                solver2.compute(H_T_H + state_cov.inverse());
                auto K1 = solver2.solve(I_mat);

                K = K1 * Hsub_T_R_inv;

                auto vec = state_propagat - state;
                MatrixXd vec_marginalized;
                vec_marginalized.resize(NEW_DIM_STATE, 1);
                vec_marginalized.setZero();
                vec_marginalized.block<18,1>(0,0) = vec.block<18,1>(0,0);
                if(!degeneration_detected){
                    for (int i = 0; i < teammate_num_with_observation; ++i) {
                        int id = teammateID_with_observation[i];
                        vec_marginalized.block<6,1>(18 + i * 6, 0) = vec.block<6,1>(18 + id * 6, 0);
                    }
                }

                MatrixXd solution_marginalized;
                solution_marginalized.resize(NEW_DIM_STATE, 1);
                solution_marginalized.setZero();
                solution_marginalized = K * meas_vec + vec_marginalized - K * Hsub * vec_marginalized;

                solution.setZero();
                solution.block<18,1>(0,0) = solution_marginalized.block<18,1>(0,0);
                if(!degeneration_detected){
                    for (int i = 0; i < teammate_num_with_observation; ++i) {
                        int id = teammateID_with_observation[i];
                        solution.block<6,1>(18 + id * 6, 0) = solution_marginalized.block<6,1>(18 + i * 6, 0);
                    }
                }

                //state update
                state += solution;
                swarm->state = state;
                rot_add = solution.block<3, 1>(0, 0);
                T_add = solution.block<3, 1>(3, 0);

                if ((rot_add.norm() * 57.3 < 0.01) && (T_add.norm() * 100 < 0.015))
                    flg_EKF_converged = true;

                euler_cur = RotMtoEuler(state.rot_end);

                /*** Rematch Judgement ***/
                nearest_search_en = false;
                if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2)))) {
                    nearest_search_en = true;
                    rematch_num++;
                }

                /*** Convergence Judgements and Covariance Update ***/
                if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1))) {
                    if (flg_EKF_inited) {
                        /*** Covariance Update ***/
                        SparseMatrix<double> I_STATE;
                        I_STATE.resize(NEW_DIM_STATE, NEW_DIM_STATE);
                        I_STATE.setIdentity();
                        state_cov = (I_STATE - K * Hsub) * state_cov;

                        //Reinitialize Covariance: Clear the covariance between un-updated states and updated states
                        MatrixXd cov_temp = state.cov;
                        state.cov.setZero();
                        for (int i = 0; i < MAX_UAV_NUM; ++i) {
                            state.cov.block<6,6>(18 + 6 * i, 18 + 6 * i) = cov_temp.block<6,6>(18 + 6 * i, 18 + 6 * i);
                        }

                        //Update Full state.cov
                        state.cov.block<18,18>(0,0) = state_cov.block<18,18>(0,0);
                        if(!degeneration_detected){
                            for (int i = 0; i < teammate_num_with_observation; ++i) {
                                int id1 = teammateID_with_observation[i];
                                state.cov.block<18,6>(0,18 + id1 * 6) = state_cov.block<18,6>(0,18 + i * 6);
                                state.cov.block<6,18>(18 + id1 * 6,0) = state_cov.block<6,18>(18 + i * 6, 0);
                                state.cov.block<6,6>(18 + id1 * 6,18 + id1 * 6) = state_cov.block<6,6>(18 + i * 6,18 + i * 6);
                                for (int j = i + 1; j < teammate_num_with_observation; ++j) {
                                    int id2 = teammateID_with_observation[j];
                                    state.cov.block<6,6>(18 + id1 * 6,18 + id2 * 6) = state_cov.block<6,6>(18 + i * 6,18 + j * 6);
                                    state.cov.block<6,6>(18 + id2 * 6,18 + id1 * 6) = state_cov.block<6,6>(18 + j * 6,18 + i * 6);
                                }
                            }
                        }
                        swarm->state.cov = state.cov;
                        swarm->degenerated = degeneration_detected;
                        total_distance += (state.pos_end - position_last).norm();
                        position_last = state.pos_end;
                        geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                (euler_cur(0), euler_cur(1), euler_cur(2));
                    }
                    EKF_stop_flg = true;
                }
                if (EKF_stop_flg) break;
            }
            double ekf_consuming_time = time_ekf.stop();


            //Update lastest_ekf_state
            ekf_finish_once = true;
            latest_ekf_state = state;
            latest_ekf_time = lidar_end_time;
            state_update_flg = true;

            /******* Publish Odometry *******/
            publish_odometry(pubLidarSlamOdom);

            /******* Multi-UAV Publish quadstate *******/
            //高频率发布自身odom
            swarm->PublishQuadstate(p_imu->unbiased_gyr, lidar_end_time, first_lidar_time);

            if(frame_num % (original_frequency/10) == 0){
                //10Hz发布世界系外参和队友odom
                swarm->PublishGlobalExtrinsic(lidar_end_time);
                swarm->PublishConnectedTeammateList(lidar_end_time);
                //10Hz更新可视化
                VisualizeDrone(pubDrone);
                if(lidar_type != SIM)
                    swarm->PublishTeammateOdom(lidar_end_time);

            }
            publish_mavros(mavros_pose_publisher);

//            VisualizeTrajectory(traj_pub, uav_traj, V3D(odomAftMapped.pose.pose.position.x, odomAftMapped.pose.pose.position.y, odomAftMapped.pose.pose.position.z),
//                                "UAV" + SetString(drone_id), 0.1, lidar_end_time, V3D(1,1,1));


            //Reset Teammate State
            swarm->ResetTeammateState();

            /*** add the points to map kdtree ***/

            map_incremental();
            kdtree_size_end = ikdtree.size(); //ikd-tree地图中的点数

            
            /******* Publish points *******/
            if (scan_pub_en || pcd_save_en) publish_frame_world(pubCloudRegistered, pubCloudRegisteredSparse);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubCloudRegisteredBody);
            if (path_en) publish_path(pubPath);


            /*** Debug variables Logging ***/
            frame_num++;
            Matrix<double, 4, 1> lio_pose;
            lio_pose.block<3,1>(0,0) = state.pos_end;
            lio_pose(3) = lidar_end_time;
            LIO_pose.push_back(lio_pose);

            double cur_frame_time = time_all.stop() * 1000 + ave_pre_processing_time;
            fout_solve_time << setiosflags(ios::fixed) << setprecision(6)  << lidar_end_time << "    " << cur_frame_time/1000 << "             " << ekf_consuming_time << endl;
            ave_frame_time += (cur_frame_time - ave_frame_time) / frame_num;
            if(print_log){
                cout << " -- [Current Frame Time] " << YELLOW << cur_frame_time << RESET << " ms" << endl;
                cout << " -- [Average Frame Time] " << YELLOW << ave_frame_time << RESET << " ms" << endl;
            }
            fout_out  << setiosflags(ios::fixed) << setprecision(6) << lidar_end_time << " " << euler_cur.transpose() * 57.3 << " " << state.pos_end.transpose()
                      << " " << RotMtoEuler(imu_propagate.rot_end).transpose() * 57.3 << " " << imu_propagate.pos_end.transpose() << endl;
        }
        status = ros::ok();
        rate.sleep();
    }
    cout << "Exit the process." << endl;
    return 0;
}