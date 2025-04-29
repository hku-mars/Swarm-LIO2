//
// Created by fangcheng on 2023/7/16.
//

#ifndef MULTIUAV_H
#define MULTIUAV_H

#include <omp.h>
#include <unistd.h>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <swarm_msgs/QuadStatePub.h>
#include <swarm_msgs/ObserveTeammate.h>
#include <swarm_msgs/GlobalExtrinsicStatus.h>
#include <swarm_msgs/GlobalExtrinsic.h>
#include <swarm_msgs/ConnectedTeammateList.h>
#include <common_lib.h>
#include <pcl/filters/filter.h>
#include "esikf_tracker.hpp"
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include "ExtrinsicInfection.hpp"
#include <algorithm>
#include <unordered_map>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <std_msgs/Int8.h>
#include "FEC.h"

typedef unordered_map<int, ESIKF> id_esikf_map;
typedef id_esikf_map::value_type id2ekf;

using namespace std;
using namespace Eigen;

class Multi_UAV {
public:
    template<class T>
    string SetString(T &param_in);

    Multi_UAV(const ros::NodeHandle &nh, const int & drone_id_);

    ~Multi_UAV();

    struct QuadStateSub {
        QuadStateSub() {
            this->sub_time = 0.0;
            this->swarmlio_start_time = 0.0;
            this->rot_cov.setZero();
            this->pos_cov.setZero();
            this->rot = M3D::Identity();
            this->pos = Zero3d;
            this->gyr = Zero3d;
            this->vel = Zero3d;
            this->rot_end = M3D::Identity();
            this->pos_end = Zero3d;
            this->my_pos_in_teammate = Zero3d;
            this->degenerated = false;
            this->is_observed = false;
        };

        QuadStateSub(const QuadStateSub &b) {
            this->sub_time = b.sub_time;
            this->swarmlio_start_time = b.swarmlio_start_time;
            this->rot_cov = b.rot_cov;
            this->pos_cov = b.pos_cov;
            this->rot = b.rot;
            this->pos = b.pos;
            this->gyr = b.gyr;
            this->vel = b.vel;
            this->rot_end = b.rot_end;
            this->pos_end = b.pos_end;
            this->my_pos_in_teammate = b.my_pos_in_teammate;
            this->degenerated = b.degenerated;
            this->is_observed = b.is_observed;
        };

        QuadStateSub& operator=(const QuadStateSub& b){
            this->sub_time = b.sub_time;
            this->swarmlio_start_time = b.swarmlio_start_time;
            this->rot_cov = b.rot_cov;
            this->pos_cov = b.pos_cov;
            this->rot = b.rot;
            this->pos = b.pos;
            this->gyr = b.gyr;
            this->vel = b.vel;
            this->rot_end = b.rot_end;
            this->pos_end = b.pos_end;
            this->my_pos_in_teammate = b.my_pos_in_teammate;
            this->degenerated = b.degenerated;
            this->is_observed = b.is_observed;
            return *this;
        };

        double sub_time; //timestamp of the received quadrotor states
        double swarmlio_start_time; //swarm start time of the received quadrotor
        M3D rot_cov; //rotation covariance of the received quadrotor
        M3D pos_cov; //position covariance of the received quadrotor
        M3D rot;         //attitude of the received quadrotor at time sub_time
        V3D pos;         //position of the received quadrotor at time sub_time
        V3D gyr;         //unbiased angular velocity of the received quadrotor at time sub_time in its body frame
        V3D vel;         //linear velocity of the received quadrotor at time sub_time in its world frame
        M3D rot_end;     //attitude of the received quadrotor at time t_k (end of the frame)
        V3D pos_end;     //position of the received quadrotor at time t_k (end of the frame)
        V3D my_pos_in_teammate; //relative position (me to my teammate) observed and sent by my teammate
        bool degenerated; //whether the drone is degenerated
        bool is_observed;
    };

    struct TemporaryTracker {
        TemporaryTracker(const ESIKF &tracker, const double &timestamp, const int &id) {
            this->dyn_tracker = tracker;
            Vector4d pos_and_time;
            pos_and_time.block<3, 1>(0, 0) = this->dyn_tracker.get_state_pos();
            pos_and_time(3) = timestamp;
            this->dyn_pos_time.push_back(pos_and_time);
            this->exist_meas = false;
            this->meas_of_tracker = Zero3d;
            this->create_time = timestamp;
            this->id = id;
        };

        TemporaryTracker(const TemporaryTracker &b) {
            this->dyn_tracker = b.dyn_tracker;
            this->dyn_pos_time = b.dyn_pos_time;
            this->exist_meas = b.exist_meas;
            this->meas_of_tracker = b.meas_of_tracker;
            this->create_time = b.create_time;
            this->id = b.id;
        };
        ESIKF dyn_tracker;
        deque<Vector4d> dyn_pos_time;
        bool exist_meas;
        V3D meas_of_tracker;
        double create_time;
        int id;
    };

    struct Cluster {
        Cluster(const V3D &pos, const bool &is_high_inten, const double &max_dist) {
            this->pos_in_body = pos;
            this->is_high_intensity = is_high_inten;
            this->max_dist = max_dist;
        };

        Cluster(const Cluster &b) {
            this->pos_in_body = b.pos_in_body;
            this->is_high_intensity = b.is_high_intensity;
            this->max_dist = b.max_dist;
        };
        V3D pos_in_body;
        bool is_high_intensity;
        double max_dist;
    };

    struct Teammate {
        Teammate() {
            teammate_pos_in_body = Zero3d;
            is_observe_teammate = false;
            last_connect_time = 0.0;
            first_connect_time = - 1.0;
            teammate_odom_time.clear();
            total_dist = 0.0;
            last_position = Zero3d;
            world_to_gravity_deg = Zero3d;
        };
        Teammate(const Teammate &b) {
            this->teammate_state = b.teammate_state;
            this->teammate_state_temp = b.teammate_state_temp;
            this->teammate_pos_in_body = b.teammate_pos_in_body;
            this->is_observe_teammate = b.is_observe_teammate;
            this->last_connect_time = b.last_connect_time;
            this->first_connect_time = b.first_connect_time;
            this->teammate_odom_time = b.teammate_odom_time;
            this->total_dist = b.total_dist;
            this->last_position = b.last_position;
            this->world_to_gravity_deg = b.world_to_gravity_deg;
        };
        QuadStateSub teammate_state;
        QuadStateSub teammate_state_temp;
        V3D teammate_pos_in_body;
        bool is_observe_teammate;
        double last_connect_time;
        double first_connect_time;
        deque<Vector4d> teammate_odom_time;
        double total_dist;
        V3D last_position;
        V3D world_to_gravity_deg;
    };

    void BuildMatrixWithUpperTriangular(const VD(12) &vec, M3D &rot_cov, M3D &pos_cov);

    void QuadstateCbk(const swarm_msgs::QuadStatePub::ConstPtr &msg);

    void GlobalExtrinsicCbk(const swarm_msgs::GlobalExtrinsicStatus::ConstPtr &msg);


    void ResetReconnectedGlobalExtrinsic(StatesGroup &state_in, const double &lidar_end_time);

    void UpdateFactorGraph(const bool &print_log);

    void UpdateGlobalExtrinsicAndCreateNewTeammateTracker(StatesGroup &state_in, const double &lidar_end_time);

    void CopyTeammateState(Teammate &teammate);

    //Reset each teammate's is_observed flag
    void ResetTeammateState();

    //可以从drone_id知道飞机编号
    void PublishQuadstate(const V3D &unbiased_gyr, const double &lidar_end_time, const double &first_lidar_time);

    void PublishGlobalExtrinsic(const double &lidar_end_time);


    template<class T>
    bool LoadParam(string param_name, T &param_value, T default_value);

    template<class T>
    bool LoadParam(string param_name, vector<T> &param_value, vector<T> default_value);

    template<typename T>
    void SetPosestamp(T &out);


    //本机是否被队友(具体编号)观察到
    bool IsObservedByTeammate(const Teammate &teammate);

    bool IsObserveTeammate(const Teammate &teammate);

    //队友发来的信息是否太陈旧，若是，则不满足匀速模型，无法使用互定位信息
    bool IsDurationShort(const double &lidar_end_time, Teammate &teammate, const int &id);

    //Propagate teammate's pose in its own global frame at t_k.
    void PropagateTeammateState(const double &lidar_end_time, Teammate &teammate);


    Matrix<double, (3), (12)>
    JacobianActiveObserve(V3D &active_observation_meas, const int &id, const Teammate &teammate);


    Matrix<double, (3), (12)>
    JacobComputeCase2(V3D &passive_observation_meas, const int &id, const Teammate &teammate);


    Matrix<double, (3), (12)>
    JacobianPassiveObserve(V3D &passive_observation_meas, const int &id, const Teammate &teammate,
                           const double &lidar_end_time);

    M3D GetActiveMutualObserveMeasurementNoise(const bool &degenerated, const StatesGroup &state_, const int &id, const Teammate &teammate, const double &mutual_observe_noise);

    M3D GetPassiveMutualObserveMeasurementNoise(const bool &degenerated, const StatesGroup &state_, const int &id, const Teammate &teammate, const double & lidar_end_time, const double &mutual_observe_noise);

    //Euclidean Cluster Extraction
    void ClusterExtractPredictRegion(const double &lidar_end_time, const PointCloudXYZI::Ptr cur_pcl_undistort);


    void ClusterExtractHighIntensity(const double &lidar_end_time, const PointCloudXYZI::Ptr cur_pcl_undistort);


    void CheckClusterValidation(const int &id, Teammate &teammate);


    void PredictTracker(const double &lidar_end_time, const int &id);

    void UpdateTracker(const double &lidar_end_time, const int &id, const Teammate &teammate,
                       const bool &cluster_meas, const bool &print_log);


    bool DeleteInvalidTemporaryTracker(const double &lidar_end_time, const int &index);


    bool TrajMatching(vector<Vector3d> &dyn_positions,
                      vector<Vector3d> &uav_positions,
                      Matrix3d &Rot,
                      Vector3d &trans,
                      double &Coeff,
                      const int &id);

    bool CreateTeammateTracker(const double &lidar_end_time, const int &index, StatesGroup &state_in,
                               StatesGroup &state_prop, const bool &print_log);

    void CreateTempTrackerByHighIntensity(const double &lidar_end_time);

    void CheckTempClusterValidation(const int &index);

    void PredictTemporaryTracker(const double &lidar_end_time, const int &index);

    void UpdateTemporaryTracker(const double &lidar_end_time, const int &index, const bool &print_log);

    void VisualizeText(const ros::Publisher &pub, const double &time, const int &id, const double &scale, const V3D &pos, const string &text, const V3D &color);

    void VisualizeBoundingBox(const ros::Publisher &pub, const double &time, const int &id, const V3D &color, const V3D &pos, const double &size);


    void VisualizeDeleteAllCluster(const double &time);

    void PublishTeammateOdom(const double &lidar_end_time);
    void PublishConnectedTeammateList(const double &lidar_end_time);

    void VisualizeTeammateTracker(const double &lidar_end_time, const int &teammate_id);

    void VisualizeDisconnectedTracker(const double &lidar_end_time, const int &teammate_id, const Teammate &teammate);

    void VisualizeMeshUAV(const double &lidar_end_time, const int &teammate_id, const Teammate &teammate);

    void VisualizeTempTracker(const double &lidar_end_time);

    void VisualizeTemporaryPredictRegion(const double &lidar_end_time);

    void VisualizeTempTrackerDelete(const double &lidar_end_time, const int &index);

    void VisualizeCluster(const double &lidar_end_time, const V3D &pos, const int &cluster_index, const V3D &size);

    void VisualizePredictRegion(const double &lidar_end_time, const int &id);

    void VisualizeRectangle(const ros::Publisher &pub_rect, const double &lidar_end_time, const int &rect_id, const V3D &position, const V3D rect_size);

    void VisualizeTeammateTrajectory(const ros::Publisher pub_, deque<Vector4d> &traj,
                                     const V3D position,
                                     const string namespace_,
                                     const double mkr_size,
                                     const double &timestamp,
                                     const int teammate_id,
                                     const int has_observation
    );

    void VisualizeTeammateTrajectorySphere(const ros::Publisher pub_, deque<Vector4d> &traj,
                                     const V3D position,
                                     const string namespace_,
                                     const double mkr_size,
                                     const double &timestamp,
                                     const int teammate_id,
                                     const int has_observation
    );

    id_esikf_map teammate_tracker;
    int pos_num_in_traj;
    vector<TemporaryTracker> temp_tracker;
    typedef unordered_map<int, Teammate> id_teammate_map;
    typedef id_teammate_map::value_type id2teammate;
    id_teammate_map teammates;
    string topic_name_prefix;
    M3D rot_world_to_gravity;
    StatesGroup state;
    ExtrinsicInfection extrinsic_infection;
    mutex mtx_buffer_reconnectID;
    bool degenerated;
    deque<Vector4d> teammate_traj[MAX_UAV_NUM];

private:
    ros::NodeHandle nh_;
    ros::Subscriber QuadState_subscriber, GlobalExtrinsic_subscriber;
    ros::Subscriber QuadState_subscriber_sim, GlobalExtrinsic_subscriber_sim;
    ros::Publisher QuadState_publisher, GlobalExtrinsic_publisher, pubUAV, pubMeshUAV, pubCluster, pubPredictRegionInput, pubHighIntenInput, pubPredictRegion, pubTempTracker, pubTeammateList, pubTeammateNum;
    ros::Publisher* pubTeammateOdom = new ros::Publisher[MAX_UAV_NUM];
    ros::Publisher* pubTeammateTraj = new ros::Publisher[MAX_UAV_NUM];
    swarm_msgs::QuadStatePub quadstate_msg_pub;
    swarm_msgs::GlobalExtrinsicStatus global_extrinsic_msg;
    int drone_id, inten_threshold, min_high_inten_cluster_size, min_cluster_size, cluster_id{0}, lidar_type, actual_uav_num{4};
    vector<Cluster> cluster_pos_tag;
    vector<int> reconnected_id;
    vector<int> teammate_id_by_traj_matching;
    ros::Publisher pubTeammateIdTrajMatching;
    double predict_region_radius, valid_cluster_dist_thresh, valid_cluster_size_thresh, reset_tracker_thresh, temp_predict_region_radius;
    double valid_temp_cluster_dist_thresh, same_obj_thresh, traj_matching_start_thresh, ave_match_error_thresh;
    nav_msgs::Odometry TeammateOdom;
    bool found_all_teammates{false}, cluster_extraction_in_predict_region;
    double text_scale, mesh_scale;
};
#endif //MULTIUAV_H