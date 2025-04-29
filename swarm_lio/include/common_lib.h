#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <so3_math.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <swarm_lio/States.h>
#include <swarm_lio/Pose6D.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <color.h>
#include <boost/filesystem.hpp>
#include "scope_timer.hpp"

using namespace std;
using namespace Eigen;
#define MAX_DRONE_ID (40) //集群中最大的飞机编号

// #define DEBUG_PRINT
#define DEPLOY
#define MAX_UAV_NUM (MAX_DRONE_ID + 1) //集群中最大的飞机数量
#define PI_M (3.14159265358)
#define G_m_s2 (9.81)         // Gravity const in GuangDong/China
#define DIM_STATE (18 + 6 * MAX_UAV_NUM)      // Dimension of states (Let Dim(SO(3)) = 3)

#define LIDAR_SP_LEN    (2)
#define INIT_COV   (1)
#define NUM_MATCH_POINTS    (5)

#define VEC_FROM_ARRAY(v)        v[0],v[1],v[2]
#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]

#define DEBUG_FILE_DIR(name)     (string(string(ROOT_DIR) + "Log/"+ name))

typedef swarm_lio::Pose6D     Pose6D;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZRGB     PointTypeRGB;
typedef pcl::PointCloud<PointType>    PointCloudXYZI;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
const M3D Eye3d(M3D::Identity());
const V3D Zero3d(0, 0, 0);
typedef Vector3f V3F;

#define MD(a,b)  Matrix<double, (a), (b)>
#define VD(a)    Matrix<double, (a), 1>



// Vector3d Lidar_offset_to_IMU(0.05512, 0.02226, -0.0297); // Horizon
// Vector3d Lidar_offset_to_IMU(0.04165, 0.02326, -0.0284); // Avia

enum LID_TYPE{AVIA = 1, VELO, OUSTER, L515, PANDAR, SIM}; //{1, 2, 3, 4, 5, 6}
struct MeasureGroup     // Lidar data and imu dates for the current process
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };
    double lidar_beg_time;
    PointCloudXYZI::Ptr lidar;
    deque<sensor_msgs::Imu::ConstPtr> imu;
};

struct StatesGroup
{
    StatesGroup() {
		this->rot_end = M3D::Identity();
		this->pos_end = Zero3d;
        this->vel_end = Zero3d;
        this->bias_g  = Zero3d;
        this->bias_a  = Zero3d;
        this->gravity = Zero3d;
        for (int id = 0; id < MAX_UAV_NUM; id++) {
            this->global_extrinsic_rot[id] = M3D::Identity();
            this->global_extrinsic_trans[id] = Zero3d;
        }
        this->cov.resize(DIM_STATE, DIM_STATE);
        this->cov.setIdentity();
        this->cov *= INIT_COV;
        this->cov.block<9,9>(9,9) = MD(9,9)::Identity() * 0.00001;
	};

    StatesGroup(const StatesGroup& b) {
		this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g  = b.bias_g;
        this->bias_a  = b.bias_a;
        this->gravity = b.gravity;
        for (int id = 0; id < MAX_UAV_NUM; id++) {
            this->global_extrinsic_rot[id] = b.global_extrinsic_rot[id];
            this->global_extrinsic_trans[id] = b.global_extrinsic_trans[id];
        }
        this->cov = b.cov;
	};

    StatesGroup& operator=(const StatesGroup& b)
	{
        this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g  = b.bias_g;
        this->bias_a  = b.bias_a;
        this->gravity = b.gravity;
        this->cov     = b.cov;
        for (int id = 0; id < MAX_UAV_NUM; id++) {
            this->global_extrinsic_rot[id] = b.global_extrinsic_rot[id];
            this->global_extrinsic_trans[id] = b.global_extrinsic_trans[id];
        }
        return *this;
	};

    StatesGroup operator+(const Matrix<double, DIM_STATE, 1> &state_add)
	{
        StatesGroup a;
		a.rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
		a.pos_end = this->pos_end + state_add.block<3,1>(3,0);
        a.vel_end = this->vel_end + state_add.block<3,1>(6,0);
        a.bias_g  = this->bias_g  + state_add.block<3,1>(9,0);
        a.bias_a  = this->bias_a  + state_add.block<3,1>(12,0);
        a.gravity = this->gravity + state_add.block<3,1>(15,0);
        for (int id = 0; id < MAX_UAV_NUM; id++) {
            int start_row = 18 + 6 * id;
            a.global_extrinsic_rot[id] = this->global_extrinsic_rot[id] * Exp(state_add(start_row,0), state_add(start_row+1,0), state_add(start_row+2,0));
            a.global_extrinsic_trans[id] = this->global_extrinsic_trans[id] + state_add.block<3,1>(start_row+3,0);
        }
        a.cov = this->cov;
		return a;
	};

    StatesGroup& operator+=(const Matrix<double, DIM_STATE, 1> &state_add)
	{
        this->rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
		this->pos_end += state_add.block<3,1>(3,0);
        this->vel_end += state_add.block<3,1>(6,0);
        this->bias_g  += state_add.block<3,1>(9,0);
        this->bias_a  += state_add.block<3,1>(12,0);
        this->gravity += state_add.block<3,1>(15,0);
        for (int id = 0; id < MAX_UAV_NUM; id++) {
            int start_row = 18 + 6 * id;
            this->global_extrinsic_rot[id] = this->global_extrinsic_rot[id] * Exp(state_add(start_row,0), state_add(start_row+1,0), state_add(start_row+2,0));
            this->global_extrinsic_trans[id] +=  state_add.block<3,1>(start_row+3,0);
        }
		return *this;
	};

    Matrix<double, DIM_STATE, 1> operator-(const StatesGroup& b)
	{
        Matrix<double, DIM_STATE, 1> a;
        M3D rotd(b.rot_end.transpose() * this->rot_end);
        a.block<3,1>(0,0)  = Log(rotd);
        a.block<3,1>(3,0)  = this->pos_end - b.pos_end;
        a.block<3,1>(6,0)  = this->vel_end - b.vel_end;
        a.block<3,1>(9,0)  = this->bias_g  - b.bias_g;
        a.block<3,1>(12,0) = this->bias_a  - b.bias_a;
        a.block<3,1>(15,0) = this->gravity - b.gravity;

        for (int id = 0; id < MAX_UAV_NUM; id++) {
            int start_row = 18 + 6 * id;
            M3D global_extrinsic_rotd(b.global_extrinsic_rot[id].transpose() * this->global_extrinsic_rot[id]);
            a.block<3,1>(start_row, 0) = Log(global_extrinsic_rotd);
            a.block<3,1>(start_row+3, 0) = this->global_extrinsic_trans[id] - b.global_extrinsic_trans[id];
        }
		return a;
	};

	M3D rot_end;      // the estimated attitude (rotation matrix) at the end lidar point
    V3D pos_end;      // the estimated position at the end lidar point (world frame)
    V3D vel_end;      // the estimated velocity at the end lidar point (world frame)
    V3D bias_g;       // gyroscope bias
    V3D bias_a;       // accelerator bias
    V3D gravity;      // the estimated gravity acceleration
    M3D global_extrinsic_rot[MAX_UAV_NUM];   //Extrinsic rotation between the global frames (mine and other quadrotors')
    V3D global_extrinsic_trans[MAX_UAV_NUM]; //Extrinsic translation between the global frames (mine and other quadrotors')
//    Matrix<double, DIM_STATE, DIM_STATE>  cov;
    //Dynamic Definition
    MatrixXd cov;
};

template<class T>
string SetString(T &param_in){
    stringstream ss;
    ss << param_in;
    string str = ss.str();
    return str;
}

template<typename T>
T rad2deg(T radians)
{
  return radians * 180.0 / PI_M;
}

template<typename T>
T deg2rad(T degrees)
{
  return degrees * PI_M / 180.0;
}

template<typename T>
auto set_pose6d(const double t, const Matrix<T, 3, 1> &a, const Matrix<T, 3, 1> &g, \
                const Matrix<T, 3, 1> &v, const Matrix<T, 3, 1> &p, const Matrix<T, 3, 3> &R)
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++)  rot_kp.rot[i*3+j] = R(i,j);
    }
    // Map<M3D>(rot_kp.rot, 3,3) = R;
    return move(rot_kp);
}

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/
template<typename T>
bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold, const int &point_num)
{
    MatrixXf A(point_num, 3);
    MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point_num; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }
    normvec = A.colPivHouseholderQr().solve(b);
    
    for (int j = 0; j < point_num; j++)
    {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
        {
            return false;
        }
    }

    normvec.normalize();
    return true;
}

template<typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }

    return true;
}


#endif
