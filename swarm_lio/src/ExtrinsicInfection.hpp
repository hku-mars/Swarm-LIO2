//
// Created by fangcheng on 2022/11/06.
//
#ifndef ExtrinsicInfection_HPP
#define ExtrinsicInfection_HPP

#include <omp.h>
#include <unistd.h>
#include <Eigen/Core>
#include <common_lib.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <unordered_map>
#include "mars_simple_graph.hpp"
#include <vector>
using namespace std;
using namespace Eigen;

class ExtrinsicInfection{
public:
    template<typename T>
    bool isEqual(std::vector<T> const &v1, std::vector<T> const &v2)
    {
        return (v1.size() == v2.size() &&
                std::equal(v1.begin(), v1.end(), v2.begin()));
    }

    ExtrinsicInfection(){
        //Insert initials (initial values of global extrinsic)
        simple_graph = mars::MarsSimpleGraph<mars::EdgeData>(MAX_UAV_NUM);
    }

    ~ExtrinsicInfection() = default;



    //先验因子后面不会再被优化
    void InsertPriorFactor(const int &key, const M3D &rot, const V3D &trans, const double &noise, gtsam::NonlinearFactorGraph &graph){
        gtsam::Vector Vector6(6);
        Vector6 << noise, noise, noise, noise, noise, noise;
        gtsam::noiseModel::Diagonal::shared_ptr priorModel = gtsam::noiseModel::Diagonal::Variances(Vector6);
        graph.add(gtsam::PriorFactor<gtsam::Pose3>
                                (key, gtsam::Pose3(gtsam::Rot3(Eye3d),gtsam::Point3(Zero3d)), priorModel));
    }

    void InsertEdge(const int &index_from, const int &index_to, const M3D &rot, const V3D &trans, const double &noise, gtsam::NonlinearFactorGraph &graph){
        //edge 1_T_2 的噪声矩阵
        gtsam::Vector Vector6(6);
        Vector6 << noise, noise, noise, noise, noise, noise;
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
        //添加edge 1_T_2
        gtsam::Rot3 R_sam(rot);
        gtsam::Point3 t_sam(trans);
        gtsam::NonlinearFactor::shared_ptr factor(
                new gtsam::BetweenFactor<gtsam::Pose3>(index_from, index_to, gtsam::Pose3(R_sam, t_sam),
                                                       odometryNoise)); //添加edge，即约束
        graph.push_back(factor);
    }


    void SolveGraphIsam2(StatesGroup &state){
        vector<mars::EdgeData> edges;
        edges.clear();
        vector<int> connected_nodes;
        connected_nodes.clear();
        if(!simple_graph.GetConnectedEdgesAndNodes(drone_id, edges, connected_nodes))
            return;

        static vector<int> last_connected_nodes;
        bool same_nodes = isEqual(last_connected_nodes, connected_nodes);
        last_connected_nodes.assign(connected_nodes.begin(), connected_nodes.end());
        if(same_nodes){
//            cout << "Same Connected Nodes, Skip Optimization" << endl;
            return;
        }


        //顶点，即优化变量
        gtsam::Values initial;
        cout << "Connected Nodes: " << endl;
        for (int i = 0; i < connected_nodes.size(); ++i) {
            int id = connected_nodes[i];
            cout << connected_nodes[i] << ", ";
            if(state.global_extrinsic_trans[id].norm() < 0.001)
                initial.insert(id, gtsam::Pose3(gtsam::Rot3(Eye3d), gtsam::Point3(Zero3d)));
            else
                initial.insert(id, gtsam::Pose3(gtsam::Rot3(state.global_extrinsic_rot[id]), gtsam::Point3(state.global_extrinsic_trans[id])));
        }
        cout << endl;



        //定义因子图
        gtsam::NonlinearFactorGraph GTSAM_graph;
        //计算结果
        gtsam::Values result;

        //添加先验因子，本机和本机的外参，噪声很小
        InsertPriorFactor(drone_id, Eye3d, Zero3d, 1e-6, GTSAM_graph);

        //添加边，本机和其他机器人的外参，噪声略大
        for (int i = 0; i < edges.size(); ++i) {
            InsertEdge(edges[i].from, edges[i].to, edges[i].rotation, edges[i].translation, 1e-4, GTSAM_graph);
        }

        //Print Edges
        cout << "Edges: " << endl;
        for(auto e : edges){
            cout<<e << ",   ";
        }
        cout << endl;

        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        gtsam::ISAM2 isam(parameters);


//        try{
            isam.update(GTSAM_graph, initial);
            isam.update();
            result = isam.calculateEstimate();


            for (int i = 0; i < connected_nodes.size(); ++i) {
                int id = connected_nodes[i];
                if (state.global_extrinsic_trans[id].norm() < 0.001) {
                    state.global_extrinsic_rot[id] = GetGlobalExtrinsic(id, result).block<3, 3>(0, 0);
                    state.global_extrinsic_trans[id] = GetGlobalExtrinsic(id, result).block<3, 1>(0, 3);
                }
            }
//        }
//        catch(gtsam::IndeterminantLinearSystemException &e){
//            cout << "Graph is not connected!" << endl;
//        }


    }



    void PrintGraph(const int &index, gtsam::NonlinearFactorGraph &graph){
        string root_dir = ROOT_DIR;
        graph.saveGraph(string(root_dir + "/Log/Graph" + SetString(drone_id)) + "_" + SetString(index) + string(".dot"));
    }

//
//    void SolveGraphLM(){
//        gtsam::LevenbergMarquardtParams parameters;
//        parameters.relativeErrorTol = 1e-5;
//        parameters.maxIterations = 5;
//        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, parameters);
//        result = optimizer.optimize();
//    }
//
//    void SolveGraphGaussNewton(){
//        gtsam::GaussNewtonParams parameters;
//        parameters.relativeErrorTol = 1e-5;
//        parameters.maxIterations = 5;
//        gtsam::GaussNewtonOptimizer optimizer(graph, initial, parameters);
//        result = optimizer.optimize();
//    }

    Matrix4d GetGlobalExtrinsic(const int &key, const gtsam::Values &result){
        Matrix4d extrinsic;
        extrinsic.setIdentity();
        gtsam::Pose3 pose = result.at(key).cast<gtsam::Pose3>();
        extrinsic.block<3,3>(0,0) = pose.rotation().matrix();
        extrinsic.block<3,1>(0,3) = pose.translation();
        return extrinsic;
    }

    void PrintResult(const gtsam::Values &result){
        for (uint i = 0; i < result.size(); i++) {
            gtsam::Pose3 pose = result.at(i).cast<gtsam::Pose3>();
            cout << drone_id << "^T_" << i << " = " << RotMtoEuler(pose.rotation().matrix()).transpose() * 57.3 << " "
                 << pose.translation().transpose() << endl;
        }
    }

    void ResetGraph(gtsam::NonlinearFactorGraph &graph){
        graph.resize(0);
    }

    int GetGraphSize(const gtsam::NonlinearFactorGraph &graph){
        return graph.size();
    }

public:
    int drone_id;
    mars::MarsSimpleGraph<mars::EdgeData> simple_graph;
};


#endif