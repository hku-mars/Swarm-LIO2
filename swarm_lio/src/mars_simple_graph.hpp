//
// Created by yunfan on 16/8/2023.
//

#ifndef MARS_SIMPLE_GRAPH_MARS_SIMPLE_GRAPH_HPP
#define MARS_SIMPLE_GRAPH_MARS_SIMPLE_GRAPH_HPP
#include "vector"
#include "iostream"
#include "cstdlib"
#include "cmath"
#include "queue"
#include "Eigen/Eigen"
namespace mars{
    using namespace std;
    using namespace Eigen;

    struct BaseEdgeData{
        BaseEdgeData(){
            from = -1;
            to = -1;
            is_valid = false;
        };
        int from,to;
        bool is_valid;
    };

    class EdgeData:public mars::BaseEdgeData{
    public:
        Matrix3d rotation;
        Vector3d translation;

        EdgeData inverse() const {
            EdgeData out;
            out.rotation = this->rotation.transpose();
            out.translation = - out.rotation * this->translation;
            out.from = this->to;
            out.to = this->from;
            return out;
        }

        friend ostream& operator << (ostream& os, EdgeData& cc)
        {
            os<<cc.from<<" --> "<<cc.to;
            return os;
        }
    };

    template<typename T> // T should be inherited from BaseEdgeData
    class MarsSimpleGraph {
    private:
        vector<T> nodes;

        int max_node_size_{0};

        int toAddress(const int & idx, const int idy){
            return idx*max_node_size_ + idy;
        }

        pair<int,int> toIndex(const int & idx){
            return make_pair(idx/max_node_size_, idx%max_node_size_);
        }

        vector<bool> use_as_root_;


    public:
        MarsSimpleGraph() = default;

        MarsSimpleGraph(int max_node_num){
            max_node_size_ = max_node_num;
            nodes.resize(max_node_num*max_node_num);
            use_as_root_.resize(max_node_num);
        }

        bool AddEdge(const T&edge){
            // add edge
            int address = toAddress(edge.from, edge.to);
            if(address >= nodes.size()){
                return false;
            }
            nodes[address] = edge;
            nodes[address].is_valid = true;

            // add reverse edge
            T edge_reverse = edge.inverse();
            address = toAddress(edge_reverse.from, edge_reverse.to);
            if(address >= nodes.size()){
                return false;
            }
            nodes[address] = edge_reverse;
            nodes[address].is_valid = true;
            return true;
        }

        bool RemoveEdge(const T&edge){
            int address = toAddress(edge.from, edge.to);
            if(address >= nodes.size()){
                return false;
            }
            nodes[address] = T();
            return true;
        }

        bool RemoveNode(const int&node_id){
            if(node_id > max_node_size_){
                return false;
            }
            for(int i = 0 ; i < max_node_size_ ;i++){
                int address = toAddress(node_id, i);
                nodes[address] = T();
                address = toAddress(i, node_id);
                nodes[address] = T();
            }
            return true;
        }

        bool GetConnectedEdgesAndNodes(const int & node_id, vector<T>& edges, vector<int> & connect_nodes){
            if(node_id > max_node_size_){
                return false;
            }
            static vector<bool> is_connected(max_node_size_, false);
            std::fill(is_connected.begin(), is_connected.end(), false);
            std::fill(use_as_root_.begin(), use_as_root_.end(), false);
            edges.clear();
            connect_nodes.clear();
            // DFS based edge search
            queue<int> search_queue;
            search_queue.push(node_id);
            while(!search_queue.empty()){
                int idx = search_queue.front();
                search_queue.pop();

                if(use_as_root_[idx]){
                    continue;
                }
                // valid node, set as root to true;
                use_as_root_[idx] = true;

                // find all edge connecting to idx
                for(int i = 0 ; i < max_node_size_ ;i++){
                    int address = toAddress(idx, i);

                    if(address >= nodes.size()){
                        return false;
                    }

                    if(!(nodes[address].is_valid)){
                        continue;
                    }

                    // valid edge add to queue
                    search_queue.push(i);
                    is_connected[i] = true;
                    if(i < idx)
                        continue;
                    edges.push_back(nodes[address]);

                }
            }

            if(edges.empty()){
                return false;
            }

            for(int i = 0 ; i < is_connected.size();i++){
                if(is_connected[i] ){
                    connect_nodes.push_back(i);
                }
            }

            return true;
        }

        ~MarsSimpleGraph()=default;

        void PrintGraph(){
            for(int i = 0 ; i < max_node_size_; i++){
                for(int j = 0 ; j < max_node_size_; j++){
                    int address = toAddress(i, j);
                    if(address >= nodes.size()){
                        return;
                    }
                    if(nodes[address].is_valid){
                        cout<<"1 ";
                    }else{
                        cout<<"0 ";
                    }
                }
                cout<<endl;
            }
        }
    private:

    };
}


#endif //MARS_SIMPLE_GRAPH_MARS_SIMPLE_GRAPH_HPP
