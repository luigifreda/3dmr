#ifndef NODE_SET_H_
#define NODE_SET_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

#include <set>
#include <map>

#include "Tree.h"

namespace explplanner
{
    
template<typename StateVec>    
class NodeSet 
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
public:

    NodeSet();
    NodeSet(Node<StateVec>* nodeIn, int labelIn);
    NodeSet(std::set<Node<StateVec>*>& nodesIn, int labelIn);
    
    ~NodeSet();

    void clear();
    
    void insert(Node<StateVec>* node);
    
    void insert(std::set<Node<StateVec>*>& otherNodes);
    
    void updateData();
    
public: 
    
    double informationGain; // total information gain of the set 
    double utility;         // total utility = exp(-params_.degressiveCoeff_ * centroidNode->distance_) * informationGain
    double navigationCost;  // navigation cost to reach the centroid 
    int label;              // label of the node set 
    
    Eigen::Vector3d centroid;     
    Node<StateVec>* centroidNode; // closest node to centroid 
            
    std::set<Node<StateVec>*> nodes;  
    
private: 
    
    void findClosestNodeToCentroid();
    
};

}

#endif
