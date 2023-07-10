#ifndef EXPLORATION_FRONTIER_TREE_H_
#define EXPLORATION_FRONTIER_TREE_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <kdtree/kdtree.h>

#include <set>
#include <map>

#include "Tree.h"
#include "NodeSet.h"
#include "NavigationTree.h"

namespace explplanner
{
    
///\class FrontierTree 
///\brief 
///\author Luigi Freda  
class FrontierTree : public TreeBase<Eigen::Vector4d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static const double kColorGainScale; 
    static const double kTreeZVisualizationOffset;
    static const double kTreeTextZVisualizationOffset;    
    static const double kTreeTextMessageHeight;
    
    static const std::string kRvizNamespace;     
    
public:
    typedef Eigen::Vector4d StateVec;

    FrontierTree();
    FrontierTree(std::shared_ptr<volumetric_mapping::OctomapManager>& p_manager);
    ~FrontierTree();

    void initialize();
    
    void setRoot(double x, double y, double z);
    bool addNode(Node<StateVec>* node);      
    
    void updateFrontierNodes();    
    void addFrontierNodes(const std::vector<Node<StateVec>* >& frontierNodes);
        
    void clear();
    
    void clusterFrontiers();    
    
    int getNumNodes() const { return counter_ + 1; }
    
    kdtree* getKdTree() { return kdTree_; }
   
    void getClustersVecPtr(std::vector<NodeSet<StateVec>* >& clusters);
        
    Node<StateVec>* planBestFrontierCentroid(std::shared_ptr<NavigationTree>& p_nav_tree); 
    std::vector<geometry_msgs::Pose> getPathToBestFrontierCentroid(std::shared_ptr<NavigationTree>& p_nav_tree, Eigen::Vector3d& robot_position);    
       
    void resetSelectedBackTrackingCluster(); 
        
public:
    
    void publishNode(Node<StateVec>* node);
    void publishTree();
    void publishNodeRecursive(Node<StateVec>* node);

protected:
    
    Node<StateVec>* currentNode_;
    
    int g_ID_; // graphics ID for visual markers    
    int iterationCount_; 
    
    kdtree* kdTree_;
    
    std::set<Node<StateVec>*> frontierNodes_; 
        
    visualization_msgs::MarkerArray frontiersMarkerArray_;
    
    double maxNeighborDistance_;
    
    Node<StateVec>* bestBacktrackingNode_; 
    
protected: // clustering 

    std::map<int, NodeSet<StateVec> > clusters_; 
    int numClusters_;
    
    visualization_msgs::MarkerArray clustersMarkerArray_;    
};

}

#endif
