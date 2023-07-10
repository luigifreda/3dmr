#ifndef TEAM_MODEL_H_
#define TEAM_MODEL_H_

#include <iostream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/StdVector>

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <boost/thread/recursive_mutex.hpp>

#include <exploration_msgs/ExplorationRobotMessage.h>


///\class RobotPlanningState 
///\brief 
///\author Luigi Freda
class RobotPlanningState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
public: 
    RobotPlanningState();
    
    void Reset();
    
public:  // planning information 
    Eigen::Vector3d goal;    
    nav_msgs::Path path; 
    double path_length;  // nav cost 
    ros::Time timestamp;
    
    bool bNoInformation;     
    bool bCompleted; 
    
public: // deployment information 
    Eigen::Vector3d position;  // current robot position   
    bool bValidPosition;       // position info is valid?
    ros::Time timestampPosition;            
};


///\class TeamModel 
///\brief 
///\author Luigi Freda
class TeamModel
{
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static const double kExpirationTime; // [sec]
    static const double kNodeConflictDistance; // [m]
    static const int kNumRobotsDefault;

public:

    TeamModel();
    
    // update team model 
    void Update(); 
    
    // check expired states 
    void CleanExpiredData();
    
    bool IsNodeConflict();
    
public: // setters 
    
    void setMyRobotId(const int& my_robot_id) { my_robot_id_ = my_robot_id; }
    
    void setNumRobots(const int& num_robots);
    
    void setRobotPlanData(const int& robot_id, const Eigen::Vector3d& goal, const ros::Time& timestamp);
    void setRobotPlanData(const int& robot_id, const Eigen::Vector3d& goal, const nav_msgs::Path& path, const double path_cost, const ros::Time& timestamp);
    
    void setRobotPositionData(const int& robot_id, const Eigen::Vector3d& position, const ros::Time& timestamp);    
    
    void setRobotMessage(const exploration_msgs::ExplorationRobotMessage::ConstPtr msg, bool isMsgDirect = false);
    
    void setConflictDistance(double distance) { conflict_distance_ = distance; }

public: 
    
    Eigen::Vector3d getMyGoal(); // get my goal 
    nav_msgs::Path getMyPath();    
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > getConflictingNodes();
    double getConflictingDistance();
    
    bool isValid(const int& robot_id);
    
protected:
    
    void checkForRobotId(int robot_id); 
    
    double computePathLength(const nav_msgs::Path& path);

protected:

    boost::recursive_mutex mutex_; 

    std::vector<RobotPlanningState, Eigen::aligned_allocator<RobotPlanningState> > robot_states_;
    int num_robots_;
    int my_robot_id_; 
    
    double conflict_distance_;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > conflicting_nodes_; 
};


#endif // TEAM_MODEL_H_
