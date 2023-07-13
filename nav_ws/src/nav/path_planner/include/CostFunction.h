/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com>  
* For more information see <https://github.com/luigifreda/3dmr>
*
* 3DMR is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DMR is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DMR. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include <ros/ros.h>

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif 

#include <pcl/point_types.h>
#include <cstring>
#include <limits>

static const std::string CostFunctionNames[] = {"BaseCost","OriginalCost","TraversabilityCost","TraversabilityProdCost"};

///	\class UtilityFunction
///	\author Luigi Freda
///	\brief Base class for utility function management: mixing traversability with an auxiliary function 
///	\note  cost(n) = dist(current, n) + heuristic(n) + lambda_trav*(traversability(n)-min_trav)/range_trav +  lambda_aux_*aux_conf*(max_aux_utility_ - aux_utility(n))/range_aux_utility_
/// 	\todo 
///	\date
///	\warning
class BaseCostFunction
{

public:
    
    static const float kLamdaTravDefault; 
    static const float kLamdaAuxDefault; 
    static const float kLamdaDzDefault; 
    static const double kEpsilon; 
            

        enum CostFunctionType {kBaseCost         = 0,   
                         kSimpleCost             = 1,  // cost(n) = (dist(current, n) + heuristic(n))*( 1 + ( traversability(n) - trav_min)/(trav_max - trav_min) )
                         kTraversabilityCost     = 2, 
                         kTraversabilityProdCost = 3,
                         kNumCostFunctions
        };
        
public:
 
    BaseCostFunction()
    {
        type_ = kBaseCost;
        lambda_trav_ = kLamdaTravDefault;
        lambda_aux_ = kLamdaAuxDefault;
        lambda_dz_ = kLamdaDzDefault; 
        min_trav_cost_ = min_aux_utility_ = 0;
        max_trav_cost_ = max_aux_utility_ = 0; 
        range_trav_cost_ = range_aux_utility_ = 1; 
        
        time0_ = ros::Time::now();
        tau_exp_decay_ = std::numeric_limits<float>::max();
    }
    
    virtual void initTime()
    {
        time0_ = ros::Time::now();   
    }
    
    double timeExpDecay()
    {
        //return exp(-(ros::Time::now()-time0_).toSec()/tau_exp_decay_);
        double deltaT = (ros::Time::now()-time0_).toSec();
        //std::cout << "timeExpDecay() - deltaT: " <<  deltaT << std::endl; 
        double res = exp(-deltaT/tau_exp_decay_);
        std::cout << "timeExpDecay() - res: " <<  res << std::endl; 
        return res; 
    }
    
    // A* heuristic utility
    virtual double heuristic(const pcl::PointXYZI& current, const pcl::PointXYZI& goal) 
    {
        return dist(current,goal);
    }
    
    virtual double cost(const pcl::PointXYZI& current, const pcl::PointXYZI& next, const pcl::PointXYZI& goal, double traversability_cost, double aux_utility = 0, double aux_conf = 1)    
    {
        return dist(current,next) + heuristic(next, goal) + lambda_trav_*(traversability_cost-min_trav_cost_)/(range_trav_cost_+kEpsilon) + lambda_aux_*timeExpDecay()*aux_conf*(max_aux_utility_ - aux_utility)/(range_aux_utility_+kEpsilon);
    }
    
    virtual std::string getName() { return CostFunctionNames[type_]; }

    
public: // getters
    
    float& GetMaxTrav() {return max_trav_cost_;}
    float& GetMinTrav() {return min_trav_cost_;}
    float& GetRangeTrav() {return range_trav_cost_;}
    
    float& GetMaxAuxUtility() {return max_aux_utility_;}
    float& GetMinAuxUtility() {return min_aux_utility_;}
    float& GetRangeAuxUtility() {return range_aux_utility_;}
    
    float& GetLambdaTrav() {return lambda_trav_;}
    float& GetLambdaTravDz() {return lambda_dz_;}
        
    float& GetLambdaAuxUtility() {return lambda_aux_;}
    float& GetTauExpDecay() { return tau_exp_decay_; }
    
public: // setters 
    
    void SetLambdaTrav(float val) { lambda_trav_ = val; }
    void SetLambdaTravDz(float val) { lambda_dz_ = val; }
        
    void SetLambdaAuxUtility(float val) { lambda_aux_ = val; }
    
    void SetTauExpDecay(float val) { tau_exp_decay_ = val; }
        
protected:
    
    template<class Point>
    inline double dist(const Point& p1, const Point& p2)
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
    }

protected:
    
    float lambda_trav_;
    float lambda_aux_; 
    float lambda_dz_;
    
    float max_trav_cost_;
    float min_trav_cost_; 
    float range_trav_cost_; // max - min
    
    float max_aux_utility_;
    float min_aux_utility_; 
    float range_aux_utility_; // max - min
    
    float tau_exp_decay_;   // [s] exponential time constant for the decay 
    ros::Time time0_;
    
    CostFunctionType type_; 
};


///	\class SimpleCostFunction
///	\author Luigi Freda
///	\brief cost(n) = dist(current, n)+ traversability(n) + heuristic(n)
///	\note This was originally used in the planner
/// 	\todo 
///	\date
///	\warning
class SimpleCostFunction: public BaseCostFunction
{
public: 
    SimpleCostFunction(){ type_ = kSimpleCost; }
        
    double cost(const pcl::PointXYZI& current, const pcl::PointXYZI& next, const pcl::PointXYZI& goal, double traversability_cost, double aux_utility = 0, double aux_conf = 1)    
    {
        //double heuristic = dist(pcl_traversability_.points[neighbors[i].point_idx], goal_); // A* heuristic             
        //cost = dist(pcl_traversability_[nodes_[current_node_idx_].point_idx], pcl_traversability_[neighbors[i].point_idx]) + pcl_traversability_[neighbors[i].point_idx].intensity + heuristic;
        
        //return dist(current,next) +  heuristic(next, goal) + lambda_trav_*traversability_cost + lambda_dz_* fabs(current.z - next.z);
        return (dist(current,next) +  heuristic(next, goal) + lambda_dz_* fabs(current.z - next.z) )*(1. + lambda_trav_*(traversability_cost-min_trav_cost_)/(range_trav_cost_+kEpsilon) );
                
    }
};

///	\class TraversabilityCostFunction
///	\author Luigi Freda
///	\brief  cost(n) = (dist(current, n) + heuristic(n))*(lambda_trav*(traversability(n)-min_trav)/range_trav +  lambda_aux_*aux_conf*(max_aux_utility_ - aux_utility(n))/range_aux_utility_ + 1)
///	\note Here dist and heuristic are treated in the same way. As a matter of fact they are distance. This idea comes from the paper of Kuffner "Randomized Statistical Path Planning"
/// 	\todo 
///	\date
///	\warning
class TraversabilityCostFunction: public BaseCostFunction
{
public: 
    TraversabilityCostFunction(){type_ = kTraversabilityCost; }
    
    
    double cost(const pcl::PointXYZI& current, const pcl::PointXYZI& next, const pcl::PointXYZI& goal, double traversability_cost, double aux_utility = 0, double aux_conf = 1)    
    {
        return (dist(current,next) +  heuristic(next, goal))*
                ( lambda_trav_*(traversability_cost-min_trav_cost_)/(range_trav_cost_+kEpsilon) 
                + lambda_aux_*timeExpDecay()*aux_conf*(max_aux_utility_-aux_utility)/(range_aux_utility_+kEpsilon) +1.);
    }
};

///	\class TraversabilityProdCostFunction
///	\author Luigi
///	\brief cost(n) = (dist(current, n) + heuristic(n))* (lambda_trav*(traversability(n)-min_trav)/range_trav + 1)*( lambda_aux_*aux_conf*(max_aux_utility_ - aux_utility(n))/range_aux_utility_ + 1.)
///	\note Here we use the product in order to mix the cost functions. The (+1) is added in order to avoid that when one function = 0, the influence of all the other cost functions are nullified
/// 	\todo 
///	\date
///	\warning
class TraversabilityProdCostFunction: public BaseCostFunction
{
public: 
    TraversabilityProdCostFunction(){type_ = kTraversabilityProdCost; }
    
    
    double cost(const pcl::PointXYZI& current, const pcl::PointXYZI& next, const pcl::PointXYZI& goal, double traversability_cost, double aux_utility = 0, double aux_conf = 1)    
    {
        return (dist(current,next) +  heuristic(next, goal))*
                ( lambda_trav_*(traversability_cost-min_trav_cost_)/(range_trav_cost_+kEpsilon) +1.)*
                ( lambda_aux_*timeExpDecay()*aux_conf*(max_aux_utility_-aux_utility)/(range_aux_utility_+kEpsilon) +1. );
    }
};


#endif /* COST_FUNCTION_H */

