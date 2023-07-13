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

#ifndef PRIORITY_QUEUE_H_
#define PRIORITY_QUEUE_H_

#include <utility>      // std::pair, std::make_pair
#include <iostream>     // std::cout
#include <queue>        // std::priority_queue
#include <algorithm>    // std::sort

#include <boost/thread/recursive_mutex.hpp>
#include <geometry_msgs/Point.h>



///\class PriorityElement 
///\brief Element of a priority queue
///\author Luigi Freda
template<typename T, typename PriorityType = int>
class PriorityElement
{
public:

    PriorityElement() : id(-1), priority(0){}

    bool operator==(const PriorityElement& rhs) const
    {
        if (this->id == rhs.id)
            return true;
        return false;
    }


public:

    friend std::ostream& operator<<(std::ostream& os, const PriorityElement& obj)
    {
        os << "id: " << obj.id << ", data: " << obj.data << ", priority: " << obj.priority;
        return os;
    }

public:

    T data;
    int id; // -1 is invalid 
    PriorityType priority;
};



///\class PriorityQueueComparison 
///\brief Class for specifying the priority-based comparison
///\author Luigi Freda
template<typename T, typename PriorityType = int>
class PriorityQueueComparison
{
public:

    PriorityQueueComparison() {}

    bool operator()(const PriorityElement<T, PriorityType>& lhs, const PriorityElement<T, PriorityType>& rhs) const
    {
        return (lhs.priority < rhs.priority);
    }
};



///\class PriorityQueue 
///\brief Class for implementing a priority queue 
///\author Luigi Freda
template<typename T, typename PriorityType = int>
class PriorityQueue
{
public:
    typedef PriorityElement<T, PriorityType> PriorityElementT;
    typedef std::vector<PriorityElementT> PriorityContainerT;
    typedef PriorityQueueComparison<T, PriorityType> PriorityQueueComparisonT;

public:

    PriorityQueue(){}

    void Push(const int id, const T& data, const PriorityType priority)
    {
        boost::recursive_mutex::scoped_lock locker(mutex_);

        PriorityElementT new_element;
        new_element.id = id;        
        new_element.data = data;
        new_element.priority = priority;

        typename PriorityContainerT::iterator it;

        // check if the element id is already there 
        it = std::find(queue_.begin(), queue_.end(), new_element);
        if (it != queue_.end())
        {
            // already there, update it!
            *it = new_element;
        }
        else
        {
            // insert new element 
            queue_.push_back(new_element);
        }
        // sort and move the highest priority on the back 
        std::sort(queue_.begin(), queue_.begin(), comparison_);        
    }

    // remove top element 
    void Pop()
    {
        boost::recursive_mutex::scoped_lock locker(mutex_);
        queue_.pop_back();
    }

    // get top element 
    PriorityElementT& Top()
    {
        boost::recursive_mutex::scoped_lock locker(mutex_);
        return queue_.back();
    }
    
    void Erase(const int id)
    {
        boost::recursive_mutex::scoped_lock locker(mutex_);

        PriorityElementT element_to_erase;
        element_to_erase.id = id;

        typename PriorityContainerT::iterator it;

        // check if the element id is there 
        it = std::find(queue_.begin(), queue_.end(), element_to_erase);
        if (it != queue_.end())
        {
            queue_.erase(it);
            std::sort(queue_.begin(), queue_.begin(), comparison_); // for security, re-sort it!
        }
    }
    
    void Clear()
    {
        boost::recursive_mutex::scoped_lock locker(mutex_);
        queue_.clear();
    }
    
    bool IsEmpty()
    {
        boost::recursive_mutex::scoped_lock locker(mutex_);
        return queue_.empty();
    }
   


public:

    friend std::ostream& operator<<(std::ostream& os, PriorityQueue& obj)
    {
        boost::recursive_mutex::scoped_lock locker(obj.mutex_);        
        os << "num elements: " << obj.queue_.size() << std::endl; 
        for(size_t jj=0; jj< obj.queue_.size(); jj++)
        {
            os << obj.queue_[jj] << std::endl; 
        }
        return os;
    }

protected:

    PriorityContainerT queue_;
    PriorityQueueComparisonT comparison_;

    boost::recursive_mutex mutex_;
};


///\class PriorityQueuePoint 
///\brief Class for implementing a priority queue with 3D points 
///\author Luigi Freda
class PriorityPointQueue: public PriorityQueue<geometry_msgs::Point, int> 
{
public:
    
    typedef PriorityPointQueue::PriorityElementT PriorityPoint;
    
public:
    
    PriorityPointQueue(){}  
    
    geometry_msgs::Point& TopPoint()
    {
        boost::recursive_mutex::scoped_lock locker(mutex_);  
        PriorityPoint& priority_elem = Top(); 
        return priority_elem.data;
    }
    
    bool RemoveClosePoints(const geometry_msgs::Point& ref_point, const double distanceTh, std::vector<PriorityPoint>& removed_priority_points)
    {
        bool res = false;
        
        removed_priority_points.clear(); 
        
        boost::recursive_mutex::scoped_lock locker(mutex_);         
        for(PriorityContainerT::iterator it=queue_.begin(); it!=queue_.end(); /*nop*/)
        {
            const geometry_msgs::Point& current_point = (*it).data;
            const double distance = sqrt( pow(ref_point.x - current_point.x,2) + pow(ref_point.y - current_point.y,2) + pow(ref_point.z - current_point.z,2) );
            std::cout << "pp distance: " << distance << std::endl; 
            if(distance < distanceTh)
            {
                removed_priority_points.push_back((*it));
                
                it = queue_.erase(it);
                res = true; 
            }
            else
            {
                it++;
            }
        }
        return res; // return if we have removed something 
    }
    
};

#endif // PRIORITY_QUEUE_H_
