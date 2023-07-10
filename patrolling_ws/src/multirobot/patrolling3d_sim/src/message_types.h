/**
* This file is part of the ROS package patrolling3d_sim which belongs to the framework 3DPATROLLING. 
* This file is a VERY modified version of the corresponding file in patrolling_sim 
* http://wiki.ros.org/patrolling_sim see license below.
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> (La Sapienza University)
* For more information see <https://gitlab.com/luigifreda/3dpatrolling>
*
* 3DPATROLLING is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DPATROLLING is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DPATROLLING. If not, see <http://www.gnu.org/licenses/>.
*/

/**
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Luigi Freda (2016-Present) and Luca Iocchi (2014) 
**/

#ifndef PATROLLING_MESSAGE_TYPES_H
#define PATROLLING_MESSAGE_TYPES_H

// Enable/disable stamped messages 
#define USE_STAMPED_MSG_INT16_MULTIARRAY 1  // use or not messages with header and timestamp 
                                            // N.B.:  real robots or distributed nodes (deployed on different computers) need to synch their system clocks,
                                            // otherwise set this macro to 0

/// < ==========================================================================
#define DISABLE_COOPERATION_AND_COORDINATION 0  /// < 1 CC disabled, (CC=Cooperation and Coordination)
                                                /// < 0 CC enabled      
/// < ==========================================================================

// Message types
#define INITIALIZE_MSG_TYPE 10         // initialization: [ID_ROBOT, msg_type, sub_msg_type]
#define TARGET_REACHED_MSG_TYPE 11     // reached goal:  [ID_ROBOT, msg_type, vertex] 
#define INTERFERENCE_MSG_TYPE 12       // interference is occurring:  [ID_ROBOT, msg_type]
#define TARGET_PLANNED_MSG_TYPE 14     // planned (intention) goal but still to check if I can go there: [ID_ROBOT, msg_type, vertex] 
#define TARGET_SELECTED_MSG_TYPE 15    // selected and verified a goal: [ID_ROBOT, msg_type, vertex, nav_cost] 
#define TARGET_ABORTED_MSG_TYPE 16     // aborted goal: [ID_ROBOT, msg_type, vertex]  
#define TARGET_INTERCEPTED_MSG_TYPE 17 // a vertex has beend intercepted (visited) along the way: [ID_ROBOT, msg_type, vertex] 
#define VERTEX_COVERED_MSG_TYPE 18     // a vertex is covered: [ID_ROBOT, msg_type, vertex]   
#define IDLENESS_SYNC_MSG_TYPE 19      // idleness synchronization message:  [ID_ROBOT, msg_type, idleness[1],...,idleness[dimension]] 
#define NAV_COST_MSG_TYPE 20 // selected and verified a goal from a graph node: [ID_ROBOT, msg_type, vertex1, vertex2, nav_cost] 

// Sub-message types 
#define SUB_MSG_REALLOCATION (-1000)
#define SUB_MSG_START (100)
#define SUB_MSG_END (999)
#define SUB_MSG_ROBOT_INIT (1)


#define GBS_MSG_TYPE 31
#define SEBS_MSG_TYPE 32
#define CBLS_MSG_TYPE 32


// Message types for DTAGreedy algorithm
#define DTAGREEDY_MSG_TYPE 40 // Idleness message: msg format: [ID_ROBOT,msg_type,global_idleness[1..dimension],next_vertex]


// Message types for DTASSI algorithm
#define DTASSI_TR 41  //Task request, msg format: [ID_ROBOT,msg_type,next_vertex_index,bid_value]
#define DTASSI_BID 42 //Bid Message, msg format: [ID_ROBOT,msg_type,next_vertex_index,bid_value]


#endif