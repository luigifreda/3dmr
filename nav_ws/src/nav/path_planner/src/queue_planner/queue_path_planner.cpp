/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com> and Alcor Lab (La Sapienza University)
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

#include "QueuePathPlanner.h"
#include <signal.h>

void mySigintHandler(int signum)
{
    std::cout << "mySigintHandler()" << std::endl;
    //boost::recursive_mutex::scoped_lock destroy_locker(destroy_mutex); 

    //p_planner.reset();
    //p_marker_controller.reset(); 

    ros::shutdown();
    exit(signum);
}

int main(int argc, char *argv[])
{
    // ros initialization
    ros::init(argc,argv,"queue_path_planner");
    
    //override default sigint handler 
    signal(SIGINT, mySigintHandler);
    
    // queue planner
    QueuePathPlanner planner;

    // ros loop
#if 1    
    ros::spin();
#else    
    ros::MultiThreadedSpinner spinner(2); // Use 2-4 threads
    spinner.spin(); // spin() will not return until the node has been shutdown
#endif     

    return 0;
}
