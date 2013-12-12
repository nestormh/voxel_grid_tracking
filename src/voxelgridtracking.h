/*
 *  Copyright 2013 Néstor Morales Hernández <nestor@isaatc.ull.es>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 * 
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef VOXELGRIDTRACKING_H
#define VOXELGRIDTRACKING_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace voxel_grid_tracking {
    
class VoxelGridTracking
{
public:
    VoxelGridTracking();
    
    void start();
protected:
    void readOctomap();
    void deltaTimeCallback(const std_msgs::Float64::ConstPtr& msg) ;
    
    ros::ServiceClient m_octomapClient;
    ros::Subscriber m_deltaTimeSub;
    
    double m_deltaTime;
};

}

#endif // VOXELGRIDTRACKING_H
