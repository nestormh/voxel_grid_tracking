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


#ifndef POLARGRIDTRACKINGROS_H
#define POLARGRIDTRACKINGROS_H

#include "polargridtracking.h"

#include "ros/ros.h"

namespace polar_grid_tracking {

class PolarGridTrackingROS : public PolarGridTracking
{
public:
    PolarGridTrackingROS(const uint32_t & rows, const uint32_t & cols, const double & cellSizeX, const double & cellSizeZ, 
                         const double & maxVelX, const double & maxVelZ, const t_Camera_params & cameraParams, 
                         const double & particlesPerCell, const double & threshProbForCreation);
    
    void compute(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr & pointCloud);
protected:
    void publishAll(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud);
    
    void publishPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr & pointCloud);
    void publishBinaryMap();
    void publishParticles(ros::Publisher & particlesPub, const double & zPlane);
    void publishColumnAverage(const double & zPlane);
    
    ros::Publisher m_pointCloudPub;
    ros::Publisher m_binaryMapPub;
    ros::Publisher m_particlesPub;
    ros::Publisher m_oldParticlesPub;
    ros::Publisher m_colAvgPub;
};

}

#endif // POLARGRIDTRACKINGROS_H
