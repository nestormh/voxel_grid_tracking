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
#include <tf/transform_datatypes.h>

#include <pcl_ros/point_cloud.h>

#include "polargridtracking.h"

#include "voxel.h"

#define DEFAULT_BASE_FRAME "left_cam"

namespace voxel_grid_tracking {
    
class VoxelGridTracking
{
public:
    VoxelGridTracking();
    
    void start();
    void setDeltaYawSpeedAndTime(const double & deltaYaw, const double & deltaSpeed, const double & deltaTime);
protected:
    // Callbacks
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    // Method functions
    void compute(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr & pointCloud);
    void reset();
    void getVoxelGridFromPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud);
    void getMeasurementModel();
    void initialization();
    void particleToVoxel(const Particle3d & particle, 
                         int32_t & posX, int32_t & posY, int32_t & posZ);
    void prediction();
    
    // Visualization functions
    void publishVoxels();
    void publishParticles();    
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloud;
    
    double m_deltaYaw, m_deltaSpeed, m_deltaTime;
    
    VoxelGrid m_grid;
    
    typedef boost::multi_array<double, 4> ColorMatrix;
    ColorMatrix m_colors;
    
    uint32_t m_dimX, m_dimY, m_dimZ;
    
    bool m_initialized;
    
    tf::StampedTransform m_lastMapOdomTransform;
    
    // Parameters
    polar_grid_tracking::t_Camera_params m_cameraParams;
    double m_minX, m_maxX, m_minY, m_maxY, m_minZ, m_maxZ;
    double m_cellSizeX, m_cellSizeY, m_cellSizeZ;
    double m_maxVelX, m_maxVelY, m_maxVelZ;
    double m_particlesPerCell, m_threshProbForCreation;
    string m_baseFrame;

    // Subscribers
    ros::Subscriber m_pointCloudSub;
    
    // Publishers
    ros::Publisher m_voxelsPub;
    ros::Publisher m_pointsPerVoxelPub;
    ros::Publisher m_particlesPub;
    ros::Publisher m_particlesPositionPub;
};

}

#endif // VOXELGRIDTRACKING_H
