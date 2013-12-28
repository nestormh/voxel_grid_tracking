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
#include "voxelobstacle.h"

#define DEFAULT_BASE_FRAME "left_cam"
#define MAX_OBSTACLES_VISUALIZATION 10000

namespace voxel_grid_tracking {
    
class VoxelGridTracking
{
public:
    VoxelGridTracking();
    
    void start();
    void setEgoMotion(const double & deltaYaw, const double & deltaPitch, const double & deltaSpeed, const double & deltaTime);
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
    void measurementBasedUpdate();
    void segment();
    void aggregation();
    void noiseRemoval();
    void updateObstacles();
    void filterObstacles();
    void joinCommonVolumes();
    void updateSpeedFromObstacles();
    
    // Visualization functions
    void publishVoxels();
    void publishParticles();
    void publishMainVectors();
    void publishObstacles();
    void publishObstacleCubes();
    void publishROI();
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloud;
    
    double m_deltaYaw, m_deltaPitch, m_speed, m_deltaTime;
    
    VoxelGrid m_grid;
    
    typedef boost::multi_array<double, 4> ColorMatrix;
    ColorMatrix m_colors;
    typedef boost::multi_array<double, 2> ColorVector;
    ColorVector m_obstacleColors;
    
    uint32_t m_dimX, m_dimY, m_dimZ;
    
    bool m_initialized;
    
    tf::StampedTransform m_lastMapOdomTransform;
    
    typedef vector<VoxelObstacle> ObstacleList;
    ObstacleList m_obstacles;
    
    uint32_t m_currentId;
    
    // Parameters
    polar_grid_tracking::t_Camera_params m_cameraParams;
    double m_minX, m_maxX, m_minY, m_maxY, m_minZ, m_maxZ;
    double m_cellSizeX, m_cellSizeY, m_cellSizeZ;
    double m_maxVelX, m_maxVelY, m_maxVelZ;
    double m_particlesPerCell, m_threshProbForCreation;
    string m_baseFrame;
    uint32_t m_neighBorX, m_neighBorY, m_neighBorZ;
    double m_threshYaw, m_threshPitch, m_threshMagnitude;
    uint32_t m_minVoxelsPerObstacle;
    double m_minObstacleDensity;
    double m_minVoxelDensity;
    SpeedMethod m_speedMethod;
    SpeedMethod m_obstacleSpeedMethod;
    double m_yawInterval;
    double m_pitchInterval;
    double m_maxCommonVolume;
    double m_minObstacleHeight;

    // Subscribers
    ros::Subscriber m_pointCloudSub;
    
    // Publishers
    ros::Publisher m_voxelsPub;
    ros::Publisher m_pointsPerVoxelPub;
    ros::Publisher m_particlesPub;
    ros::Publisher m_particlesPositionPub;
    ros::Publisher m_mainVectorsPub;
    ros::Publisher m_obstaclesPub;
    ros::Publisher m_obstacleCubesPub;
    ros::Publisher m_obstacleSpeedPub;
    ros::Publisher m_obstacleSpeedTextPub;
    ros::Publisher m_ROIPub;
};

}

#endif // VOXELGRIDTRACKING_H
