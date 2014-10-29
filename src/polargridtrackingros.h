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
#include "sensor_msgs/PointCloud2.h"

#include <tf/transform_datatypes.h>

#include "ros/ros.h"

namespace polar_grid_tracking {

class polar_grid_trackingROS : public polar_grid_tracking
{
public:
    polar_grid_trackingROS(const uint32_t & rows, const uint32_t & cols, const double & cellSizeX, const double & cellSizeZ, 
                         const double & maxVelX, const double & maxVelZ, const t_Camera_params & cameraParams, 
                         const double & particlesPerCell, const double & threshProbForCreation, 
                         const double & gridDepthFactor, const uint32_t &  gridColumnFactor, const double & yawInterval,
                         const double & threshYaw, const double & threshMagnitude);
    
    void start();
    void compute(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr & pointCloud);
protected:
    void reconstructObjects(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr & pointCloud);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) ;
    
    void publishAll(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud);
    
    void publishPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr & pointCloud);
    void publishPointCloud(const pcl::PointCloud< PointXYZRGBDirected >::Ptr & pointCloud);
    void publishPointCloudOrientation(const pcl::PointCloud< PointXYZRGBDirected >::Ptr & pointCloud);
    void publishBinaryMap();
    void publishParticles(ros::Publisher & particlesPub, const double & zPlane);
    void publishColumnAverage(const double & zPlane);
    void publishPolarGrid();
    void publishPolarCellYaw(const double & zPlane);
    void clearObstaclesAndROIs();
    void publishObstacles();
    void publishROIs();
    void publishPointCloudInObstacles(const pcl::PointCloud< PointXYZRGBDirected >::Ptr & pointCloud);
    void visualizeROI2d();
    
    ros::Publisher m_pointCloudPub;
    ros::Publisher m_extendedPointCloudPub;
    ros::Publisher m_extendedPointCloudOrientationPub;
    ros::Publisher m_binaryMapPub;
    ros::Publisher m_particlesPub;
    ros::Publisher m_oldParticlesPub;
    ros::Publisher m_colAvgPub;
    ros::Publisher m_polarGridPub;
    ros::Publisher m_polarCellYawPub;
    ros::Publisher m_obstaclesPub;
    ros::Publisher m_roiPub;
    ros::Publisher m_pointCloudInObstaclePub;
    
    ros::Subscriber m_pointCloudSub;
    
    tf::StampedTransform m_lastMapOdomTransform;
    uint32_t m_currentId;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloud;
};

}

#endif // POLARGRIDTRACKINGROS_H
