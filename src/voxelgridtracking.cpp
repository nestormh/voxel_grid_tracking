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


#include "voxelgridtracking.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <boost/foreach.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>

#include "utilspolargridtracking.h"

using namespace std;

namespace voxel_grid_tracking {
    
VoxelGridTracking::VoxelGridTracking()
{
    m_pointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    ros::NodeHandle nh("~");
    m_deltaTimeSub = nh.subscribe<std_msgs::Float64>("deltaTime", 1, boost::bind(&VoxelGridTracking::deltaTimeCallback, this, _1));    
    m_pointCloudSub = nh.subscribe<sensor_msgs::PointCloud2>("pointCloud", 1, boost::bind(&VoxelGridTracking::pointCloudCallback, this, _1));
    
    m_voxelsPub = nh.advertise<visualization_msgs::MarkerArray>("voxels", 1);
    m_pointsPerVoxelPub = nh.advertise<sensor_msgs::PointCloud2> ("pointPerVoxel", 1);
    
    // TODO: Get from params
    m_minX = -6.0;
    m_maxX = 6.0;
    m_minY = 0.0;
    m_maxY = 12.0;
    m_minZ = 0.0;
    m_maxZ = 3.5;
    
    m_cellSizeX = 0.25;
    m_cellSizeY = 0.25;
    m_cellSizeZ = 0.25;

    m_maxVelX = 5.0;
    m_maxVelY = 5.0;
    m_maxVelZ = 0.0;
    
    if (m_maxVelZ != 0.0) {
        ROS_WARN("The max speed expected for the z axis is %f. Are you sure you expect this behaviour?", m_maxVelZ);
    }

    m_particlesPerCell = 1000;
    m_threshProbForCreation = 0.9999; //0.2;
    
    m_baseFrame = DEFAULT_BASE_FRAME;
    // TODO: End of TODO
    
    
    m_dimX = (m_maxX - m_minX) / m_cellSizeX;
    m_dimY = (m_maxY - m_minY) / m_cellSizeY;
    m_dimZ = (m_maxZ - m_minZ) / m_cellSizeZ;
    
    m_grid.resize(boost::extents[m_dimX][m_dimY][m_dimZ]);

    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                m_grid[x][y][z] = Voxel(x, y, z, m_cellSizeX, m_cellSizeY, m_cellSizeZ, 
                                        m_maxVelX, m_maxVelY, m_maxVelZ, m_cameraParams);
            }
        }
    }
}

void VoxelGridTracking::start()
{

    tf::StampedTransform lastMapOdomTransform;
    lastMapOdomTransform.stamp_ = ros::Time(-1);
    
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Rate rate(10.0);
    while (ros::ok()) {
        try{
            while (! listener.waitForTransform ("/map", "/odom", ros::Time(0), ros::Duration(10.0), ros::Duration(0.01)));
            
            listener.lookupTransform("/map", "/odom", ros::Time(0), transform);
            if (lastMapOdomTransform.stamp_ != ros::Time(-1)) {
                if (lastMapOdomTransform.stamp_ != transform.stamp_) {
                    double yaw, yaw1, yaw2, pitch, roll;
                    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw2);
                    tf::Matrix3x3(lastMapOdomTransform.getRotation()).getRPY(roll, pitch, yaw1);
                    yaw = yaw2 - yaw1;
                    
                    double deltaX = lastMapOdomTransform.getOrigin().getX() - transform.getOrigin().getX();
                    double deltaY = lastMapOdomTransform.getOrigin().getY() - transform.getOrigin().getY();
                    
                    double deltaS = sqrt(deltaX * deltaX + deltaY * deltaY);
                    double speed = 0.0;
                    if (deltaS != 0.0) {
                        speed = deltaS / m_deltaTime;
                    }
                    
                    cout << "Transformation Received!!!!" << endl;
                    setDeltaYawSpeedAndTime(yaw, speed, m_deltaTime);
                    rate.sleep();
                    
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                    pcl::copyPointCloud(*m_pointCloud, *pointCloud);
                    
                    compute(pointCloud);
                }
            }
            lastMapOdomTransform = transform;
            ros::spinOnce();
        } catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
        
        rate.sleep();
    }
    
    ros::spin();
}
void VoxelGridTracking::deltaTimeCallback(const std_msgs::Float64::ConstPtr& msg)
{
    m_deltaTime = msg.get()->data;
}

void VoxelGridTracking::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) 
{
    pcl::fromROSMsg<pcl::PointXYZRGB>(*msg, *m_pointCloud);
    cout << "Received point cloud with size = " << m_pointCloud->size() << endl;
}

void VoxelGridTracking::setDeltaYawSpeedAndTime(const double& deltaYaw, const double& deltaSpeed, const double& deltaTime)
{
    m_deltaYaw = deltaYaw;
    m_deltaSpeed = deltaSpeed;
    m_deltaTime = deltaTime;
}

void VoxelGridTracking::compute(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud)
{
    INIT_CLOCK(startCompute)
    reset();
    getVoxelGridFromPointCloud(pointCloud);
    
//     getMeasurementModel();
//     
//     if (m_initialized) {
//         prediction();
//         measurementBasedUpdate();
//         reconstructObjects(pointCloud);
//     } 
//     publishParticles(m_oldParticlesPub, 2.0);
//     
//     initialization();
//     
//     publishAll(pointCloud);
    END_CLOCK(totalCompute, startCompute)
    
    publishVoxels();
    
    ROS_INFO("[%s] Total time: %f seconds", __FUNCTION__, totalCompute);
}

void VoxelGridTracking::reset()
{
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                m_grid[x][y][z].reset();
            }
        }
    }
}


void VoxelGridTracking::getVoxelGridFromPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud)
{
    BOOST_FOREACH(pcl::PointXYZRGB& point, *pointCloud) {
        
        const uint32_t xPos = (point.x - m_minX) / m_cellSizeX;
        const uint32_t yPos = (point.y - m_minY) / m_cellSizeY;
        const uint32_t zPos = (point.z - m_minZ) / m_cellSizeZ;
        
        
        if ((xPos >= 0) && (xPos < m_dimX) &&
            (yPos >= 0) && (yPos < m_dimY) && 
            (zPos >= 0) && (zPos < m_dimZ)) {

            m_grid[xPos][yPos][zPos].addPoint(point);
        }
    }
    
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                Voxel & voxel = m_grid[x][y][z];
                
                voxel.update();
            }
        }
    }
}

void VoxelGridTracking::publishVoxels()
{
    cout << __FUNCTION__ << endl;
    visualization_msgs::MarkerArray voxelMarkers;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vizPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    uint32_t idCount = 0;
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                const Voxel & voxel = m_grid[x][y][z];
                
                if (voxel.occupied() || ((x == 0) && (y == 0) && (z == 0))) {
                    visualization_msgs::Marker voxelMarker;
                    voxelMarker.header.frame_id = m_baseFrame;
                    voxelMarker.header.stamp = ros::Time();
                    voxelMarker.id = idCount++;
                    voxelMarker.ns = "voxels";
                    voxelMarker.type = visualization_msgs::Marker::CUBE;
                    voxelMarker.action = visualization_msgs::Marker::ADD;

                    voxelMarker.pose.position.x = (x + 0.5) * m_cellSizeX + m_minX + voxel.centroidX();
                    voxelMarker.pose.position.y = (y + 0.5) * m_cellSizeY + m_minY + voxel.centroidY();
                    voxelMarker.pose.position.z = (z + 0.5) * m_cellSizeZ + m_minZ + voxel.centroidZ();
                    
                    voxelMarker.pose.orientation.x = 0.0;
                    voxelMarker.pose.orientation.y = 0.0;
                    voxelMarker.pose.orientation.z = 0.0;
                    voxelMarker.pose.orientation.w = 1.0;
                    voxelMarker.scale.x = m_cellSizeX;
                    voxelMarker.scale.y = m_cellSizeY;
                    voxelMarker.scale.z = m_cellSizeZ;
                    voxelMarker.color.a = 0.4;
                    voxelMarker.color.r = (double)rand() / RAND_MAX;
                    voxelMarker.color.g = (double)rand() / RAND_MAX;
                    voxelMarker.color.b = (double)rand() / RAND_MAX;
                    
                    BOOST_FOREACH(const pcl::PointXYZRGB & point, voxel.getPoints()->points) {
                        pcl::PointXYZRGB tmpPoint;
                        
                        tmpPoint.x = point.x;
                        tmpPoint.y = point.y;
                        tmpPoint.z = point.z;
                        tmpPoint.r = point.r; //voxelMarker.color.r * 255;
                        tmpPoint.g = point.g; //voxelMarker.color.g * 255;
                        tmpPoint.b = point.b; //voxelMarker.color.b * 255;
                        
                        vizPointCloud->push_back(tmpPoint);
                    }
                    
                    voxelMarkers.markers.push_back(voxelMarker);
                }
            }
        }
    }

    m_voxelsPub.publish(voxelMarkers);
    
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg (*vizPointCloud, cloudMsg);
    cloudMsg.header.frame_id = m_baseFrame;
    cloudMsg.header.stamp = ros::Time();
    
    m_pointsPerVoxelPub.publish(cloudMsg);
}

}