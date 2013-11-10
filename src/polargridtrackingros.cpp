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


#include "polargridtrackingros.h"

#include<pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/GridCells.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"
#include <tf/transform_datatypes.h>

#include <boost/foreach.hpp>

namespace polar_grid_tracking {
    
PolarGridTrackingROS::PolarGridTrackingROS(const uint32_t& rows, const uint32_t& cols, const double& cellSizeX, 
                                           const double& cellSizeZ, const double& maxVelX, const double& maxVelZ, 
                                           const t_Camera_params& cameraParams, const double& particlesPerCell, 
                                           const double& threshProbForCreation): 
                                            PolarGridTracking(rows, cols, cellSizeX, cellSizeZ, maxVelX, maxVelZ, cameraParams, 
                                                              particlesPerCell, threshProbForCreation)
{
    ros::NodeHandle nh("~");
    m_pointCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("pointCloud", 1);
    m_binaryMapPub = nh.advertise<nav_msgs::GridCells> ("binaryMap", 1);
    m_particlesPub = nh.advertise<geometry_msgs::PoseArray> ("particles", 1);
    m_oldParticlesPub = nh.advertise<geometry_msgs::PoseArray> ("oldParticles", 1);
    m_colAvgPub = nh.advertise<geometry_msgs::PoseArray> ("colAvg", 1);
    
//     ros::spin();
}

void PolarGridTrackingROS::compute(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud)
{
//     PolarGridTracking::compute(pointCloud);
    getBinaryMapFromPointCloud(pointCloud);
    
//     drawBinaryMap(map);
//     drawTopDownMap(pointCloud);
    
    getMeasurementModel();
    
    if (m_initialized) {
        prediction();
        measurementBasedUpdate();
    } 
    publishParticles(m_oldParticlesPub, 2.0);
    initialization();
    
    
//     if (! m_initialized) {
//     } else {
//         publishParticles(m_oldParticlesPub);
//         prediction();
//     }
    
    publishAll(pointCloud);
}

void PolarGridTrackingROS::publishAll(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud)
{
        publishPointCloud(pointCloud);
        publishBinaryMap();
        publishParticles(m_particlesPub, 1.0);
        publishColumnAverage(0.5);
}

void PolarGridTrackingROS::publishPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud)
{
    pcl::PointCloud< pcl::PointXYZRGB >::Ptr tmpPointCloud(new pcl::PointCloud< pcl::PointXYZRGB >);
    for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = pointCloud->begin(); 
        it != pointCloud->end(); it++) {
        
        const pcl::PointXYZRGB & point = *it;
        pcl::PointXYZRGB newPoint;
    
        newPoint.x = point.x;
        newPoint.y = point.z;
        newPoint.z = point.y;
        newPoint.r = point.r;
        newPoint.g = point.g;
        newPoint.b = point.b;
        
        tmpPointCloud->push_back(newPoint);
    }

    
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg (*tmpPointCloud, cloudMsg);
    cloudMsg.header.frame_id="left_cam";
    
    m_pointCloudPub.publish(cloudMsg);
    
    ros::spinOnce();
}

void PolarGridTrackingROS::publishBinaryMap()
{
    nav_msgs::GridCells gridCells;
    gridCells.header.frame_id = "left_cam";
    gridCells.header.stamp = ros::Time();
    gridCells.cell_width = m_cellSizeX;
    gridCells.cell_height = m_cellSizeZ;

    const double maxZ = m_grid.rows() * m_cellSizeZ;
    const double maxX = m_grid.cols() / 2.0 * m_cellSizeX;
    const double minX = -maxX;
    
    const double factorX = m_grid.cols() / (maxX - minX);
    const double factorZ = m_grid.rows() / maxZ;
    
    const double offsetX = (m_grid.cols() * m_cellSizeX) / 2.0;
    
    uint32_t cellIdx = 0;
    for (uint32_t i = 0; i < m_map.rows(); i++) {
        for (int j = 0; j < m_map.cols(); j++) {
            if (m_map(i, j)) {
                geometry_msgs::Point point;
                point.x = j * m_cellSizeX - offsetX;
                point.y = i * m_cellSizeZ;
                point.z = 0;
                gridCells.cells.push_back(point);
            }
        }
    }
    
    m_binaryMapPub.publish(gridCells);
}

void PolarGridTrackingROS::publishParticles(ros::Publisher & particlesPub, const double & zPlane)
{
    geometry_msgs::PoseArray particles;
    
    particles.header.frame_id = "left_cam";
    particles.header.stamp = ros::Time();
    
    const double offsetX = ((m_grid.cols() + 1) * m_cellSizeX) / 2.0;
    const double offsetZ = m_cellSizeZ / 2.0;
    
    for (uint32_t z = 0; z < m_grid.rows(); z++) {
        for (uint32_t x = 0; x < m_grid.cols(); x++) {
            const Cell & cell = m_grid(z, x);
            for (uint32_t i = 0; i < cell.numParticles(); i++) {
                const Particle & particle = cell.getParticle(i);
                geometry_msgs::Pose pose;
                
                pose.position.x = particle.x() - offsetX;
                pose.position.y = particle.z() - offsetZ;
                pose.position.z = zPlane;
                
                const double & vx = particle.vx();
                const double & vz = particle.vz();
                                
                const double & norm = sqrt(vx * vx + vz * vz);
                
                double yaw = acos(vx / norm);
                if (vz < 0)
                    yaw = -yaw;
                
                const tf::Quaternion & quat = tf::createQuaternionFromRPY(0.0, 0.0, yaw);
                pose.orientation.w = quat.w();
                pose.orientation.x = quat.x();
                pose.orientation.y = quat.y();
                pose.orientation.z = quat.z();
                
                particles.poses.push_back(pose);
            }
        }
    }
    
    particlesPub.publish(particles);
}

void PolarGridTrackingROS::publishColumnAverage(const double & zPlane)
{
    geometry_msgs::PoseArray colAverages;
    
    colAverages.header.frame_id = "left_cam";
    colAverages.header.stamp = ros::Time();
    
    const double offsetX = (m_grid.cols() * m_cellSizeX) / 2.0;
    
    for (uint32_t x = 0; x < m_grid.cols(); x++) {
        for (uint32_t z = 0; z < m_grid.rows(); z++) {
            if (m_map(z, x)) {
                
                geometry_msgs::Pose pose;
                
                pose.position.x = x * m_cellSizeX - offsetX;
                pose.position.y = z * m_cellSizeZ;
                pose.position.z = zPlane;
                
                double vx, vz;
                const vector<Particle> & particles = m_grid(z, x).getParticles();
                if (particles.size() != 0) {
                    BOOST_FOREACH(const Particle & particle, particles) {
                        vx += particle.vx();
                        vz += particle.vz();
                    }
                    vx /= particles.size();
                    vz /= particles.size();
                    
                    const double & norm = sqrt(vx * vx + vz * vz);
                    
                    double yaw = acos(vx / norm);
                    if (vz < 0)
                        yaw = -yaw;
                    
                    const tf::Quaternion & quat = tf::createQuaternionFromRPY(0.0, 0.0, yaw);
                    pose.orientation.w = quat.w();
                    pose.orientation.x = quat.x();
                    pose.orientation.y = quat.y();
                    pose.orientation.z = quat.z();
                    
                    colAverages.poses.push_back(pose);
                }
                
                break;
            }
        }
    }
    
    m_colAvgPub.publish(colAverages);
}


}