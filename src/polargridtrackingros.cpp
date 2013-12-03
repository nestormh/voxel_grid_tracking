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
#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseArray.h"
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

#include "utils.h"

#include <boost/foreach.hpp>

namespace polar_grid_tracking {
    
PolarGridTrackingROS::PolarGridTrackingROS(const uint32_t& rows, const uint32_t& cols, const double& cellSizeX, 
                                           const double& cellSizeZ, const double& maxVelX, const double& maxVelZ, 
                                           const t_Camera_params& cameraParams, const double& particlesPerCell, 
                                           const double& threshProbForCreation, 
                                           const double & gridDepthFactor, const uint32_t &  gridColumnFactor, const double & yawInterval,
                                           const double & threshYaw, const double & threshMagnitude): 
                                            PolarGridTracking(rows, cols, cellSizeX, cellSizeZ, maxVelX, maxVelZ, cameraParams, 
                                                              particlesPerCell, threshProbForCreation, 
                                                              gridDepthFactor, gridColumnFactor, yawInterval,
                                                              threshYaw, threshMagnitude)
{
    ros::NodeHandle nh("~");
    m_pointCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("pointCloud", 1);
    m_extendedPointCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("extendedPointCloud", 1);
    m_extendedPointCloudOrientationPub = nh.advertise<geometry_msgs::PoseArray> ("pointCloudOrientation", 1);
    m_binaryMapPub = nh.advertise<nav_msgs::GridCells> ("binaryMap", 1);
    m_particlesPub = nh.advertise<geometry_msgs::PoseArray> ("particles", 1);
    m_oldParticlesPub = nh.advertise<geometry_msgs::PoseArray> ("oldParticles", 1);
    m_colAvgPub = nh.advertise<geometry_msgs::PoseArray> ("colAvg", 1);
    m_polarGridPub = nh.advertise<visualization_msgs::Marker>("polarGrid", 10);
    m_polarCellYawPub = nh.advertise<geometry_msgs::PoseArray> ("polarCellYaw", 1);
    m_obstaclesPub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 10);
    
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
        reconstructObjects(pointCloud);
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
//         publishPointCloud(pointCloud);
        publishBinaryMap();
        publishParticles(m_particlesPub, 1.0);
        publishColumnAverage(0.5);
}

void PolarGridTrackingROS::publishPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = pointCloud->begin(); 
        it != pointCloud->end(); it++) {
        
        const pcl::PointXYZRGB & point = *it;
        pcl::PointXYZRGB newPoint;
    
        newPoint.x = point.x - m_cellSizeX / 2.0;
        newPoint.y = point.z - m_cellSizeZ / 2.0;
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


void PolarGridTrackingROS::publishPointCloud(const pcl::PointCloud< PointXYZRGBDirected >::Ptr & pointCloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (pcl::PointCloud<PointXYZRGBDirected>::const_iterator it = pointCloud->begin();
            it != pointCloud->end(); it++) {

        const PointXYZRGBDirected & point = *it;
        pcl::PointXYZRGB newPoint;

        newPoint.x = point.x - m_cellSizeX / 2.0;
        newPoint.y = point.z - m_cellSizeZ / 2.0;
        newPoint.z = point.y;
        newPoint.r = point.r;
        newPoint.g = point.g;
        newPoint.b = point.b;

        tmpPointCloud->push_back(newPoint);
    }


    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg (*tmpPointCloud, cloudMsg);
    cloudMsg.header.frame_id="left_cam";

    m_extendedPointCloudPub.publish(cloudMsg);

    ros::spinOnce();
}

void PolarGridTrackingROS::publishPointCloudOrientation(const pcl::PointCloud< PointXYZRGBDirected >::Ptr & pointCloud)
{
//     ros::Publisher & particlesPub, const double & zPlane
    geometry_msgs::PoseArray poses;
    
    poses.header.frame_id = "left_cam";
    poses.header.stamp = ros::Time();
    
    const double offsetX = ((m_grid.cols() + 1) * m_cellSizeX) / 2.0;
    const double offsetZ = m_cellSizeZ / 2.0;
    
    for (pcl::PointCloud<PointXYZRGBDirected>::const_iterator it = pointCloud->begin();
         it != pointCloud->end(); it++) {
        
        const PointXYZRGBDirected & point = *it;
        geometry_msgs::Pose pose;
        
        pose.position.x = point.x - m_cellSizeX / 2.0;
        pose.position.y = point.z - m_cellSizeZ / 2.0;
        pose.position.z = point.y;
    
        const tf::Quaternion & quat = tf::createQuaternionFromRPY(0.0, 0.0, point.yaw);
        pose.orientation.w = quat.w();
        pose.orientation.x = quat.x();
        pose.orientation.y = quat.y();
        pose.orientation.z = quat.z();
        
        poses.poses.push_back(pose);
    }
    
    m_extendedPointCloudOrientationPub.publish(poses);
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
                m_grid(z, x).getMainVectors(vx, vz);
                const vector<Particle> & particles = m_grid(z, x).getParticles();
                if (particles.size() != 0) {
                    
                    const double & norm = sqrt(vx * vx + vz * vz);
                    
                    if (norm == 0.0) continue;
                    
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

void PolarGridTrackingROS::reconstructObjects(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud)
{   
    pcl::PointCloud< PointXYZRGBDirected >::Ptr extendedPointCloud;
    resetPolarGrid();
    extendPointCloud(pointCloud, extendedPointCloud);
    
    generateObstacles();
        
    publishPolarGrid();
    publishPointCloud(extendedPointCloud);
    publishPointCloudOrientation(extendedPointCloud);
    publishPolarCellYaw(1.0);
    publishObstacles();
}

void PolarGridTrackingROS::publishPolarGrid()
{
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "left_cam";
    line_list.header.stamp = ros::Time::now();
//     line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    
    line_list.scale.x = 0.01;
    
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;
    
    // Create the vertices for the points and lines
    double z = (double)(m_cameraParams.ku * m_cameraParams.baseline) / m_cameraParams.width;
    
    const double maxDepth = m_grid.rows() * m_cellSizeZ * (m_gridDepthFactor + 1.0);
    const double fb = m_cameraParams.ku * m_cameraParams.baseline;
    const double maxCol = m_cameraParams.u0 - m_cameraParams.width - 1;
    while (z <= maxDepth) {
        const double d = fb / z;
        const double xMin = m_cameraParams.baseline * m_cameraParams.u0 / d;
        const double xMax = m_cameraParams.baseline * maxCol / d;
        
        geometry_msgs::Point p1, p2;
        p1.x = xMin;
        p2.x = xMax;
        p1.y = p2.y = z;
        p1.z = p2.z = 0;
        
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        
        z *= 1.0 + m_gridDepthFactor;
    }
    
    z /= 1.0 + m_gridDepthFactor;
    
    const double dMin = fb / z;
    for (uint32_t i = 0; i < m_cameraParams.width - 1; i += m_gridColumnFactor) {
        const double x = m_cameraParams.baseline * (m_cameraParams.u0 - i) / dMin;
        
        geometry_msgs::Point p1, p2;
        p1.x = x;
        p1.y = z;
        p1.z = 0;
        
        p2.x = p2.y = p2.z = 0;
        
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
    }
    
    m_polarGridPub.publish(line_list);
    
}

void PolarGridTrackingROS::publishPolarCellYaw(const double & zPlane)
{
    geometry_msgs::PoseArray polarCellYaw;
    
    polarCellYaw.header.frame_id = "left_cam";
    polarCellYaw.header.stamp = ros::Time();
    
    const double z0 = (double)(m_cameraParams.ku * m_cameraParams.baseline) / m_cameraParams.width;
    const double fb = m_cameraParams.ku * m_cameraParams.baseline;
    
    for (uint32_t r = 0; r < m_polarGrid.rows(); r++) {
        for (uint32_t c = 0; c < m_polarGrid.cols(); c++) {
            if (m_polarGrid(r, c).getNumVectors() != 0) {
                
                geometry_msgs::Pose pose;
                
                pose.position.y = z0 * pow(1.0 + m_gridDepthFactor, r + 0.5);
                const double dMin = fb / pose.position.y;
                const double u = m_gridColumnFactor * (c + 2.5);
                pose.position.x = pose.position.y * (m_cameraParams.u0 - u) / m_cameraParams.ku;
                pose.position.z = zPlane;
                
                const double yaw = m_polarGrid(r, c).getYaw();
                    
                const tf::Quaternion & quat = tf::createQuaternionFromRPY(0.0, 0.0, yaw);
                pose.orientation.w = quat.w();
                pose.orientation.x = quat.x();
                pose.orientation.y = quat.y();
                pose.orientation.z = quat.z();
                
                polarCellYaw.poses.push_back(pose);
            }
        }
    }
    
    m_polarCellYawPub.publish(polarCellYaw);
}

void PolarGridTrackingROS::publishObstacles()
{
    visualization_msgs::MarkerArray obstacles;
    
    const double z0 = (double)(m_cameraParams.ku * m_cameraParams.baseline) / m_cameraParams.width;
    const double fb = m_cameraParams.ku * m_cameraParams.baseline;
    
    for (uint32_t i = 0; i < m_obstacles.size(); i++) {
        const vector<PolarCell> & cells = m_obstacles[i].cells();
        if (cells.size() <= 1)
            continue;
        
        visualization_msgs::Marker triangles;
        
        triangles.header.frame_id = "left_cam";
        triangles.header.stamp = ros::Time::now();
        triangles.action = visualization_msgs::Marker::ADD;
        triangles.pose.orientation.w = 1.0;
        
        triangles.id = i;
        triangles.type = visualization_msgs::Marker::TRIANGLE_LIST;
        
        triangles.scale.x = 1.0;
        triangles.scale.y = 1.0;
        triangles.scale.z = 1.0;
        
        triangles.color.r = (double)rand() / RAND_MAX;
        triangles.color.g = (double)rand() / RAND_MAX;
        triangles.color.b = (double)rand() / RAND_MAX;
        if (cells.size() > 1)
            triangles.color.a = 1.0;
        else
            triangles.color.a = 0.5;
                
        BOOST_FOREACH(const PolarCell & cell, cells) {
        
            geometry_msgs::Point p1, p2, p3, p4;
            
            const double row = cell.row() - 0.5;
            const double col = cell.col() + 2.0;

            p1.y = z0 * pow(1.0 + m_gridDepthFactor, row);
            const double u1 = m_gridColumnFactor * col;
            p1.x = p1.y * (m_cameraParams.u0 - u1) / m_cameraParams.ku;
            p1.z = 0.0;

            p2.y = z0 * pow(1.0 + m_gridDepthFactor, row + 1.0);
            const double u2 = m_gridColumnFactor * col;
            p2.x = p2.y * (m_cameraParams.u0 - u2) / m_cameraParams.ku;
            p2.z = 0.0;
            
            p3.y = z0 * pow(1.0 + m_gridDepthFactor, row);
            const double u3 = m_gridColumnFactor * (col + 1.0);
            p3.x = p3.y * (m_cameraParams.u0 - u3) / m_cameraParams.ku;
            p3.z = 0.0;
            
            p4.y = z0 * pow(1.0 + m_gridDepthFactor, row + 1.0);
            const double u4 = m_gridColumnFactor * (col + 1.0);
            p4.x = p4.y * (m_cameraParams.u0 - u4) / m_cameraParams.ku;
            p4.z = 0.0;
            
            triangles.points.push_back(p1);
            triangles.points.push_back(p2);
            triangles.points.push_back(p3);
            
            triangles.points.push_back(p4);
            triangles.points.push_back(p2);
            triangles.points.push_back(p3);
        }
        
        obstacles.markers.push_back(triangles);
    }
    
    m_obstaclesPub.publish(obstacles);
}

}