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
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <boost/foreach.hpp>
#include <boost/graph/graph_concepts.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <queue>

#include "utilspolargridtracking.h"

using namespace std;

namespace voxel_grid_tracking {
    
VoxelGridTracking::VoxelGridTracking()
{
    m_pointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // TODO: Get from params
    m_cameraParams.minX = 0.0;
    m_cameraParams.minY = 0.0;
    m_cameraParams.width = 1244;
    m_cameraParams.height = 370;
    m_cameraParams.u0 = 604.081;
    m_cameraParams.v0 = 180.507;
    m_cameraParams.ku = 707.049;
    m_cameraParams.kv = 707.049;
    m_cameraParams.distortion = 0;
    m_cameraParams.baseline = 0.472539;
    m_cameraParams.R = Eigen::MatrixXd(3, 3);
    m_cameraParams.R << 0.999984, -0.00501274, -0.00271074,
                        0.00500201, 0.99998, -0.00395038,
                        0.00273049, 0.00393676, 0.999988;
    m_cameraParams.t = Eigen::MatrixXd(3, 1);
    m_cameraParams.t << 0.0598969, -1.00137, 0.00463762;
    
    m_minX = -6.0;
    m_maxX = 6.0;
    m_minY = 0.0;
    m_maxY = 12.0;
    m_minZ = -0.4;
    m_maxZ = 3.5;
    
    m_cellSizeX = 0.25;
    m_cellSizeY = 0.25;
    m_cellSizeZ = 0.5;

    m_maxVelX = 5.0;
    m_maxVelY = 5.0;
    m_maxVelZ = 0.0;
    
    if (m_maxVelZ != 0.0) {
        ROS_WARN("The max speed expected for the z axis is %f. Are you sure you expect this behaviour?", m_maxVelZ);
    }

    m_particlesPerCell = 10000;
    m_threshProbForCreation = 0.2;
    
    m_neighBorX = 1;
    m_neighBorY = 1;
    m_neighBorZ = 1;
    
    m_threshYaw = 45.0 * M_PI / 180.0;
    m_threshPitch = 9999999.0; //0.0;
    m_threshMagnitude = 9999999.0;
    
    m_minVoxelsPerObstacle = 2;
    m_minObstacleDensity = 20.0;
    m_minVoxelDensity = 10.0;
    
    // SPEED_METHOD_MEAN, SPEED_METHOD_CIRC_HIST
    m_speedMethod = SPEED_METHOD_CIRC_HIST;
    
    m_yawInterval = 5.0 * M_PI / 180.0;
    m_pitchInterval = 2 * M_PI;
    
    m_baseFrame = DEFAULT_BASE_FRAME;
    // TODO: End of TODO
    
    m_initialized = false;
    
    m_dimX = (m_maxX - m_minX) / m_cellSizeX;
    m_dimY = (m_maxY - m_minY) / m_cellSizeY;
    m_dimZ = (m_maxZ - m_minZ) / m_cellSizeZ;
    
    m_grid.resize(boost::extents[m_dimX][m_dimY][m_dimZ]);
    m_colors.resize(boost::extents[m_dimX][m_dimY][m_dimZ][3]);
    
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                m_grid[x][y][z] = Voxel(x, y, z, 
                                        (x + 0.5) * m_cellSizeX + m_minX,
                                        (y + 0.5) * m_cellSizeY + m_minY,
                                        (z + 0.5) * m_cellSizeZ + m_minZ,
                                        m_cellSizeX, m_cellSizeY, m_cellSizeZ, 
                                        m_maxVelX, m_maxVelY, m_maxVelZ, 
                                        m_cameraParams, m_speedMethod,
                                        m_yawInterval, m_pitchInterval);
                for (uint32_t c = 0; c < 3; c++) {
                    m_colors[x][y][z][c] = (double)rand() / RAND_MAX;
                }
            }
        }
    }
    
    m_obstacleColors.resize(boost::extents[MAX_OBSTACLES_VISUALIZATION][3]);
    for (uint32_t i = 0; i < MAX_OBSTACLES_VISUALIZATION; i++) {
        for (uint32_t c = 0; c < 3; c++) {
            m_obstacleColors[i][c] = (double)rand() / RAND_MAX;
        }
    }
    
    m_lastMapOdomTransform.stamp_ = ros::Time(-1);
    ros::NodeHandle nh("~");
// //         m_deltaTimeSub = nh.subscribe<std_msgs::Float64>("deltaTime", 1, boost::bind(&VoxelGridTracking::deltaTimeCallback, this, _1));    
    m_pointCloudSub = nh.subscribe<sensor_msgs::PointCloud2>("pointCloud", 1, boost::bind(&VoxelGridTracking::pointCloudCallback, this, _1));
    
    m_voxelsPub = nh.advertise<visualization_msgs::MarkerArray>("voxels", 1);
    m_particlesPub = nh.advertise<geometry_msgs::PoseArray> ("particles", 1);
    m_particlesPositionPub = nh.advertise<sensor_msgs::PointCloud2> ("particlesPosition", 1);
    m_pointsPerVoxelPub = nh.advertise<sensor_msgs::PointCloud2> ("pointPerVoxel", 1);
    m_mainVectorsPub = nh.advertise<visualization_msgs::MarkerArray>("mainVectors", 1);
    m_obstaclesPub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 1);
    
//     ros::spin();
}

void VoxelGridTracking::start()
{

    tf::StampedTransform lastMapOdomTransform;
    lastMapOdomTransform.stamp_ = ros::Time(-1);
    
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Rate rate(15.0);
    while (ros::ok()) {
        try{
            while ((! listener.waitForTransform ("/map", "/odom", ros::Time(0), ros::Duration(10.0), ros::Duration(0.01))) && (ros::ok()));
            
            if (! ros::ok())
                break;
            
            listener.lookupTransform("/map", "/odom", ros::Time(0), transform);
            if (lastMapOdomTransform.stamp_ != ros::Time(-1)) {
                if (lastMapOdomTransform.stamp_ != transform.stamp_) {
                    double yaw, yaw1, yaw2, pitch, roll;
                    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw2);
                    tf::Matrix3x3(lastMapOdomTransform.getRotation()).getRPY(roll, pitch, yaw1);
                    yaw = yaw2 - yaw1;
                    
                    double deltaX = lastMapOdomTransform.getOrigin().getX() - transform.getOrigin().getX();
                    double deltaY = lastMapOdomTransform.getOrigin().getY() - transform.getOrigin().getY();
                    
                    m_deltaTime = transform.stamp_.toSec() - lastMapOdomTransform.stamp_.toSec();
                    
                    double deltaS = sqrt(deltaX * deltaX + deltaY * deltaY);
                    double speed = 0.0;
                    if (deltaS != 0.0) {
                        speed = deltaS / m_deltaTime;
                    }
                    
                    cout << "Transformation Received!!!!" << endl;
                    cout << "yaw " << yaw << endl;
                    cout << "speed " << speed << endl;
                    cout << "m_deltaTime " << m_deltaTime << endl;
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
        
//         ros::spinOnce();
        rate.sleep();
    }
    
//     ros::spin();
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
    
    getMeasurementModel();
    
    if (m_initialized) {
        prediction();
        measurementBasedUpdate();
        segment();
//         noiseRemoval();
        aggregation();
//         reconstructObjects(pointCloud);
    } 
//     publishParticles(m_oldParticlesPub, 2.0);
//     
    initialization();
//     
//     publishAll(pointCloud);
    END_CLOCK(totalCompute, startCompute)
    
    ROS_INFO("[%s] Total time: %f seconds", __FUNCTION__, totalCompute);
    
    publishVoxels();
    publishParticles();
    publishVoxels();
    publishParticles();
    publishMainVectors();
    publishObstacles();
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

void VoxelGridTracking::getMeasurementModel()
{
    
    // #pragma omp for schedule(dynamic)
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                Voxel & voxel = m_grid[x][y][z];
                const int sigmaX = voxel.sigmaX();
                const int sigmaY = voxel.sigmaY();
                const int sigmaZ = voxel.sigmaZ();
                
                uint32_t totalOccupied = 0;
                for (uint32_t x1 = max(0, (int)(x - sigmaX)); x1 <= min((int)(m_dimX - 1), (int)(x + sigmaX)); x1++) {
                    for (uint32_t y1 = max(0, (int)(y - sigmaY)); y1 <= min((int)(m_dimY - 1), (int)(y + sigmaY)); y1++) {
                        for (uint32_t z1 = max(0, (int)(z - sigmaZ)); z1 <= min((int)(m_dimZ - 1), (int)(z + sigmaZ)); z1++) {
                            totalOccupied += m_grid[x1][y1][z1].occupied()? 1 : 0;
                        }
                    }
                }
                
                // p(m(x,z) | occupied)
                const double occupiedProb = (double)totalOccupied / ((2.0 * (double)sigmaX + 1.0) + (2.0 * (double)sigmaY + 1.0) + (2.0 * (double)sigmaZ + 1.0));
                voxel.setOccupiedProb(occupiedProb);
            }
        }
    }
}

void VoxelGridTracking::initialization()
{
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                Voxel & voxel = m_grid[x][y][z];
                
                const double & occupiedProb = voxel.occupiedProb();
                // FIXME: Is it really important the fact that it is occupied or not?
                if (voxel.occupied() && voxel.empty() && (occupiedProb > m_threshProbForCreation)) {
                    const uint32_t numParticles = m_particlesPerCell * occupiedProb / 2.0;
                    voxel.createParticles(numParticles);
                }
            }
        }
    }
    
    m_initialized = true;
}

void VoxelGridTracking::particleToVoxel(const Particle3d & particle, 
                                        int32_t & posX, int32_t & posY, int32_t & posZ)
{
    const double dPosX = (particle.x() - m_minX) / m_cellSizeX;
    const double dPosY = (particle.y() - m_minY) / m_cellSizeY;
    const double dPosZ = (particle.z() - m_minZ) / m_cellSizeZ;
    
    posX = (dPosX < 0.0)? -1 : dPosX;
    posY = (dPosY < 0.0)? -1 : dPosY;
    posZ = (dPosZ < 0.0)? -1 : dPosZ;
}

void VoxelGridTracking::prediction()
{
    const double dx = m_deltaSpeed * m_deltaTime * cos(m_deltaYaw); // / m_cellSizeX;
    const double dy = m_deltaSpeed * m_deltaTime * cos(m_deltaYaw); // / m_cellSizeX;
    const double dz = 0.0;
    
    Eigen::MatrixXd R(6, 6);
    Eigen::VectorXd t(6, 1);
    Eigen::MatrixXd stateTransition(6, 6);
    R << cos(m_deltaYaw), -sin(m_deltaYaw), 0, 0, 0, 0,
         sin(m_deltaYaw), cos(m_deltaYaw), 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0,
        0, 0, 0, cos(m_deltaYaw), -sin(m_deltaYaw), 0,
        0, 0, 0, sin(m_deltaYaw), cos(m_deltaYaw), 0,
        0, 0, 0, 0, 0, 1;
    
    t << dx, dy, dz, 0, 0, 0;
    stateTransition << 1, 0, 0, m_deltaTime, 0, 0,
                        0, 1, 0, 0, m_deltaTime, 0,
                        0, 0, 1, 0, 0, m_deltaTime,
                        0, 0, 0, 1, 0, 0,
                        0, 0, 0, 0, 1, 0,
                        0, 0, 0, 0, 0, 1;
    
    // TODO: Put correct values for deltaX, deltaY, deltaZ, deltaVX, deltaVY, deltaVZ in class Particle, based on the covariance matrix
    vector <Particle3d> newParticles;
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                Voxel & voxel = m_grid[x][y][z];
                voxel.transformParticles(R, t, stateTransition, newParticles);
                voxel.clearParticles();
            }
        }
    }
    
    BOOST_FOREACH(const Particle3d & particle, newParticles) {
        int32_t xPos, yPos, zPos;
        particleToVoxel(particle, xPos, yPos, zPos);
        
        if ((xPos >= 0) && (xPos < m_dimX) &&
            (yPos >= 0) && (yPos < m_dimY) &&
            (zPos >= 0) && (zPos < m_dimZ)) {                    
        
            if (m_grid[xPos][yPos][zPos].occupied())
                m_grid[xPos][yPos][zPos].addParticle(particle);
        }
    }
}

void VoxelGridTracking::measurementBasedUpdate()
{
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                Voxel & voxel = m_grid[x][y][z];
            
                if (! voxel.empty()) {
                    voxel.setOccupiedPosteriorProb(m_particlesPerCell);
                    const double Nrc = voxel.occupiedPosteriorProb() * m_particlesPerCell;
                    const double fc = Nrc / voxel.numParticles();
                    
                    if (fc > 1.0) {
                        const double Fn = floor(fc);       // Integer part
                        const double Ff = fc - Fn;         // Fractional part
                        
                        const uint32_t numParticles = voxel.numParticles();
                        for (uint32_t i = 0; i < numParticles; i++) {
                            
                            const Particle3d & p = voxel.getParticle(i);
                            
                            for (uint32_t k = 1; k < Fn; k++)
                                voxel.makeCopy(p);
                            
                            const double r = (double)rand() / (double)RAND_MAX;
                            if (r < Ff)
                                voxel.makeCopy(p);
                        }
                    } else if (fc < 1.0) {
                        const uint32_t numParticles = voxel.numParticles();
                        for (uint32_t i = 0; i < numParticles; i++) {
                            const double r = (double)rand() / (double)RAND_MAX;
                            if (r > fc)
                                voxel.removeParticle(i);
                        }
                    }
                    
                    voxel.setMainVectors();
                }
            }
        }
    }
}

void VoxelGridTracking::segment()
{
    m_obstacles.clear();
    
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                Voxel & voxel = m_grid[x][y][z];
                if (! voxel.empty()) {
                    if (! voxel.assignedToObstacle()) {
                        
                        std::deque<Voxel> voxelsQueue;
                        
                        VoxelObstacle obst(m_obstacles.size(), m_threshYaw, m_threshPitch, m_threshMagnitude, m_minVoxelDensity, m_speedMethod);
                        if (! obst.addVoxelToObstacle(voxel))
                            continue;
                        
                        voxelsQueue.push_back(voxel);
                        
                        while (! voxelsQueue.empty()) {
                            Voxel & currVoxel = voxelsQueue.back();
                            voxelsQueue.pop_back();
                            
                            for (uint32_t x1 = max(0, (int)(currVoxel.x() - m_neighBorX)); x1 <= min(m_dimX - 1, (int)currVoxel.x() + m_neighBorX); x1++) {
                                for (uint32_t y1 = max(0, (int)(currVoxel.y() - m_neighBorY)); y1 <= min(m_dimY - 1, (int)currVoxel.y() + m_neighBorY); y1++) {
                                    for (uint32_t z1 = max(0, (int)(currVoxel.z() - m_neighBorZ)); z1 <= min(m_dimZ - 1, (int)currVoxel.z() + m_neighBorZ); z1++) {
                                        Voxel & newVoxel = m_grid[x1][y1][z1];
                                        if ((! newVoxel.assignedToObstacle()) && (! newVoxel.empty())) {
                                            
                                            if (obst.addVoxelToObstacle(newVoxel))
                                                voxelsQueue.push_back(newVoxel);
                                        }
                                    }
                                }
                            }
                        }
                        
                        m_obstacles.push_back(obst);
                    }
                }
            }
        }
    }
}

void VoxelGridTracking::aggregation()
{
    ObstacleList::iterator it = m_obstacles.begin();
    while (it != m_obstacles.end()) {
        bool joined = false;
        if (it->numVoxels() <= m_minVoxelsPerObstacle) {
            for (ObstacleList::iterator it2 = m_obstacles.begin(); it2 != m_obstacles.end(); it2++) {
                if (it->isObstacleConnected(*it2)) {
                    it2->joinObstacles(*it);
                    it = m_obstacles.erase(it);
                    joined = true;
                    
                    break;
                }
            }
        }
        if (! joined)
            it++;
    }
}

void VoxelGridTracking::noiseRemoval()
{
    ObstacleList::iterator it = m_obstacles.begin();
    while (it != m_obstacles.end()) {
//         cout << "density " << it->density() << endl;
        if (it->density() < m_minObstacleDensity)
            it = m_obstacles.erase(it);
        else
            it++;
    }
}



void VoxelGridTracking::publishVoxels()
{
    visualization_msgs::MarkerArray voxelMarkers;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vizPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    uint32_t idCount = 0;
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                const Voxel & voxel = m_grid[x][y][z];
                
                if (voxel.occupied()) {
                    visualization_msgs::Marker voxelMarker;
                    voxelMarker.header.frame_id = m_baseFrame;
                    voxelMarker.header.stamp = ros::Time();
                    voxelMarker.id = idCount++;
                    voxelMarker.ns = "voxels";
                    voxelMarker.type = visualization_msgs::Marker::CUBE;
                    voxelMarker.action = visualization_msgs::Marker::ADD;

                    voxelMarker.pose.position.x = voxel.centroidX();
                    voxelMarker.pose.position.y = voxel.centroidY();
                    voxelMarker.pose.position.z = voxel.centroidZ();
                    
                    voxelMarker.pose.orientation.x = 0.0;
                    voxelMarker.pose.orientation.y = 0.0;
                    voxelMarker.pose.orientation.z = 0.0;
                    voxelMarker.pose.orientation.w = 1.0;
                    voxelMarker.scale.x = m_cellSizeX;
                    voxelMarker.scale.y = m_cellSizeY;
                    voxelMarker.scale.z = m_cellSizeZ;
                    voxelMarker.color.r = m_colors[x][y][z][0];
                    voxelMarker.color.g = m_colors[x][y][z][1];
                    voxelMarker.color.b = m_colors[x][y][z][2];
//                     voxelMarker.color.a = 0.2;
//                     voxelMarker.color.r = 255;
//                     voxelMarker.color.g = 0;
//                     voxelMarker.color.b = 0;
                    voxelMarker.color.a = voxel.occupiedProb();
                    
                    BOOST_FOREACH(const pcl::PointXYZRGB & point, voxel.getPoints()->points) {
                        pcl::PointXYZRGB tmpPoint;
                        
                        tmpPoint.x = point.x;
                        tmpPoint.y = point.y;
                        tmpPoint.z = point.z;
                        tmpPoint.r = point.r;
                        tmpPoint.g = point.g;
                        tmpPoint.b = point.b;
//                         tmpPoint.r = voxelMarker.color.r * 255;
//                         tmpPoint.g = voxelMarker.color.g * 255;
//                         tmpPoint.b = voxelMarker.color.b * 255;
                        
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

void VoxelGridTracking::publishParticles()
{
    geometry_msgs::PoseArray particles;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr particlePointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    particles.header.frame_id = m_baseFrame;
    particles.header.stamp = ros::Time();
    
    uint32_t idCount = 0;
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                const Voxel & voxel = m_grid[x][y][z];
                
                for (uint32_t i = 0; i < voxel.numParticles(); i++) {
                    const Particle3d & particle = voxel.getParticle(i);
                    geometry_msgs::Pose pose;
                    
                    pose.position.x = particle.x();
                    pose.position.y = particle.y();
                    pose.position.z = particle.z();
                    
                    const double & vx = particle.vx();
                    const double & vy = particle.vy();
                    const double & vz = particle.vz();
                    
                    const double & normYaw = sqrt(vx * vx + vy * vy);
                    const double & normPitch = sqrt(vy * vy + vz * vz);
                    
                    double yaw = acos(vx / normYaw);
                    if (vy < 0)
                        yaw = -yaw;
                    
                    double pitch = asin(vz / normPitch);
                    if (vy < 0)
                        pitch = -pitch;
                    
                    const tf::Quaternion & quat = tf::createQuaternionFromRPY(0.0, pitch, yaw);
                    pose.orientation.w = quat.w();
                    pose.orientation.x = quat.x();
                    pose.orientation.y = quat.y();
                    pose.orientation.z = quat.z();
                    
                    particles.poses.push_back(pose);
                
                    pcl::PointXYZRGB tmpPoint;
                    
                    tmpPoint.x = particle.x();
                    tmpPoint.y = particle.y();
                    tmpPoint.z = particle.z();
                    tmpPoint.r = m_colors[x][y][z][0] * 255;
                    tmpPoint.g = m_colors[x][y][z][1] * 255;
                    tmpPoint.b = m_colors[x][y][z][2] * 255;
                    
                    particlePointCloud->push_back(tmpPoint);
                }
            }
        }
    }
    
    m_particlesPub.publish(particles);
    
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg (*particlePointCloud, cloudMsg);
    cloudMsg.header.frame_id = m_baseFrame;
    cloudMsg.header.stamp = ros::Time();
    
    m_particlesPositionPub.publish(cloudMsg);
}

void VoxelGridTracking::publishMainVectors()
{
    visualization_msgs::MarkerArray mainVectors;
    
    uint32_t idCount = 0;
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                const Voxel & voxel = m_grid[x][y][z];
                
                if (! voxel.empty()) {
                    
                    visualization_msgs::Marker mainVector;
                    mainVector.header.frame_id = "left_cam";
                    mainVector.header.stamp = ros::Time();
                    mainVector.id = idCount++;
                    mainVector.ns = "mainVectors";
                    mainVector.type = visualization_msgs::Marker::ARROW;
                    mainVector.action = visualization_msgs::Marker::ADD;
                    
                    mainVector.pose.orientation.x = 0.0;
                    mainVector.pose.orientation.y = 0.0;
                    mainVector.pose.orientation.z = 0.0;
                    mainVector.pose.orientation.w = 1.0;
                    mainVector.scale.x = 0.01;
                    mainVector.scale.y = 0.03;
                    mainVector.scale.z = 0.1;
                    mainVector.color.a = 1.0;
                    mainVector.color.r = m_colors[x][y][z][0];
                    mainVector.color.g = m_colors[x][y][z][1];
                    mainVector.color.b = m_colors[x][y][z][2];
                    
                    //         orientation.lifetime = ros::Duration(5.0);
                    
                    geometry_msgs::Point origin, dest;
                    origin.x = voxel.centroidX();
                    origin.y = voxel.centroidY();
                    origin.z = voxel.centroidZ();
                    
                    
                    dest.x = voxel.centroidX() + voxel.vx() * m_deltaTime;
                    dest.y = voxel.centroidY() + voxel.vy() * m_deltaTime;
                    dest.z = voxel.centroidZ() + voxel.vz() * m_deltaTime;
                    
                    mainVector.points.push_back(origin);
                    mainVector.points.push_back(dest);
                    
                    mainVectors.markers.push_back(mainVector);
                }
            }
        }
    }
    
    m_mainVectorsPub.publish(mainVectors);
}

void VoxelGridTracking::publishObstacles()
{
    visualization_msgs::MarkerArray voxelCleaners;
    
    for (uint32_t i = 0; i < MAX_OBSTACLES_VISUALIZATION; i++) {
        visualization_msgs::Marker voxelMarker;
        voxelMarker.header.frame_id = m_baseFrame;
        voxelMarker.header.stamp = ros::Time();
        voxelMarker.id = i;
        voxelMarker.ns = "obstacles";
        voxelMarker.type = visualization_msgs::Marker::CUBE;
        voxelMarker.action = visualization_msgs::Marker::DELETE;
        
        voxelCleaners.markers.push_back(voxelMarker);
    }
    
    m_obstaclesPub.publish(voxelCleaners);
    
    visualization_msgs::MarkerArray voxelMarkers;

    uint32_t idCount = 0;
    
    for (uint32_t i = 0; i < m_obstacles.size(); i++) {
        const vector<Voxel> & voxels = m_obstacles[i].voxels();
        BOOST_FOREACH(const Voxel & voxel, voxels) {
            if (! voxel.empty()) {
                visualization_msgs::Marker voxelMarker;
                voxelMarker.header.frame_id = m_baseFrame;
                voxelMarker.header.stamp = ros::Time();
                voxelMarker.id = idCount++;
                voxelMarker.ns = "obstacles";
                voxelMarker.type = visualization_msgs::Marker::CUBE;
                voxelMarker.action = visualization_msgs::Marker::ADD;
                
                voxelMarker.pose.position.x = voxel.centroidX();
                voxelMarker.pose.position.y = voxel.centroidY();
                voxelMarker.pose.position.z = voxel.centroidZ();
                
                voxelMarker.pose.orientation.x = 0.0;
                voxelMarker.pose.orientation.y = 0.0;
                voxelMarker.pose.orientation.z = 0.0;
                voxelMarker.pose.orientation.w = 1.0;
                voxelMarker.scale.x = m_cellSizeX;
                voxelMarker.scale.y = m_cellSizeY;
                voxelMarker.scale.z = m_cellSizeZ;
                voxelMarker.color.r = m_obstacleColors[i % MAX_OBSTACLES_VISUALIZATION][0];
                voxelMarker.color.g = m_obstacleColors[i % MAX_OBSTACLES_VISUALIZATION][1];
                voxelMarker.color.b = m_obstacleColors[i % MAX_OBSTACLES_VISUALIZATION][2];
                voxelMarker.color.a = 0.6;
                //                     voxelMarker.color.r = 255;
                //                     voxelMarker.color.g = 0;
                //                     voxelMarker.color.b = 0;
//                 voxelMarker.color.a = voxel.occupiedProb();
                
                voxelMarkers.markers.push_back(voxelMarker);
            }
        }
    }
    
    m_obstaclesPub.publish(voxelMarkers);
}


}
