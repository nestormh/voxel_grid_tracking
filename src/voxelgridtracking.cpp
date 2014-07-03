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

#include "PolarGridTracking/roiArray.h"
#include "utilspolargridtracking.h"

using namespace std;

namespace voxel_grid_tracking {
    
VoxelGridTracking::VoxelGridTracking()
{
    m_pointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    m_fakePointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    
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
    
    m_minX = 0.0;
    m_maxX = 24.0;
    m_minY = -8.0;
    m_maxY = 8.0;
    m_minZ = 0.0;
    m_maxZ = 3.5; //3.5;
    
    m_cellSizeX = 0.25;
    m_cellSizeY = 0.25;
    m_cellSizeZ = 0.5;

    m_maxVelX = 3.0;
    m_maxVelY = 3.0;
    m_maxVelZ = 0.0;
    
    if (m_maxVelZ != 0.0) {
        ROS_WARN("The max speed expected for the z axis is %f. Are you sure you expect this behaviour?", m_maxVelZ);
    }

    m_particlesPerVoxel = 1000;
    m_threshProbForCreation = 0.0; //0.2;
    
    m_neighBorX = 1;
    m_neighBorY = 1;
    m_neighBorZ = 1;
    
    m_threshYaw = 90.0 * M_PI / 180.0;
    m_threshPitch = 9999999.0; //0.0;
    m_threshMagnitude = 9999999.0;
    
    m_minVoxelsPerObstacle = 2;
    m_minObstacleDensity = 20.0;
    m_minVoxelDensity = 10.0;
    m_maxCommonVolume = 0.8;
    
    // SPEED_METHOD_MEAN, SPEED_METHOD_CIRC_HIST
    m_speedMethod = SPEED_METHOD_CIRC_HIST;
    
    m_obstacleSpeedMethod = SPEED_METHOD_MEAN;
    
    m_yawInterval = 5.0 * M_PI / 180.0;
    m_pitchInterval = 2 * M_PI;
    
    m_minObstacleHeight = 1.25;
    m_maxObstacleHeight = 2.0;
    
    m_baseFrame = DEFAULT_BASE_FRAME;
    
    m_timeIncrementForFakePointCloud = 10.0;
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
    
    m_particleColors.resize(boost::extents[MAX_PARTICLE_AGE_REPRESENTATION]);
    m_particleColors[0] = cv::Scalar(255, 0, 0);   // Blue
    m_particleColors[1] = cv::Scalar(0, 255, 0);   // Green
    m_particleColors[2] = cv::Scalar(0, 0, 255);   // Red
    m_particleColors[3] = cv::Scalar(255, 255, 0);  // Cyan
    m_particleColors[4] = cv::Scalar(255, 0, 255);  // Magenta
    m_particleColors[5] = cv::Scalar(0, 255, 255);   // Yellow
    m_particleColors[6] = cv::Scalar(0, 0, 0);        // Black
    m_particleColors[7] = cv::Scalar(255, 255, 255);    // White
    
    m_lastMapOdomTransform.stamp_ = ros::Time(-1);
    ros::NodeHandle nh("~");
    
    nh.param<string>("map_frame", m_mapFrame, "/map");
    nh.param<string>("pose_frame", m_poseFrame, "/base_footprint");
    nh.param<string>("camera_frame", m_cameraFrame, "/base_left_cam");
    
// //         m_deltaTimeSub = nh.subscribe<std_msgs::Float64>("deltaTime", 1, boost::bind(&VoxelGridTracking::deltaTimeCallback, this, _1));    
//     m_pointCloudSub = nh.subscribe<sensor_msgs::PointCloud2>("pointCloud", 0, boost::bind(&VoxelGridTracking::pointCloudCallback, this, _1));
    
    m_pointCloudSub.subscribe(nh, "pointCloud", 0);
    m_tfPointCloudSync.reset(new TfPointCloudSynchronizer(m_pointCloudSub, m_tfListener, m_mapFrame, 0));
    m_tfPointCloudSync->registerCallback( boost::bind(&VoxelGridTracking::pointCloudCallback, this, _1));
    
    m_voxelsPub = nh.advertise<visualization_msgs::MarkerArray>("voxels", 1);
    m_particlesPub = nh.advertise<visualization_msgs::MarkerArray> ("particles", 1);
    m_pointsPerVoxelPub = nh.advertise<sensor_msgs::PointCloud2> ("pointPerVoxel", 1);
    m_mainVectorsPub = nh.advertise<visualization_msgs::MarkerArray>("mainVectors", 1);
    m_obstaclesPub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 1);
    m_obstacleCubesPub = nh.advertise<visualization_msgs::MarkerArray>("cubes", 1);
    m_obstacleSpeedPub = nh.advertise<visualization_msgs::MarkerArray>("obstacleSpeed", 1);
    m_obstacleSpeedTextPub = nh.advertise<visualization_msgs::MarkerArray>("obstacleSpeedText", 1);
    m_ROIPub = nh.advertise<PolarGridTracking::roiArray>("roiArray", 1);
    m_fakePointCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("fakePointCloud", 1);
//     ros::spin();
}

void VoxelGridTracking::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) 
{
//     cout << "CALLBACK" << endl;
//     cout << "Pointcloud " << msg->header.stamp << endl;
    try {
        m_tfListener.lookupTransform(m_mapFrame, m_poseFrame, ros::Time(0), m_pose2MapTransform);
    } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
//     cout << "TF " << m_pose2MapTransform.stamp_ << endl;
    m_deltaTime = (msg->header.stamp - m_lastPointCloudTime).toSec();
//     cout << "m_deltaTime = " << msg->header.stamp << " - " << m_lastPointCloudTime << " = " << m_deltaTime << endl;
    
    pcl::fromROSMsg<pcl::PointXYZRGB>(*msg, *m_pointCloud);
    cout << "Received point cloud " << msg->header.seq << " with size = " << m_pointCloud->size() << endl;
    m_currentId = msg->header.seq;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*m_pointCloud, *pointCloud);
    
    m_lastPointCloudTime = msg->header.stamp;
    
    compute(pointCloud);
}

void VoxelGridTracking::compute(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud)
{
    INIT_CLOCK(startCompute)
    // TODO: Remove particles reset
    reset();
    getVoxelGridFromPointCloud(pointCloud);
    getMeasurementModel();
    
    if (m_initialized) {
        prediction();
        publishParticles();
        measurementBasedUpdate();
//         segment();
//         
//         updateObstacles();
//         filterObstacles();
//         updateSpeedFromObstacles();
    } 
    initialization();
//     publishParticles(m_oldParticlesPub, 2.0);
    
//     initialization();

    END_CLOCK(totalCompute, startCompute)
    
    ROS_INFO("[%s] Total time: %f seconds", __FUNCTION__, totalCompute);
    
    INIT_CLOCK(startVis)
    publishVoxels();
//     publishParticles();
    publishMainVectors();
//     publishObstacles();
//     publishObstacleCubes();
//     publishROI();
//     publishFakePointCloud();
//     visualizeROI2d();
    END_CLOCK(totalVis, startVis)
    
    ROS_INFO("[%s] Total visualization time: %f seconds", __FUNCTION__, totalVis);
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
                                
                const int & sigmaX = voxel.sigmaX();
                const int & sigmaY = voxel.sigmaY();
                const int & sigmaZ = voxel.sigmaZ();
                
                if (voxel.occupied()) {
                    for (uint32_t x1 = max(0, (int)(x - sigmaX)); x1 <= min((int)(m_dimX - 1), (int)(x + sigmaX)); x1++) {
                        for (uint32_t y1 = max(0, (int)(y - sigmaY)); y1 <= min((int)(m_dimY - 1), (int)(y + sigmaY)); y1++) {
                            for (uint32_t z1 = max(0, (int)(z - sigmaZ)); z1 <= min((int)(m_dimZ - 1), (int)(z + sigmaZ)); z1++) {
                                m_grid[x1][y1][z1].incNeighborOcc();
                            }
                        }
                    }
                }
            }
        }
    }
       
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                Voxel & voxel = m_grid[x][y][z];
                const int & sigmaX = voxel.sigmaX();
                const int & sigmaY = voxel.sigmaY();
                const int & sigmaZ = voxel.sigmaZ();
                
                // p(m(x,z) | occupied)
                const double occupiedProb = (double)voxel.neighborOcc() / ((2.0 * (double)sigmaX + 1.0) + (2.0 * (double)sigmaY + 1.0) + (2.0 * (double)sigmaZ + 1.0));
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

                    // TODO The number of generated particles depends on the occupancy probability
                    const uint32_t numParticles = m_particlesPerVoxel * occupiedProb; // / 2.0;
                    voxel.createParticles(numParticles, m_pose2MapTransform);
                }
            }
        }
    }
    
    m_initialized = true;
}

inline void VoxelGridTracking::particleToVoxel(const Particle3d & particle, 
                                               int32_t & posX, int32_t & posY, int32_t & posZ)
{
    tf::Vector3 point = m_pose2MapTransform.inverse() * tf::Vector3(particle.x(), particle.y(), particle.z());
    
    const double dPosX = (point[0] - m_minX) / m_cellSizeX;
    const double dPosY = (point[1] - m_minY) / m_cellSizeY;
    const double dPosZ = (point[2] - m_minZ) / m_cellSizeZ;
    
    posX = (dPosX < 0.0)? -1 : dPosX;
    posY = (dPosY < 0.0)? -1 : dPosY;
    posZ = (dPosZ < 0.0)? -1 : dPosZ;
}

void VoxelGridTracking::prediction()
{
//     const double speed = 0.0; //m_speed;
//     const double deltaYaw = 0.0; //m_deltaYaw;
//     const double deltaPitch = 0.0; //m_deltaPitch;
//     
//     const double dx = -speed * m_deltaTime * cos(deltaYaw); // / m_cellSizeX;
//     const double dy = -speed * m_deltaTime * cos(deltaYaw); // / m_cellSizeX;
//     const double dz = -0.0;
//     
//     Eigen::MatrixXd R(6, 6);
//     Eigen::VectorXd t(6, 1);
    Eigen::MatrixXd stateTransition(6, 6);
//     R << cos(deltaYaw), -sin(deltaYaw), 0, 0, 0, 0,
//          sin(deltaYaw), cos(deltaYaw), 0, 0, 0, 0,
//         0, 0, 1, 0, 0, 0,
//         0, 0, 0, cos(deltaYaw), -sin(deltaYaw), 0,
//         0, 0, 0, sin(deltaYaw), cos(deltaYaw), 0,
//         0, 0, 0, 0, 0, 1;
//     
//     t << dx, dy, dz, 0, 0, 0;
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
                voxel.transformParticles(stateTransition, newParticles);
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
                
                if ((! voxel.empty()) && (voxel.occupied())) {
                    voxel.sortParticles();
                    voxel.setMainVectors(m_deltaX, m_deltaY, m_deltaZ);
                }
            }
        }
    }
    
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                Voxel & voxel = m_grid[x][y][z];
            
                if ((! voxel.empty()) && (voxel.occupied())) {
                    voxel.setOccupiedPosteriorProb(m_particlesPerVoxel);
                    const double Nrc = voxel.occupiedPosteriorProb() * m_particlesPerVoxel;
                    const double fc = Nrc / voxel.numParticles();
                    
                    if (fc > 1.0) {
                        const double Fn = floor(fc);       // Integer part
                        const double Ff = fc - Fn;         // Fractional part
                        
                        for (uint32_t i = 0; i < voxel.numParticles(); i++) {
                            
                            const Particle3d & p = voxel.getParticle(i);
                            
                            for (uint32_t k = 1; k < Fn; k++)
                                for (uint32_t n = 0; n < p.age(); n++)
                                    voxel.makeCopy(p);
                            
                            const double r = (double)rand() / (double)RAND_MAX;
                            if (r < Ff)
                                voxel.makeCopy(p);
                        }
                    } else if (fc < 1.0) {
                        for (uint32_t i = 0; i < voxel.numParticles(); i++) {
                            const double r = (double)rand() / (double)RAND_MAX;
                            if (r > fc)
                                voxel.removeParticle(i);
                        }
                    }
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
                        
                        VoxelObstacle obst(m_obstacles.size(), m_threshYaw, m_threshPitch, m_threshMagnitude, m_minVoxelDensity, m_obstacleSpeedMethod, m_yawInterval, m_pitchInterval);
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
                        
                        if (obst.numVoxels() > m_minVoxelsPerObstacle)
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
//             for (ObstacleList::iterator it2 = m_obstacles.begin(); it2 != m_obstacles.end(); it2++) {
//                 if (it->isObstacleConnected(*it2)) {
//                     it2->joinObstacles(*it);
                    it = m_obstacles.erase(it);
                    joined = true;
//                     
//                     break;
//                 }
//             }
        }
        if (! joined)
            it++;
    }
}

void VoxelGridTracking::noiseRemoval()
{
    ObstacleList::iterator it = m_obstacles.begin();
    while (it != m_obstacles.end()) {
        bool erased = false;
        if (it->numVoxels() <= m_minVoxelsPerObstacle) {
            it = m_obstacles.erase(it);
            erased = true;
        }
        if (! erased)
            it++;
    }
}

void VoxelGridTracking::updateObstacles()
{
    BOOST_FOREACH(VoxelObstacle & obstacle, m_obstacles) {
        obstacle.update(m_cellSizeX, m_cellSizeY, m_cellSizeZ);
    }
}

void VoxelGridTracking::joinCommonVolumes()
{
    ObstacleList::iterator it1 = m_obstacles.begin();
    uint32_t counter = 0;
    while (it1 != m_obstacles.end()) {
        bool joined = false;
        const double volume1 = (it1->maxX() - it1->minX()) * (it1->maxY() - it1->minY()) * (it1->maxZ() - it1->minZ());
        
        for (ObstacleList::iterator it2 = m_obstacles.begin(); it2 != m_obstacles.end(); it2++) {
            if (it1 == it2)
                continue;
            
            const double & commonVolume = VoxelObstacle::commonVolume(*it1, *it2);
            
            if (commonVolume > 0.0) {
                const double volume2 = (it2->maxX() - it2->minX()) * (it2->maxY() - it2->minY()) * (it2->maxZ() - it2->minZ());
                    
                const double volumePercent = commonVolume / min(volume1, volume2);
                
                if (commonVolume >= m_maxCommonVolume) {
                    
                    it1->joinObstacles(*it2);
                    it1->update(m_cellSizeX, m_cellSizeY, m_cellSizeZ);
                    it2 = m_obstacles.erase(it2);
                    if (it2 != m_obstacles.begin())
                        it2--;
                }
            }
        }

        if (! joined)
            it1++;
    }
}

void VoxelGridTracking::updateSpeedFromObstacles()
{
    BOOST_FOREACH(VoxelObstacle & obstacle, m_obstacles)
        obstacle.updateSpeed(m_deltaX, m_deltaY, m_deltaZ);
}

void VoxelGridTracking::filterObstacles()
{
    ObstacleList::iterator it = m_obstacles.begin();
    while (it != m_obstacles.end()) {
//         if (((it->centerZ() - (it->sizeZ() / 2.0)) > m_minZ) || 
//             ((it->centerZ() + (it->sizeZ() / 2.0)) < 0.75)) {
        if (it->sizeZ() < m_minObstacleHeight) {
            it = m_obstacles.erase(it);
//         }  else if (fabs(it->centerZ() - (it->sizeZ() / 2.0) - m_minZ) > m_cellSizeZ) {
//             it = m_obstacles.erase(it);
        } else {
            it++;
        }
//         bool joined = false;
//         if (it->numVoxels() <= m_minVoxelsPerObstacle) {
//             it = m_obstacles.erase(it);
//             joined = true;
//         }
//         if (! joined)
//             it++;
    }
}

void VoxelGridTracking::generateFakePointClouds()
{
    m_fakePointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    for (uint32_t i = 0; i < m_obstacles.size(); i++) {
        const VoxelObstacle & obstacle = m_obstacles[i];
        
        if ((obstacle.minZ() - (m_cellSizeZ / 2.0)) == m_minZ) {
            
//             const double & tColission = min(obstacle.centerX() / (m_deltaX - obstacle.vx()), 1.5);
//             const double deltaTime = tColission / m_timeIncrementForFakePointCloud;

            const double tColission = 1.0;
            const double deltaTime = 0.3;
            
            BOOST_FOREACH(const Voxel & voxel, obstacle.voxels()) {
                BOOST_FOREACH(const pcl::PointXYZRGB & point, voxel.getPoints()->points) {
                    for (double t = 0; t <= tColission; t += deltaTime) {
    //                     double t = 1.0;
                    
                        pcl::PointXYZRGB newPoint;
                        newPoint.x = point.x - obstacle.vx() * t;
                        newPoint.y = point.y + obstacle.vy() * t;
                        newPoint.z = point.z + obstacle.vz() * t;
    //                     newPoint.r = m_obstacleColors[i % MAX_OBSTACLES_VISUALIZATION][0] * 255;
    //                     newPoint.g = m_obstacleColors[i % MAX_OBSTACLES_VISUALIZATION][1] * 255;
    //                     newPoint.b = m_obstacleColors[i % MAX_OBSTACLES_VISUALIZATION][2] * 255;
                        newPoint.r = point.r;
                        newPoint.g = point.g;
                        newPoint.b = point.b;
                        
                        m_fakePointCloud->push_back(newPoint);                        
                    }
                }
            }
        }
    }
}

void VoxelGridTracking::publishVoxels()
{
    visualization_msgs::MarkerArray voxelMarkers;
    
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr vizPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    uint32_t idCount = 0;
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                const Voxel & voxel = m_grid[x][y][z];
                
                if (voxel.occupiedProb() > 0.0) {
                    visualization_msgs::Marker voxelMarker;
                    voxelMarker.header.frame_id = m_poseFrame;
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
                    
//                     BOOST_FOREACH(const pcl::PointXYZRGB & point, voxel.getPoints()->points) {
//                         pcl::PointXYZRGB tmpPoint;
//                         
//                         tmpPoint.x = point.x;
//                         tmpPoint.y = point.y;
//                         tmpPoint.z = point.z;
//                         tmpPoint.r = point.r;
//                         tmpPoint.g = point.g;
//                         tmpPoint.b = point.b;
// //                         tmpPoint.r = voxelMarker.color.r * 255;
// //                         tmpPoint.g = voxelMarker.color.g * 255;
// //                         tmpPoint.b = voxelMarker.color.b * 255;
//                         
//                         vizPointCloud->push_back(tmpPoint);
//                     }
                    voxelMarkers.markers.push_back(voxelMarker);
                }
            }
        }
    }

    m_voxelsPub.publish(voxelMarkers);
    
//     sensor_msgs::PointCloud2 cloudMsg;
//     pcl::toROSMsg (*vizPointCloud, cloudMsg);
//     cloudMsg.header.frame_id = m_baseFrame;
//     cloudMsg.header.stamp = ros::Time();
//     
//     m_pointsPerVoxelPub.publish(cloudMsg);
}

void VoxelGridTracking::publishParticles()
{
//     geometry_msgs::PoseArray particles;
//     
//     particles.header.frame_id = m_mapFrame;
//     particles.header.stamp = ros::Time();
//         
//     uint32_t idCount = 0;
//     for (uint32_t x = 0; x < m_dimX; x++) {
//         for (uint32_t y = 0; y < m_dimY; y++) {
//             for (uint32_t z = 0; z < m_dimZ; z++) {
//                 const Voxel & voxel = m_grid[x][y][z];
//                 
//                 for (uint32_t i = 0; i < voxel.numParticles(); i++) {
//                     const Particle3d & particle = voxel.getParticle(i);
//                     geometry_msgs::Pose pose;
//                     
//                     pose.position.x = particle.x();
//                     pose.position.y = particle.y();
//                     pose.position.z = particle.z();
//                     
//                     const double & vx = particle.vx();
//                     const double & vy = particle.vy();
//                     const double & vz = particle.vz();
//                     
//                     const tf::Quaternion & quat = particle.getQuaternion();
//                     pose.orientation.w = quat.w();
//                     pose.orientation.x = quat.x();
//                     pose.orientation.y = quat.y();
//                     pose.orientation.z = quat.z();
//                     
//                     particles.poses.push_back(pose);
//                 }
//             }
//         }
//     }
//     
//     m_particlesPub.publish(particles);
    
    visualization_msgs::MarkerArray particlesCleaners;
    
    for (uint32_t i = 0; i < 1000; i++) {
        for (uint32_t age = 0; age < MAX_PARTICLE_AGE_REPRESENTATION; age++) {
            stringstream ss;
            ss << "age_" << age;
            
            visualization_msgs::Marker voxelMarker;
            voxelMarker.header.frame_id = m_mapFrame;
            voxelMarker.header.stamp = ros::Time();
            voxelMarker.id = i;
            voxelMarker.ns = ss.str();
            voxelMarker.type = visualization_msgs::Marker::ARROW;
            voxelMarker.action = visualization_msgs::Marker::DELETE;
            
            particlesCleaners.markers.push_back(voxelMarker);
        }
    }
    
    m_particlesPub.publish(particlesCleaners);
    
    visualization_msgs::MarkerArray particles;
    
    uint32_t idCount = 0;
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                const Voxel & voxel = m_grid[x][y][z];
                
                for (uint32_t i = 0; i < voxel.numParticles(); i++) {
                    const Particle3d & particle = voxel.getParticle(i);
            
                    uint32_t age = particle.age();
                    if (age >= MAX_PARTICLE_AGE_REPRESENTATION)
                        age = MAX_PARTICLE_AGE_REPRESENTATION - 1;
                    
                    visualization_msgs::Marker particleVector;
                    particleVector.header.frame_id = m_mapFrame;
                    particleVector.header.stamp = ros::Time();
                    particleVector.id = idCount++;
                    stringstream ss;
                    ss << "age_" << age;
                    particleVector.ns = ss.str();
                    particleVector.type = visualization_msgs::Marker::ARROW;
                    particleVector.action = visualization_msgs::Marker::ADD;
                    
                    particleVector.pose.orientation.x = 0.0;
                    particleVector.pose.orientation.y = 0.0;
                    particleVector.pose.orientation.z = 0.0;
                    particleVector.pose.orientation.w = 1.0;
                    particleVector.scale.x = 0.01;
                    particleVector.scale.y = 0.03;
                    particleVector.scale.z = 0.1;
                    particleVector.color.a = 1.0;
                    particleVector.color.r = m_obstacleColors[age][2];
                    particleVector.color.g = m_obstacleColors[age][1];
                    particleVector.color.b = m_obstacleColors[age][0];
                    
                    //         orientation.lifetime = ros::Duration(5.0);
                    
                    const double & x = particle.x();
                    const double & y = particle.y();
                    const double & z = particle.z();
                    const double & vx = particle.vx();
                    const double & vy = particle.vy();
                    const double & vz = particle.vz();
                    
                    geometry_msgs::Point origin, dest;
                    origin.x = x;
                    origin.y = y;
                    origin.z = z;
                    
                    dest.x = x + vx * m_deltaTime;
                    dest.y = y + vy * m_deltaTime;
                    dest.z = z + vz * m_deltaTime;
                    
                    particleVector.points.push_back(origin);
                    particleVector.points.push_back(dest);
                    
                    particles.markers.push_back(particleVector);
                }
            }
        }
    }
    
    m_particlesPub.publish(particles);
}

void VoxelGridTracking::publishMainVectors()
{
    visualization_msgs::MarkerArray vectorCleaners;
    
    for (uint32_t i = 0; i < MAX_OBSTACLES_VISUALIZATION * 3; i++) {
        visualization_msgs::Marker voxelMarker;
        voxelMarker.header.frame_id = m_poseFrame;
        voxelMarker.header.stamp = ros::Time();
        voxelMarker.id = i;
        voxelMarker.ns = "mainVectors";
        voxelMarker.type = visualization_msgs::Marker::ARROW;
        voxelMarker.action = visualization_msgs::Marker::DELETE;
        
        vectorCleaners.markers.push_back(voxelMarker);
    }
    
    m_mainVectorsPub.publish(vectorCleaners);
    
    visualization_msgs::MarkerArray mainVectors;
    
    uint32_t idCount = 0;
    for (uint32_t x = 0; x < m_dimX; x++) {
        for (uint32_t y = 0; y < m_dimY; y++) {
            for (uint32_t z = 0; z < m_dimZ; z++) {
                const Voxel & voxel = m_grid[x][y][z];
                
                if (! voxel.empty()) {
                    
                    visualization_msgs::Marker mainVector;
                    mainVector.header.frame_id = m_poseFrame;
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
                    
                    cv::Vec3f color(voxel.vx(), voxel.vy(), voxel.vz());
                    if (cv::norm(color) != 0.0) {
                        color = color / cv::norm(color);

                        mainVector.color.r = fabs(color[0]);
                        mainVector.color.g = fabs(color[1]);
                        mainVector.color.b = fabs(color[2]);
                    } else {
                        mainVector.color.r = m_colors[x][y][z][0];
                        mainVector.color.g = m_colors[x][y][z][1];
                        mainVector.color.b = m_colors[x][y][z][2];
                    }
                    mainVector.color.a = 1.0;
                    
                    //         orientation.lifetime = ros::Duration(5.0);
                    
                    geometry_msgs::Point origin, dest;
                    origin.x = voxel.centroidX();
                    origin.y = voxel.centroidY();
                    origin.z = voxel.centroidZ();
                    
                    dest.x = voxel.centroidX() + voxel.vx() * m_deltaTime * 3.0;
                    dest.y = voxel.centroidY() + voxel.vy() * m_deltaTime * 3.0;
                    dest.z = voxel.centroidZ() + voxel.vz() * m_deltaTime * 3.0;
                    
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
    
    for (uint32_t i = 0; i < MAX_OBSTACLES_VISUALIZATION * 3; i++) {
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
//         if (m_obstacles[i].magnitude() == 0.0)
//             continue;
        
        const vector<Voxel> & voxels = m_obstacles[i].voxels();
        BOOST_FOREACH(const Voxel & voxel, voxels) {
//             if (! voxel.empty()) {
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
//             }
        }
    }
    
    m_obstaclesPub.publish(voxelMarkers);
}

void VoxelGridTracking::publishObstacleCubes()
{
    visualization_msgs::MarkerArray obstacleCubesCleaners;
    
    for (uint32_t i = 0; i < 1000; i++) {
        visualization_msgs::Marker obstacleCubeMarker;
        obstacleCubeMarker.header.frame_id = m_baseFrame;
        obstacleCubeMarker.header.stamp = ros::Time();
        obstacleCubeMarker.id = i;
        obstacleCubeMarker.ns = "obstacleCubes";
        obstacleCubeMarker.type = visualization_msgs::Marker::CUBE;
        obstacleCubeMarker.action = visualization_msgs::Marker::DELETE;
        
        obstacleCubesCleaners.markers.push_back(obstacleCubeMarker);
        
        obstacleCubeMarker.ns = "speedVector";
        obstacleCubeMarker.type = visualization_msgs::Marker::ARROW;
        obstacleCubeMarker.action = visualization_msgs::Marker::DELETE;
        
        obstacleCubesCleaners.markers.push_back(obstacleCubeMarker);
        
        obstacleCubeMarker.ns = "speedText";
        obstacleCubeMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        obstacleCubeMarker.action = visualization_msgs::Marker::DELETE;
        
        obstacleCubesCleaners.markers.push_back(obstacleCubeMarker);
    }
    
    m_obstacleCubesPub.publish(obstacleCubesCleaners);
    
    visualization_msgs::MarkerArray obstacleCubeMarkers;
    visualization_msgs::MarkerArray obstacleSpeedMarkers;
    visualization_msgs::MarkerArray obstacleSpeedTextMarkers;
    
    uint32_t idCount = 0;
    
    for (uint32_t i = 0; i < m_obstacles.size(); i++) {
        const VoxelObstacle & obstacle = m_obstacles[i];
        
        visualization_msgs::Marker obstacleCubeMarker;
        obstacleCubeMarker.header.frame_id = m_baseFrame;
        obstacleCubeMarker.header.stamp = ros::Time();
        obstacleCubeMarker.id = idCount++;
        obstacleCubeMarker.ns = "obstacleCubes";
        obstacleCubeMarker.type = visualization_msgs::Marker::CUBE;
        obstacleCubeMarker.action = visualization_msgs::Marker::ADD;
        
        obstacleCubeMarker.pose.position.x = obstacle.centerX();
        obstacleCubeMarker.pose.position.y = obstacle.centerY();
        obstacleCubeMarker.pose.position.z = obstacle.centerZ();
        
        obstacleCubeMarker.pose.orientation.x = 0.0;
        obstacleCubeMarker.pose.orientation.y = 0.0;
        obstacleCubeMarker.pose.orientation.z = 0.0;
        obstacleCubeMarker.pose.orientation.w = 1.0;
        obstacleCubeMarker.scale.x = obstacle.sizeX();
        obstacleCubeMarker.scale.y = obstacle.sizeY();
        obstacleCubeMarker.scale.z = obstacle.sizeZ();
        obstacleCubeMarker.color.r = m_obstacleColors[i % MAX_OBSTACLES_VISUALIZATION][0];
        obstacleCubeMarker.color.g = m_obstacleColors[i % MAX_OBSTACLES_VISUALIZATION][1];
        obstacleCubeMarker.color.b = m_obstacleColors[i % MAX_OBSTACLES_VISUALIZATION][2];
        obstacleCubeMarker.color.a = 0.4;
        
        obstacleCubeMarkers.markers.push_back(obstacleCubeMarker);
        
        // ********************************************************
        // Publication of the speed of the obstacle
        // ********************************************************
        
        visualization_msgs::Marker speedVector;
        speedVector.header.frame_id = m_baseFrame;
        speedVector.header.stamp = ros::Time();
        speedVector.id = idCount;
        speedVector.ns = "speedVector";
        speedVector.type = visualization_msgs::Marker::ARROW;
        speedVector.action = visualization_msgs::Marker::ADD;
        
        speedVector.pose.orientation.x = 0.0;
        speedVector.pose.orientation.y = 0.0;
        speedVector.pose.orientation.z = 0.0;
        speedVector.pose.orientation.w = 1.0;
        speedVector.scale.x = 0.01;
        speedVector.scale.y = 0.03;
        speedVector.scale.z = 0.1;
        speedVector.color.a = 1.0;
        speedVector.color.r = m_obstacleColors[i % MAX_OBSTACLES_VISUALIZATION][0];
        speedVector.color.g = m_obstacleColors[i % MAX_OBSTACLES_VISUALIZATION][1];
        speedVector.color.b = m_obstacleColors[i % MAX_OBSTACLES_VISUALIZATION][2];
        
        //         orientation.lifetime = ros::Duration(5.0);
        
        geometry_msgs::Point origin, dest;
        origin.x = obstacle.centerX();
        origin.y = obstacle.minY();
        origin.z = obstacle.centerZ();        

        // NOTE: This makes vectors longer than they actually are. 
        // For a realistic visualization, multiply by m_deltaTime
//         dest.x = obstacle.centerX() + obstacle.vx();
//         dest.y = obstacle.minY() + obstacle.vy();
//         dest.z = obstacle.centerZ() + obstacle.vz();
        
        dest.x = obstacle.centerX() + obstacle.vx() * m_deltaTime;
        dest.y = obstacle.minY() + obstacle.vy() * m_deltaTime;
        dest.z = obstacle.centerZ() + obstacle.vz() * m_deltaTime;

//         dest.x = obstacle.centerX() + obstacle.vx() / obstacle.numVoxels() * m_deltaTime;
//         dest.y = obstacle.minY() + obstacle.vy() / obstacle.numVoxels() * m_deltaTime;
//         dest.z = obstacle.centerZ() + obstacle.vz() / obstacle.numVoxels() * m_deltaTime;
        
        speedVector.points.push_back(origin);
        speedVector.points.push_back(dest);
        
        obstacleSpeedMarkers.markers.push_back(speedVector);
        
        // ********************************************************
        // Publication of the speed text of the obstacle
        // ********************************************************
        
        visualization_msgs::Marker speedTextVector;
        speedTextVector.header.frame_id = m_baseFrame;
        speedTextVector.header.stamp = ros::Time();
        speedTextVector.id = idCount;
        speedTextVector.ns = "speedText";
        speedTextVector.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        speedTextVector.action = visualization_msgs::Marker::ADD;
        
        speedTextVector.pose.position.x = obstacle.centerX();
        speedTextVector.pose.position.y = obstacle.centerY();
        speedTextVector.pose.position.z = obstacle.maxZ() + (m_cellSizeZ / 2.0);
        
        speedTextVector.pose.orientation.x = 0.0;
        speedTextVector.pose.orientation.y = 0.0;
        speedTextVector.pose.orientation.z = 0.0;
        speedTextVector.pose.orientation.w = 1.0;
        speedTextVector.scale.z = 0.25;
        speedTextVector.color.a = 1.0;
        speedTextVector.color.r = 0.0;
        speedTextVector.color.g = 1.0;
        speedTextVector.color.b = 0.0;
        
        stringstream ss;
        ss << std::setprecision(3) << obstacle.magnitude() << " m/s";
        speedTextVector.text = ss.str();
                
        obstacleSpeedTextMarkers.markers.push_back(speedTextVector);
    }
    
    m_obstacleCubesPub.publish(obstacleCubeMarkers);
    m_obstacleSpeedPub.publish(obstacleSpeedMarkers);
    m_obstacleSpeedTextPub.publish(obstacleSpeedTextMarkers);
}

void VoxelGridTracking::publishROI()
{
    PolarGridTracking::roiArray roiMsg;
    roiMsg.rois3d.resize(m_obstacles.size());
    roiMsg.rois2d.resize(m_obstacles.size());
    
    roiMsg.id = m_currentId;
    roiMsg.header.frame_id = m_baseFrame;
    roiMsg.header.stamp = ros::Time::now();
    
    pcl::PointXYZRGB point3d, point;
    
    for (uint32_t i = 0; i < m_obstacles.size(); i++) {
        const VoxelObstacle & obstacle = m_obstacles[i];
        
        const double halfX = obstacle.sizeX() / 2.0;
        const double halfY = obstacle.sizeY() / 2.0;
        const double halfZ = obstacle.sizeZ() / 2.0;
        
        // A
        roiMsg.rois3d[i].A.x = (obstacle.centerX() - halfX);
        roiMsg.rois3d[i].A.y = (obstacle.centerY() - halfY);
        roiMsg.rois3d[i].A.z = (obstacle.centerZ() + halfZ);
        point3d.x = -roiMsg.rois3d[i].A.x;
        point3d.y = roiMsg.rois3d[i].A.z;
        point3d.z = roiMsg.rois3d[i].A.y;
        project3dTo2d(point3d, point, m_cameraParams);
        roiMsg.rois2d[i].A.u = point.x;
        roiMsg.rois2d[i].A.v = point.y;
        
        // B
        roiMsg.rois3d[i].B.x = (obstacle.centerX() + halfX);
        roiMsg.rois3d[i].B.y = (obstacle.centerY() - halfY);
        roiMsg.rois3d[i].B.z = (obstacle.centerZ() + halfZ);
        point3d.x = -roiMsg.rois3d[i].A.x;
        point3d.y = roiMsg.rois3d[i].A.z;
        point3d.z = roiMsg.rois3d[i].A.y;
        project3dTo2d(point3d, point, m_cameraParams);
        roiMsg.rois2d[i].B.u = point.x;
        roiMsg.rois2d[i].B.v = point.y;
        
        // C
        roiMsg.rois3d[i].C.x = (obstacle.centerX() - halfX);
        roiMsg.rois3d[i].C.y = (obstacle.centerY() - halfY);
        roiMsg.rois3d[i].C.z = (obstacle.centerZ() - halfZ);
        point3d.x = -roiMsg.rois3d[i].A.x;
        point3d.y = roiMsg.rois3d[i].A.z;
        point3d.z = roiMsg.rois3d[i].A.y;
        project3dTo2d(point3d, point, m_cameraParams);
        roiMsg.rois2d[i].C.u = point.x;
        roiMsg.rois2d[i].C.v = point.y;
        
        // D
        roiMsg.rois3d[i].D.x = (obstacle.centerX() + halfX);
        roiMsg.rois3d[i].D.y = (obstacle.centerY() - halfY);
        roiMsg.rois3d[i].D.z = (obstacle.centerZ() - halfZ);
        point3d.x = -roiMsg.rois3d[i].A.x;
        point3d.y = roiMsg.rois3d[i].A.z;
        point3d.z = roiMsg.rois3d[i].A.y;
        project3dTo2d(point3d, point, m_cameraParams);
        roiMsg.rois2d[i].D.u = point.x;
        roiMsg.rois2d[i].D.v = point.y;
        
        // E
        roiMsg.rois3d[i].E.x = (obstacle.centerX() - halfX);
        roiMsg.rois3d[i].E.y = (obstacle.centerY() + halfY);
        roiMsg.rois3d[i].E.z = (obstacle.centerZ() + halfZ);
        point3d.x = -roiMsg.rois3d[i].A.x;
        point3d.y = roiMsg.rois3d[i].A.z;
        point3d.z = roiMsg.rois3d[i].A.y;
        project3dTo2d(point3d, point, m_cameraParams);
        roiMsg.rois2d[i].E.u = point.x;
        roiMsg.rois2d[i].E.v = point.y;
        
        // F
        roiMsg.rois3d[i].F.x = (obstacle.centerX() + halfX);
        roiMsg.rois3d[i].F.y = (obstacle.centerY() + halfY);
        roiMsg.rois3d[i].F.z = (obstacle.centerZ() + halfZ);
        point3d.x = -roiMsg.rois3d[i].A.x;
        point3d.y = roiMsg.rois3d[i].A.z;
        point3d.z = roiMsg.rois3d[i].A.y;
        project3dTo2d(point3d, point, m_cameraParams);
        roiMsg.rois2d[i].F.u = point.x;
        roiMsg.rois2d[i].F.v = point.y;
        
        // G
        roiMsg.rois3d[i].G.x = (obstacle.centerX() - halfX);
        roiMsg.rois3d[i].G.y = (obstacle.centerY() + halfY);
        roiMsg.rois3d[i].G.z = (obstacle.centerZ() - halfZ);
        point3d.x = -roiMsg.rois3d[i].A.x;
        point3d.y = roiMsg.rois3d[i].A.z;
        point3d.z = roiMsg.rois3d[i].A.y;
        project3dTo2d(point3d, point, m_cameraParams);
        roiMsg.rois2d[i].G.u = point.x;
        roiMsg.rois2d[i].G.v = point.y;
        
        // H
        roiMsg.rois3d[i].H.x = (obstacle.centerX() + halfX);
        roiMsg.rois3d[i].H.y = (obstacle.centerY() + halfY);
        roiMsg.rois3d[i].H.z = (obstacle.centerZ() - halfZ);
        point3d.x = -roiMsg.rois3d[i].A.x;
        point3d.y = roiMsg.rois3d[i].A.z;
        point3d.z = roiMsg.rois3d[i].A.y;
        project3dTo2d(point3d, point, m_cameraParams);
        roiMsg.rois2d[i].H.u = point.x;
        roiMsg.rois2d[i].H.v = point.y;
        
        roiMsg.rois3d[i].speed.x = obstacle.vx();
        roiMsg.rois3d[i].speed.y = obstacle.vy();
        roiMsg.rois3d[i].speed.z = obstacle.vz();
        point3d.x = -obstacle.vx();
        point3d.y = obstacle.vz();
        point3d.z = obstacle.vy();
        project3dTo2d(point3d, point, m_cameraParams);
        roiMsg.rois2d[i].speed.x = point.x;
        roiMsg.rois2d[i].speed.y = point.y;
    }
    
    m_ROIPub.publish(roiMsg);
}

void VoxelGridTracking::publishFakePointCloud()
{
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg (*m_fakePointCloud, cloudMsg);
    cloudMsg.header.frame_id = m_baseFrame;
    cloudMsg.header.stamp = ros::Time::now();
    cloudMsg.header.seq = m_currentId;
    
    m_fakePointCloudPub.publish(cloudMsg);
}

void VoxelGridTracking::visualizeROI2d()
{
    string imgPattern = "/local/imaged/Karlsruhe/2011_09_28/2011_09_28_drive_0038_sync/image_02/data/%010d.png";
//     string imgPattern = "/local/imaged/Karlsruhe/2011_09_26/2011_09_26_drive_0091_sync/image_02/data/%010d.png";
    char imgNameL[1024];
    sprintf(imgNameL, imgPattern.c_str(), m_currentId + 1);
    
    stringstream ss;
    ss << "/home/nestor/Vídeos/VoxelTracking/pedestrian/output";
    ss.width(10);
    ss.fill('0');
    ss << m_currentId;
    ss << ".png";
    
    cout << "rois2d: " << imgNameL << endl;
    
    cv::Mat img = cv::imread(string(imgNameL));
    
    pcl::PointXYZRGB point3d, point;
    
    for (uint32_t i = 0; i < m_obstacles.size(); i++) {
        const VoxelObstacle & obstacle = m_obstacles[i];
        
        if ((obstacle.minZ() - (m_cellSizeZ / 2.0)) != m_minZ)
            continue;
        
        if (obstacle.magnitude() < 1.0)
            continue;

        cv::Point2d pointUL(img.cols, img.rows), pointBR(0, 0);
        
        const double halfX = obstacle.sizeX() / 2.0;
        const double halfY = obstacle.sizeY() / 2.0;
        const double halfZ = obstacle.sizeZ() / 2.0;
        
        // A
        point3d.x = -(obstacle.centerX() - halfX);
        point3d.y = (obstacle.centerZ() - halfZ);
        point3d.z = (obstacle.centerY() + halfY);
        project3dTo2d(point3d, point, m_cameraParams);
        
        if (point.x < pointUL.x) pointUL.x = point.x;
        if (point.y < pointUL.y) pointUL.y = point.y;
        if (point.x > pointBR.x) pointBR.x = point.x;
        if (point.y > pointBR.y) pointBR.y = point.y;
        
        // B
        point3d.x = -(obstacle.centerX() + halfX);
        point3d.y = (obstacle.centerZ() + halfZ);
        point3d.z = (obstacle.centerY() - halfY);
        project3dTo2d(point3d, point, m_cameraParams);
        
        if (point.x < pointUL.x) pointUL.x = point.x;
        if (point.y < pointUL.y) pointUL.y = point.y;
        if (point.x > pointBR.x) pointBR.x = point.x;
        if (point.y > pointBR.y) pointBR.y = point.y;
        
        // C
        point3d.x = -(obstacle.centerX() - halfX);
        point3d.y = (obstacle.centerZ() - halfZ);
        point3d.z = (obstacle.centerY() - halfY);
        project3dTo2d(point3d, point, m_cameraParams);
        
        if (point.x < pointUL.x) pointUL.x = point.x;
        if (point.y < pointUL.y) pointUL.y = point.y;
        if (point.x > pointBR.x) pointBR.x = point.x;
        if (point.y > pointBR.y) pointBR.y = point.y;
        
        // D
        point3d.x = -(obstacle.centerX() + halfX);
        point3d.y = (obstacle.centerZ() - halfZ);
        point3d.z = (obstacle.centerY() - halfY);
        project3dTo2d(point3d, point, m_cameraParams);
        
        if (point.x < pointUL.x) pointUL.x = point.x;
        if (point.y < pointUL.y) pointUL.y = point.y;
        if (point.x > pointBR.x) pointBR.x = point.x;
        if (point.y > pointBR.y) pointBR.y = point.y;
        
        // E
        point3d.x = -(obstacle.centerX() - halfX);
        point3d.y = (obstacle.centerZ() + halfZ);
        point3d.z = (obstacle.centerY() + halfY);
        project3dTo2d(point3d, point, m_cameraParams);
        
        if (point.x < pointUL.x) pointUL.x = point.x;
        if (point.y < pointUL.y) pointUL.y = point.y;
        if (point.x > pointBR.x) pointBR.x = point.x;
        if (point.y > pointBR.y) pointBR.y = point.y;
        
        // F
        point3d.x = -(obstacle.centerX() + halfX);
        point3d.y = (obstacle.centerZ() + halfZ);
        point3d.z = (obstacle.centerY() + halfY);
        project3dTo2d(point3d, point, m_cameraParams);
        
        if (point.x < pointUL.x) pointUL.x = point.x;
        if (point.y < pointUL.y) pointUL.y = point.y;
        if (point.x > pointBR.x) pointBR.x = point.x;
        if (point.y > pointBR.y) pointBR.y = point.y;
        
        // G
        point3d.x = -(obstacle.centerX() - halfX);
        point3d.y = (obstacle.centerZ() - halfZ);
        point3d.z = (obstacle.centerY() + halfY);
        project3dTo2d(point3d, point, m_cameraParams);
        
        if (point.x < pointUL.x) pointUL.x = point.x;
        if (point.y < pointUL.y) pointUL.y = point.y;
        if (point.x > pointBR.x) pointBR.x = point.x;
        if (point.y > pointBR.y) pointBR.y = point.y;
        
        // H
        point3d.x = -(obstacle.centerX() + halfX);
        point3d.y = (obstacle.centerZ() - halfZ);
        point3d.z = (obstacle.centerY() + halfY);
        project3dTo2d(point3d, point, m_cameraParams);
        
        if (point.x < pointUL.x) pointUL.x = point.x;
        if (point.y < pointUL.y) pointUL.y = point.y;
        if (point.x > pointBR.x) pointBR.x = point.x;
        if (point.y > pointBR.y) pointBR.y = point.y;
        
        cv::rectangle(img, pointUL, pointBR, cv::Scalar((double)rand() / RAND_MAX * 128 + 128, (double)rand() / RAND_MAX * 128 + 128, (double)rand() / RAND_MAX * 128 + 128), 1);
    }
    
    cv::imwrite(ss.str(), img);
    
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> pointMat(3, 1);
    
    BOOST_FOREACH(const pcl::PointXYZRGB & point, m_pointCloud->points) {
        //         pointMat << m_cameraParams.t(0) - point.x, m_cameraParams.t(1) + point.y, m_cameraParams.t(2) + point.z;
        pointMat << m_cameraParams.t(0) - point.x, m_cameraParams.t(1) + point.z, m_cameraParams.t(2) + point.y;
        //         pointMat << m_cameraParams.t(0) + point.x, m_cameraParams.t(1) - point.z, m_cameraParams.t(2) + point.y;
        
        pointMat = m_cameraParams.R.inverse() * pointMat;
        
        
        const double d = m_cameraParams.ku * m_cameraParams.baseline / pointMat(2);
        const double u = m_cameraParams.u0 - ((pointMat(0) * d) / m_cameraParams.baseline);
        const double v = m_cameraParams.v0 - ((pointMat(1) * m_cameraParams.kv * d) / (m_cameraParams.ku * m_cameraParams.baseline));
        
        //         cout << cv::Point3d(- point.x, point.y, point.z) << " -> " << pointMat.transpose() 
        //         << " -> " << cv::Point3d(u, v, d) << endl;
        
        if ((u >= 0) && (u < img.cols) &&
            (v >= 0) && (v < img.rows)) {
            
            img.at<cv::Vec3b>(v, u) = cv::Vec3b(point.b, point.g, 255);
            }
    }
    
    cv::imshow("rois2d", img);
    
    cv::waitKey(200);
}
}