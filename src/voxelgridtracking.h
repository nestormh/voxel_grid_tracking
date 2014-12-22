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
#include "tf/transform_listener.h"
#include "tf/message_filter.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl_ros/point_cloud.h>

#include "polargridtracking.h"

#include "voxel.h"
#include "voxelobstacle.h"

#define DEFAULT_BASE_FRAME "left_cam"
#define MAX_OBSTACLES_VISUALIZATION 10000
#define MAX_GRID_DIMENSION 500
#define MAX_PARTICLE_AGE_REPRESENTATION 8

namespace voxel_grid_tracking {
    
class VoxelGridTracking
{
public:
    VoxelGridTracking();
    
    // TODO: Change to PointXYZNormal
    typedef pcl::PointXYZRGBNormal PointNormalType;
    typedef pcl::PointCloud< PointNormalType > PointCloudNormal;
    typedef PointCloudNormal::Ptr PointCloudNormalPtr;
    
protected:
    typedef boost::multi_array<double, 4> ColorMatrix;
    typedef boost::multi_array<double, 2> ColorVector;
    typedef boost::multi_array<cv::Scalar, 1> ParticlesColorVector;
    typedef tf::MessageFilter<sensor_msgs::PointCloud2> TfPointCloudSynchronizer;
    typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloudSubscriber;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    
    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud< PointType > PointCloud;
    typedef PointCloud::Ptr PointCloudPtr;
    
    // Callbacks
    void getCameraInfo(const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msgPointCloud);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msgPointCloud, 
                            const sensor_msgs::PointCloud2::ConstPtr& msgOFlow);
    
    // Method functions
    void compute(const PointCloudPtr & pointCloud);
    void reset();
    void extractDynamicObjects(const PointCloudPtr& pointCloud);
    void constructOctomapFromPointCloud(const PointCloudPtr& pointCloud);
    void getVoxelGridFromPointCloud(const PointCloudPtr& pointCloud);
    void getMeasurementModel();
    void initialization();
    void particleToVoxel(const ParticlePtr & particle, 
                         int32_t & posX, int32_t & posY, int32_t & posZ);
    void prediction();
    void measurementBasedUpdate();
    void segment();
    void segmentWithClustering();
    void aggregation();
    void noiseRemoval();
    void updateObstacles();
    void filterObstacles();
    void joinCommonVolumes();
    void updateSpeedFromObstacles();
    void generateFakePointClouds();
    
    void updateFromOFlow();
    
    // Visualization functions
    void publishVoxels();
    void publishParticles();
    void publishOFlow();
    void publishMainVectors();
    void publishObstacles();
    void publishObstacleCubes();
    void publishROI();
    void publishFakePointCloud();
    void visualizeROI2d();
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_fakePointCloud;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr m_oFlowCloud;
    PointCloudPtr m_lastPointCloud;
    
    double m_deltaYaw, m_deltaPitch, m_speed, m_deltaTime;
    ros::Time m_lastPointCloudTime;
    double m_deltaX, m_deltaY, m_deltaZ;
    
    VoxelGrid m_grid;
    VoxelList m_voxelList;
    ParticleList m_particles;
    
    ColorVector m_obstacleColors;
    ParticlesColorVector m_particleColors;
    
    uint32_t m_dimX, m_dimY, m_dimZ;
    
    bool m_initialized;
    
    tf::StampedTransform m_lastMapOdomTransform;
    tf::StampedTransform m_pose2MapTransform;
    tf::StampedTransform m_map2CamTransform;
    
    tf::TransformListener m_tfListener;
    
    VoxelObstacleList m_obstacles;
    
    uint32_t m_currentId;
    
    // Parameters
    polar_grid_tracking::t_Camera_params m_cameraParams;
    float m_minX, m_maxX, m_minY, m_maxY, m_minZ, m_maxZ;
    float m_factorSpeed;
    float m_cellSizeX, m_cellSizeY, m_cellSizeZ;
    float m_voxelSize;
    double m_maxVelX, m_maxVelY, m_maxVelZ, m_maxMagnitude, m_minMagnitude;
    double m_particlesPerVoxel, m_threshProbForCreation;
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
    double m_maxObstacleHeight;
    double m_timeIncrementForFakePointCloud;
    bool m_useOFlow;
    float m_threshOccupancyProb;
    uint32_t m_maxNumberOfParticles;
    
    uint32_t m_threads;
    
    float m_focalX, m_focalY, m_centerX, m_centerY;
    
    string m_mapFrame;
    string m_poseFrame;
    string m_cameraFrame;
    string m_baseFrame;//TODO: Remove
    
    sensor_msgs::CameraInfoConstPtr m_cameraInfo;

    // Synchronizers
    boost::shared_ptr<TfPointCloudSynchronizer> m_tfPointCloudSync;
    boost::shared_ptr<ExactSync> m_synchronizer;
    
    // Subscribers
    PointCloudSubscriber m_pointCloudSub;
    PointCloudSubscriber m_oFlowSub;
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo> InfoSubscriber;
    InfoSubscriber m_cameraInfoSub;
    
    // Publishers
    ros::Publisher m_dynObjectsPub;
    ros::Publisher m_voxelsPub;
    ros::Publisher m_voxelsIdxPub;
    ros::Publisher m_pointsPerVoxelPub;
    ros::Publisher m_particlesPub;
    ros::Publisher m_particles0Pub;
    ros::Publisher m_particles1Pub;
    ros::Publisher m_particles2Pub;
    ros::Publisher m_particles3Pub;
    ros::Publisher m_particlesDPub;
    ros::Publisher m_particlesSimplePub;
    ros::Publisher m_oFlowPub;
    ros::Publisher m_mainVectorsPub;
    ros::Publisher m_obstaclesPub;
    ros::Publisher m_obstacleCubesPub;
    ros::Publisher m_obstacleSpeedPub;
    ros::Publisher m_obstacleSpeedTextPub;
    ros::Publisher m_ROIPub;
    ros::Publisher m_fakePointCloudPub;
    ros::Publisher m_segmentedPointCloudPub;
    ros::Publisher m_debugSegmentPub;
    ros::Publisher m_debugProbPub;
};

}

#endif // VOXELGRIDTRACKING_H
