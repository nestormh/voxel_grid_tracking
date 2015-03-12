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
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/subscriber_filter.h>

#include <image_geometry/stereo_camera_model.h>

#include <pcl_ros/point_cloud.h>
#include <pcl-1.7/pcl/search/kdtree.h>

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
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicyOFlow;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicyOFlow;
    typedef message_filters::Synchronizer<ExactPolicyOFlow> ExactSyncOFlow;
    typedef message_filters::Synchronizer<ApproximatePolicyOFlow> ApproximateSyncOFlow;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    
//     typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2> ExactPolicyJustPointCloud;
//     typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2> ApproximatePolicyJustPointCloud;
//     typedef message_filters::Synchronizer<ExactPolicyJustPointCloud> ExactSyncJustPointCloud;
//     typedef message_filters::Synchronizer<ApproximatePolicyJustPointCloud> ApproximateSyncJustPointCloud;
    
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo> InfoSubscriber;
    
    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud< PointType > PointCloud;
    typedef PointCloud::Ptr PointCloudPtr;
    
    // Callbacks
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msgPointCloud);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msgPointCloud, 
                            const sensor_msgs::CameraInfoConstPtr& leftCameraInfo, 
                            const sensor_msgs::CameraInfoConstPtr& rightCameraInfo);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msgPointCloud, 
                            const sensor_msgs::PointCloud2::ConstPtr& msgOFlow,
                            const sensor_msgs::CameraInfoConstPtr& leftCameraInfo, 
                            const sensor_msgs::CameraInfoConstPtr& rightCameraInfo);
    void debugImageCallback(const sensor_msgs::ImageConstPtr& imgMsg);
    
    // Method functions
    void compute(const PointCloudPtr & pointCloud);
    void reset();
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
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr m_oFlowCloud;
    PointCloudPtr m_lastPointCloud;
    pcl::search::KdTree<PointType> m_kdtreeLastPointCloud;
    
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
    
    cv::Mat m_dbgImg;
    
    image_geometry::StereoCameraModel m_stereoCameraModel;
    
    // Parameters
    float m_cellSizeX, m_cellSizeY, m_cellSizeZ;
    uint32_t m_maxNumberOfParticles;
    uint32_t m_threads;
    double m_maxVelX, m_maxVelY, m_maxVelZ;
    double m_minVelX, m_minVelY, m_minVelZ;
    double m_maxMagnitude, m_minMagnitude;
    double m_yawInterval, m_pitchInterval, m_factorSpeed;
    SpeedMethod m_obstacleSpeedMethod;
    double m_threshOccupancyProb;
    uint32_t m_neighBorX, m_neighBorY, m_neighBorZ;
    
    double m_particlesPerVoxel;
    double m_threshYaw, m_threshPitch, m_threshMagnitude;
    uint32_t m_minVoxelsPerObstacle;
    double m_minVoxelDensity;
    double m_maxCommonVolume;
    double m_minObstacleHeight;
    
    SpeedMethod m_speedMethod;
    
    bool m_useOFlow;
    
    bool m_inputFromCameras;

    // Computed parameters
    float m_minX, m_maxX, m_minY, m_maxY, m_minZ, m_maxZ;
    float m_voxelSize;
    
    string m_mapFrame;
    string m_poseFrame;
    string m_cameraFrame;
    
    sensor_msgs::CameraInfoConstPtr m_cameraInfo;

    // Synchronizers
    boost::shared_ptr<TfPointCloudSynchronizer> m_tfPointCloudSync;
    boost::shared_ptr<ExactSyncOFlow> m_exactSynchronizerOFlow;
    boost::shared_ptr<ApproximateSyncOFlow> m_approxSynchronizerOFlow;
    boost::shared_ptr<ExactSync> m_exactSynchronizer;
    boost::shared_ptr<ApproximateSync> m_approxSynchronizer;
//     boost::shared_ptr<ExactSyncJustPointCloud> m_exactSynchronizerJustPointCloud;
//     boost::shared_ptr<ApproximateSyncJustPointCloud> m_approxSynchronizerJustPointCloud;
    
    // Subscribers
    PointCloudSubscriber m_pointCloudSub;
    PointCloudSubscriber m_oFlowSub;
    ros::Subscriber m_pointCloudJustPointCloudSub;
    InfoSubscriber m_cameraInfoSub;
    InfoSubscriber m_leftCameraInfoSub;
    InfoSubscriber m_rightCameraInfoSub;
    
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
    ros::Publisher m_timeStatsPub;
    ros::Publisher m_segmentedPointCloudPub;
    ros::Publisher m_debugSegmentPub;
    ros::Publisher m_debugProbPub;
    image_transport::Subscriber m_debugImgSub;
    ros::Publisher m_fakePointCloudPub;
    ros::Publisher m_fakeParticlesPub;
};

}

#endif // VOXELGRIDTRACKING_H
