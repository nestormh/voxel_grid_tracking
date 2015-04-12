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

#ifndef SVM_PEDESTRIAN_TRACKING_H
#define SVM_PEDESTRIAN_TRACKING_H

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

#include <pcl/people/ground_based_people_detection_app.h>

using namespace std;

#define DEFAULT_BASE_FRAME "left_cam"

namespace svm_pedestrian_tracking {
    
class SVMPedestrianTracking
{
public:
    SVMPedestrianTracking();
    
protected:
    typedef message_filters::Subscriber<sensor_msgs::PointCloud2> PointCloudSubscriber;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo> InfoSubscriber;
    
    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud< PointType > PointCloud;
    typedef PointCloud::Ptr PointCloudPtr;
    
    // Callbacks
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msgPointCloud, 
                            const sensor_msgs::CameraInfoConstPtr& leftCameraInfo, 
                            const sensor_msgs::CameraInfoConstPtr& rightCameraInfo);
    
    // Method functions
    void compute(PointCloudPtr & pointCloud);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloud;
    
    double m_deltaYaw, m_deltaPitch, m_speed, m_deltaTime;
    ros::Time m_lastPointCloudTime;
    double m_deltaX, m_deltaY, m_deltaZ;
    
    tf::StampedTransform m_lastMapOdomTransform;
    tf::StampedTransform m_pose2MapTransform;
    tf::StampedTransform m_map2CamTransform;
    
    tf::TransformListener m_tfListener;
    
    uint32_t m_currentId;
    
    image_geometry::StereoCameraModel m_stereoCameraModel;
    
    // Properties specifically related to the classifier
    Eigen::VectorXf m_ground_coeffs;
    pcl::people::GroundBasedPeopleDetectionApp<PointType> m_people_detector;    // people detection object
    Eigen::Matrix3f m_rgb_intrinsics_matrix;
    
    // Parameters
    string m_mapFrame;
    string m_poseFrame;
    string m_cameraFrame;
    
    double m_min_confidence;
    double m_min_height;
    double m_max_height;
    double m_voxel_size;
    
    // Synchronizers
    boost::shared_ptr<ExactSync> m_exactSynchronizer;
    boost::shared_ptr<ApproximateSync> m_approxSynchronizer;
    
    // Subscribers
    PointCloudSubscriber m_pointCloudSub;
    InfoSubscriber m_cameraInfoSub;
    InfoSubscriber m_leftCameraInfoSub;
    InfoSubscriber m_rightCameraInfoSub;
    
    // Publishers
};

}

#endif // SVM_PEDESTRIAN_TRACKING_H
