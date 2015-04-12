/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * main_ground_based_people_detection_app.cpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 * 
 * Adapted to ROS:
 * svm_pedestrian_tracking.cpp
 * Created on: 17 Mar, 2015
 * Author: Néstor Morales (nestor@isaatc.ull.es)
 *
 * Example file for performing people detection on a Kinect live stream.
 * As a first step, the ground is manually initialized, then people detection is performed with the GroundBasedPeopleDetectionApp class,
 * which implements the people detection algorithm described here:
 * M. Munaro, F. Basso and E. Menegatti,
 * Tracking people within groups with RGB-D data,
 * In Proceedings of the International Conference on Intelligent Robots and Systems (IROS) 2012, Vilamoura (Portugal), 2012.
 */


#include "svm_pedestrian_tracking.h"

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
#include <pcl/point_types.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl-1.7/pcl/common/common.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/features/normal_3d.h>
#include <pcl-1.7/pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/foreach.hpp>
#include <boost/graph/graph_concepts.hpp>

#include <opencv2/opencv.hpp>
#include <message_filters/synchronizer.h>

#include <eigen3/Eigen/Core>

#include <math.h>

 ///////////////////////////////////////////

// #include <octomap/octomap.h>
// #include <octomap/OcTree.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
// 
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
// 
// #include <iostream>
// #include <vector>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/search/search.h>
#include <pcl-1.7/pcl/search/kdtree.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/segmentation/region_growing.h>

///////////////////////////////////////////

#include <iostream>
#include <queue>
#include <pcl-1.7/pcl/impl/point_types.hpp>

#include <pcl/sample_consensus/sac_model_plane.h>

using namespace std;

namespace svm_pedestrian_tracking {
    
SVMPedestrianTracking::SVMPedestrianTracking()
{
    m_lastMapOdomTransform.stamp_ = ros::Time(-1);
    ros::NodeHandle nh("~");
    
    // Reading params
    nh.param<string>("map_frame", m_mapFrame, "/map");
    nh.param<string>("pose_frame", m_poseFrame, "/base_footprint");
    nh.param<string>("camera_frame", m_cameraFrame, "/base_left_cam");

    string svm_filename;
    nh.param<string>("svm_filename", svm_filename, "");
    nh.param<double>("min_confidence", m_min_confidence, -1.5);
    nh.param<double>("voxel_size", m_voxel_size, 0.06);
    nh.param<double>("min_height", m_min_height, 1.3);
    nh.param<double>("max_height", m_max_height, 2.3);
    
    cout << "PARAMETERS" << endl;
    cout << "==========" << endl;
    cout << "map_frame " << m_mapFrame << endl;
    cout << "pose_frame " << m_poseFrame << endl;
    cout << "camera_frame " << m_cameraFrame << endl;
    cout << "svm_filename " << svm_filename << endl;
    cout << "min_confidence " << m_min_confidence << endl;
    cout << "voxel_size " << m_voxel_size << endl;
    cout << "min_height " << m_min_height << endl;
    cout << "max_height " << m_max_height << endl;
    
    // Topics
    std::string left_info_topic = "left/camera_info";
    std::string right_info_topic = "right/camera_info";
    
    // Ground plane estimation:
    m_ground_coeffs.resize(4);
    std::vector<int> clicked_points_indices;
    PointCloudPtr clicked_points_3d (new PointCloud);
//     clicked_points_3d->push_back(PointType(0.0, 0.0, 0.0));
//     clicked_points_3d->push_back(PointType(1.0, 0.0, 0.0));
//     clicked_points_3d->push_back(PointType(0.0, 1.0, 0.0));
//     clicked_points_3d->push_back(PointType(1.0, 1.0, 0.0));
    clicked_points_3d->push_back(PointType(0.0, 1.57, 0.0));
    clicked_points_3d->push_back(PointType(1.0, 1.57, 0.0));
    clicked_points_3d->push_back(PointType(0.0, 1.57, 1.0));
//     clicked_points_3d->push_back(PointType(1.0, 1.57, 1.0));
    
    for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
        clicked_points_indices.push_back(i);
    pcl::SampleConsensusModelPlane<PointType> model_plane(clicked_points_3d);
    model_plane.computeModelCoefficients(clicked_points_indices, m_ground_coeffs);
    std::cout << "Ground plane: " << m_ground_coeffs(0) << " " << m_ground_coeffs(1) << " " << m_ground_coeffs(2) << " " << m_ground_coeffs(3) << std::endl;
    
    // Create classifier for people detection:  
    pcl::people::PersonClassifier<pcl::RGB> person_classifier;
    person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM
    
    // People detection app initialization:
    m_people_detector.setVoxelSize(m_voxel_size);                        // set the voxel size
//     m_people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
    m_people_detector.setClassifier(person_classifier);                // set person classifier
    m_people_detector.setHeightLimits(m_min_height, m_max_height);         // set person classifier
    //  m_people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical
    
    // Synchronize input topics. Optionally do approximate synchronization or not.
    bool approx;
    int queue_size;
    nh.param("approximate_sync", approx, false);
    nh.param("queue_size", queue_size, 10);
    
    m_leftCameraInfoSub.subscribe(nh, left_info_topic, 1);
    m_rightCameraInfoSub.subscribe(nh, right_info_topic, 1);
    m_pointCloudSub.subscribe(nh, "pointCloud", 10);
            
    if (approx) {
        m_approxSynchronizer.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                        m_pointCloudSub, 
                                                        m_leftCameraInfoSub, 
                                                        m_rightCameraInfoSub) );
        m_approxSynchronizer->registerCallback(boost::bind(&SVMPedestrianTracking::pointCloudCallback, 
                                                        this, _1, _2, _3));
    } else {
        m_exactSynchronizer.reset( new ExactSync(ExactPolicy(queue_size),
                                                    m_pointCloudSub,
                                                    m_leftCameraInfoSub, 
                                                    m_rightCameraInfoSub) );
        m_exactSynchronizer->registerCallback(boost::bind(&SVMPedestrianTracking::pointCloudCallback, 
                                                            this, _1, _2, _3));
    }
    
    image_transport::ImageTransport it(nh);
    m_pointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void SVMPedestrianTracking::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msgPointCloud,
                                           const sensor_msgs::CameraInfoConstPtr& leftCameraInfo, 
                                           const sensor_msgs::CameraInfoConstPtr& rightCameraInfo) 
{
    m_cameraFrame = msgPointCloud->header.frame_id;
    try {
        m_tfListener.lookupTransform(m_mapFrame, m_poseFrame, ros::Time(0), m_pose2MapTransform);
        m_tfListener.lookupTransform(m_cameraFrame, m_mapFrame, ros::Time(0), m_map2CamTransform);
    } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    
    pcl::fromROSMsg<pcl::PointXYZRGB>(*msgPointCloud, *m_pointCloud);
    
    if (m_pointCloud->size() != 0) {
        
        m_deltaTime = (msgPointCloud->header.stamp - m_lastPointCloudTime).toSec();
        
        m_lastPointCloudTime = msgPointCloud->header.stamp;
        
        m_stereoCameraModel.fromCameraInfo(*leftCameraInfo, *rightCameraInfo);
        m_currentId = leftCameraInfo->header.seq;
        
        m_rgb_intrinsics_matrix << leftCameraInfo->K[0], leftCameraInfo->K[1], leftCameraInfo->K[2],
                                   leftCameraInfo->K[3], leftCameraInfo->K[4], leftCameraInfo->K[5], 
                                   leftCameraInfo->K[6], leftCameraInfo->K[7], leftCameraInfo->K[8];
        m_people_detector.setIntrinsics(m_rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
        
        compute(m_pointCloud);
    }
    
}

/**
 * Given a certain pointCloud, the frame is computed
 * @param pointCloud: The input point cloud.
 */
void SVMPedestrianTracking::compute(PointCloudPtr& pointCloud)
{
    cout << __LINE__ << endl;
    // Perform people detection on the new cloud:
    std::vector<pcl::people::PersonCluster<PointType> > clusters;   // vector containing persons clusters
    cout << __LINE__ << endl;
    m_people_detector.setInputCloud(pointCloud);
    cout << __LINE__ << endl;
    m_people_detector.setGround(m_ground_coeffs);                    // set floor coefficients
    cout << __LINE__ << endl;
    m_people_detector.compute(clusters);                           // perform people detection
    cout << __LINE__ << endl;
    
    m_ground_coeffs = m_people_detector.getGround();                 // get updated floor coefficients
    
    cout << __LINE__ << endl;
    
    unsigned int k = 0;
    cout << __LINE__ << endl;
    for(std::vector<pcl::people::PersonCluster<PointType> >::iterator it = clusters.begin(); 
            it != clusters.end(); ++it)
    {
        cout << __LINE__ << endl;
        if(it->getPersonConfidence() > m_min_confidence)             // draw only people with confidence above a threshold
        {
            cout << __LINE__ << endl;
            // TODO: Publish detection
            k++;
        }
        cout << __LINE__ << endl;
    }
    cout << __LINE__ << endl;
    std::cout << "**************************" <<  k << " people found" << std::endl;
    cout << __LINE__ << endl;
}

}