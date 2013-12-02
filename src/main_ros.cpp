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

#include "ObstaclesFromStereo.h"
#include "polargridtracking.h"
#include "polargridtrackingros.h"
#include "utils.h"
#include "libvisohelper.h"
#include </home/nestor/Dropbox/projects/GPUCPD/src/LU-Decomposition/Libs/Cuda/include/device_launch_parameters.h>


#include <iostream>
#include <iomanip>
#include <boost/filesystem.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <opencv2/opencv.hpp>

using namespace std;

// Function declarations
void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud);
void testStereoTracking();
void testPointCloud();

// Definitions
void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->addCoordinateSystem();
    
    
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (pointCloud, rgb, "pointCloud");
    
    viewer->spin();
}

void testPointCloud() {
    const uint32_t initialIdx = 100;
    const boost::filesystem::path correspondencesPath("/local/imaged/calibrated/cdvp_3d_pedestrian_detection_dataset_vicon_1");
    const boost::filesystem::path seqName("Rectified_Images");
    const boost::filesystem::path maskName("../resultsHierarchical/pixel");
    
    vector<polar_grid_tracking::t_Camera_params> cameraParams;
    //     ObstaclesFromStereo::getParamsFromDublinDataset("/local/imaged/calibrated/cdvp_3d_pedestrian_detection_dataset_vicon_1/Groundtruth3d/GroundtruthPlane.txt", cameraParams);
    //     ObstaclesFromStereo::getParams("/local/imaged/Karlsruhe/2009_09_08_drive_0010/2009_09_08_calib.txt", cameraParams, ObstaclesFromStereo::KARLSRUHE);
    ObstaclesFromStereo::getParams("/local/imaged/Karlsruhe/2011_09_28/calib_cam_to_cam.txt", cameraParams, ObstaclesFromStereo::KARLSRUHE_V2);
    
    boost::shared_ptr<ObstaclesFromStereo> pointCloudMaker;
    polar_grid_tracking::t_SGBM_params sgbmParams;
    sgbmParams.minDisparity = 0;
    sgbmParams.numDisparities = 64;
    sgbmParams.SADWindowSize = 3;
    sgbmParams.P1 = 36;
    sgbmParams.P2 = 288;
    sgbmParams.disp12MaxDiff = 1;
    sgbmParams.preFilterCap = 63;
    sgbmParams.uniquenessRatio = 10;
    sgbmParams.speckleWindowSize = 100;
    sgbmParams.speckleRange = 32;    
    sgbmParams.fullDP = true;
    
    //     boost::filesystem::path leftPath("/local/imaged/calibrated/cdvp_3d_pedestrian_detection_dataset_vicon_1/frames/seq.058.left.png");
    //     boost::filesystem::path rightPath("/local/imaged/calibrated/cdvp_3d_pedestrian_detection_dataset_vicon_1/frames/seq.058.right.png");
    //         boost::filesystem::path leftMaskPath = correspondencesPath / seqName / maskName / boost::filesystem::path("Image" + ss.str() + ".png");
    
    //     boost::filesystem::path leftPath("/local/imaged/calibrated/stereocalib-001/image0000153LS.bmp");
    //     boost::filesystem::path rightPath("/local/imaged/calibrated/stereocalib-001/image0000153RS.bmp");
    
    //     boost::filesystem::path leftPath("/local/imaged/Karlsruhe/2009_09_08_drive_0010/I1_000100.png");
    //     boost::filesystem::path rightPath("/local/imaged/Karlsruhe/2009_09_08_drive_0010/I2_000100.png");
    
    boost::filesystem::path leftPath("/local/imaged/Karlsruhe/2011_09_28/2011_09_28_drive_0038_sync/image_02/data/0000000022.png");
    boost::filesystem::path rightPath("/local/imaged/Karlsruhe/2011_09_28/2011_09_28_drive_0038_sync/image_03/data/0000000022.png");
    
    
    cout << leftPath << endl;
    cout << rightPath << endl;
    
    cv::Mat left = cv::imread(leftPath.string());
    cv::Mat right = cv::imread(rightPath.string());
    
    cv::Mat leftMask(left.size(), CV_8UC1);
    leftMask.setTo(cv::Scalar(255));
    
    pointCloudMaker.reset(new ObstaclesFromStereo(cv::Size(left.cols, left.rows), ObstaclesFromStereo::KARLSRUHE_V2));
    pointCloudMaker->setCameraParams(cameraParams.at(0), cameraParams.at(1));
    pointCloudMaker->setMethod(ObstaclesFromStereo::SGBM);
    pointCloudMaker->setSGBMParams(sgbmParams);
    pointCloudMaker->setGroundThresh(0.3);
    pointCloudMaker->setBackGroundThresh(20);
    pointCloudMaker->setLeafSize(0.05);
    
    pointCloudMaker->generatePointClouds(left, right, leftMask);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud = pointCloudMaker->getPointCloud();
    visualizePointCloud(pointCloud);
}

void testStereoTracking() {
    const ObstaclesFromStereo::t_CalibrationFileType calibrationType = ObstaclesFromStereo::KARLSRUHE_V2;
    
    uint32_t initialIdx;
    boost::filesystem::path correspondencesPath;
    boost::filesystem::path seqName;
    string leftImagePattern;
    string rightImagePattern;
    vector<polar_grid_tracking::t_Camera_params> cameraParams;
    vector< t_ego_value > egoValues;
    
    switch (calibrationType) {
        case ObstaclesFromStereo::DUBLIN:
        {
            initialIdx = 58;
            correspondencesPath = boost::filesystem::path("/local/imaged/calibrated");
            seqName = boost::filesystem::path("cdvp_3d_pedestrian_detection_dataset_vicon_1");
            leftImagePattern = "frames/seq.%03d.left.png";
            rightImagePattern = "frames/seq.%03d.right.png";
            
            ObstaclesFromStereo::getParams("/local/imaged/calibrated/cdvp_3d_pedestrian_detection_dataset_vicon_1/Groundtruth3d/GroundtruthPlane.txt", cameraParams, ObstaclesFromStereo::DUBLIN);
            
            break;
        }
        case ObstaclesFromStereo::KARLSRUHE:
        {
            initialIdx = 100;
            correspondencesPath = boost::filesystem::path("/local/imaged/Karlsruhe");
            seqName = boost::filesystem::path("2009_09_08_drive_0010");
            leftImagePattern = "I1_%06d.png";
            rightImagePattern = "I2_%06d.png";
            
            ObstaclesFromStereo::getParams("/local/imaged/Karlsruhe/2009_09_08_drive_0010/2009_09_08_calib.txt", cameraParams, ObstaclesFromStereo::KARLSRUHE);
            
            break;
        }
        case ObstaclesFromStereo::KARLSRUHE_V2:
        {
            initialIdx = 55;
            correspondencesPath = boost::filesystem::path("/local/imaged/Karlsruhe");
            seqName = boost::filesystem::path("2011_09_28/2011_09_28_drive_0038_sync");
            leftImagePattern = "image_02/data/%010d.png";
            rightImagePattern = "image_03/data/%010d.png";
            
            ObstaclesFromStereo::getParams("/local/imaged/Karlsruhe/2011_09_28/calib_cam_to_cam.txt", cameraParams, ObstaclesFromStereo::KARLSRUHE_V2);
            
            ObstaclesFromStereo::readEgoValues((correspondencesPath / seqName).string(), egoValues);
            
            break;
        }
        default:
            exit(0);
    }
    
    LibvisoHelper visualOdom(cameraParams[0]);
    
    boost::shared_ptr<ObstaclesFromStereo> pointCloudMaker;
    polar_grid_tracking::t_SGBM_params sgbmParams;
    sgbmParams.minDisparity = 0;
    sgbmParams.numDisparities = 64;
    sgbmParams.SADWindowSize = 3;
    sgbmParams.P1 = 36;
    sgbmParams.P2 = 288;
    sgbmParams.disp12MaxDiff = 1;
    sgbmParams.preFilterCap = 63;
    sgbmParams.uniquenessRatio = 10;
    sgbmParams.speckleWindowSize = 100;
    sgbmParams.speckleRange = 32;    
    sgbmParams.fullDP = true;
    
    // TODO: Read from a parameters file
    uint32_t rows = 60; // 400
    uint32_t cols = 60; // 128
    double cellSizeX = 0.2; // 0.1
    double cellSizeZ = 0.2; // 0.1
    double maxVelX = 5.0; // 0.1
    double maxVelZ= 5.0; // 0.1 
    double particlesPerCell = 1000; //1000;
    double threshProbForCreation = 0.9999; //0.2;
    
    double gridDepthFactor = 0.05;
    uint32_t gridColumnFactor = 12;
    double yawInterval = 5.0 * M_PI / 180.0;
    
    // TODO Get it from the real measurements
    //     double deltaTime = 0.2; //1.0 / 25.0; //0.2;
    
    PolarGridTrackingROS gridTracker(rows, cols, cellSizeX, cellSizeZ, maxVelX, maxVelZ, cameraParams[0], particlesPerCell, threshProbForCreation, gridDepthFactor, gridColumnFactor, yawInterval);
    
    for (uint32_t i = initialIdx; i < 1000; i++) {
        //         stringstream ss;
        //         ss << setfill('0') << setw(3) << i;
        //         stringstream ss2;
        //         ss2 << i;
        char imgNameL[1024], imgNameR[1024];
        sprintf(imgNameL, leftImagePattern.c_str(), i);
        sprintf(imgNameR, rightImagePattern.c_str(), i);
        
        boost::filesystem::path leftPath = correspondencesPath / seqName / boost::filesystem::path(imgNameL);
        boost::filesystem::path rightPath = correspondencesPath / seqName / boost::filesystem::path(imgNameR);
        
        cout << leftPath << endl;
        cout << rightPath << endl;
        
        cv::Mat left = cv::imread(leftPath.string());
        cv::Mat right = cv::imread(rightPath.string());
        
        cv::Mat leftMask(left.size(), CV_8UC1);
        leftMask.setTo(cv::Scalar(255));
        
        if (i == initialIdx) {
            pointCloudMaker.reset(new ObstaclesFromStereo(cv::Size(left.cols, left.rows), calibrationType));
            pointCloudMaker->setCameraParams(cameraParams.at(0), cameraParams.at(1));
            pointCloudMaker->setMethod(ObstaclesFromStereo::SGBM);
            pointCloudMaker->setSGBMParams(sgbmParams);
            pointCloudMaker->setGroundThresh(0.3);
            pointCloudMaker->setBackGroundThresh(20);
            pointCloudMaker->setLeafSize(0.05);
        }
        
        cv::imshow("imgL", left);
        //         cv::imshow("imgR", right);
        //         
        pointCloudMaker->generatePointClouds(left, right, leftMask);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud = pointCloudMaker->getPointCloud();
        
        // TODO: Think in the right strategy in case yaw couldn't be obtained
        double yaw, speed, deltaTime;
        switch (calibrationType) {
            case ObstaclesFromStereo::DUBLIN:
            {
                yaw = 0.0;
                speed = 0.0;
                deltaTime = 0.2;
                break;
            }
            case ObstaclesFromStereo::KARLSRUHE:
            {
                deltaTime = 0.25;
                //                 visualOdom.compute(left, right, deltaTime, yaw, speed);
                yaw = 0.0;
                speed = 0.0;
                
                break;
            }
            case ObstaclesFromStereo::KARLSRUHE_V2:
            {
                yaw = egoValues[i].deltaYaw;
                speed = egoValues[i].speed;
                deltaTime = egoValues[i].deltaTime;
                
                break;
            }
        }
        //         if (calibrationType != ObstaclesFromStereo::KARLSRUHE_V2) {
        //             visualOdom.compute(left, right, deltaTime, yaw, speed);
        //             cout << "Yaw = " << yaw * 180 / 3.14 << endl;
        //             cout << "speed = " << speed << endl;
        //         } else {
        yaw = 0.0;
        speed = 0.0;
        //         }
        
        //         gridTracker.setDeltaYawSpeedAndTime(0.0 / 180.0 * 3.14, 0.0, 1.0);
        gridTracker.setDeltaYawSpeedAndTime(yaw, speed, deltaTime);
        //         gridTracker.setDeltaYawSpeedAndTime(45.0 / 180.0 * 3.14, 0.0, deltaTime);
        gridTracker.compute(pointCloud);
        
        //         visualizePointCloud(pointCloud);
//         if (i == initialIdx) {
//             cv::waitKey(20);
//         } else {
            uint8_t keycode = cv::waitKey(0);
            if (keycode == 27) {
                break;
            }
//         }
        
        //         if (i != initialIdx)
        //             break;
        ros::spinOnce();
}
}

int main(int argC, char **argV) {
    ros::init(argC, argV, "PolarGridTracking");
    
//     if (fork() == 0) {
//         testPointCloud();
//     }
    testStereoTracking();
    
    return 0;
}