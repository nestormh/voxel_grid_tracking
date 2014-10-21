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
// #include "utils.h"
// #include "libvisohelper.h"

#include <iostream>
#include <iomanip>
#include <boost/filesystem.hpp>
#include <boost/concept_check.hpp>

#include<pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/PointCloud2.h"

#include <pcl/visualization/pcl_visualizer.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse_yml.h>

#include <boost/foreach.hpp>

#include <rosgraph_msgs/Clock.h>

#include <sstream>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define CAMERA_FRAME_ID "left_cam"
#define BASE_CAMERA_FRAME_ID "base_left_cam"

using namespace std;

// Function declarations
void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud);
void testStereoTracking();
void testPointCloud();
void publishPointCloud(ros::Publisher & pointCloudPub, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud, const uint32_t & idx);
void publishFakePointCloud(ros::Publisher & pointCloudPub, const double & radius, const uint32_t & idx);

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

void publishPointCloud(ros::Publisher & pointCloudPub, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud, const uint32_t & idx) {
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
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
    cloudMsg.header.frame_id = CAMERA_FRAME_ID;
    cloudMsg.header.stamp = ros::Time::now();
    cloudMsg.header.seq = idx;
    
    pointCloudPub.publish(cloudMsg);
    
//     ros::spinOnce();
}

void publishFakePointCloud(ros::Publisher& pointCloudPub, const double& radius, const uint32_t & idx)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pointCloud->reserve(4 * radius / 0.01);
    
    for (double x = -radius; x <= radius; x += 0.01) {
        pcl::PointXYZ point;
        point.x = x;
        point.y = -radius;
        point.z = 0.0;
        
        pointCloud->push_back(point);
        
        point.y = radius;
        pointCloud->push_back(point);
    }
    
    for (double y = -radius; y <= radius; y += 0.01) {
            pcl::PointXYZ point;
            point.x = -radius;
            point.y = y;
            point.z = 0.0;
                
            pointCloud->push_back(point);
            
            point.x = radius;
            pointCloud->push_back(point);
    }
    
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg (*pointCloud, cloudMsg);
    cloudMsg.header.frame_id = CAMERA_FRAME_ID;
    cloudMsg.header.stamp = ros::Time::now();
    cloudMsg.header.seq = idx;
    
    pointCloudPub.publish(cloudMsg);
}

void testStereoTracking() {
    cv::namedWindow("imgL");
//     cv::waitKey(0);
    
    const ObstaclesFromStereo::t_CalibrationFileType calibrationType = ObstaclesFromStereo::KARLSRUHE_V2;
    
    ros::NodeHandle nh("~");
    ros::Publisher pointCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("pointCloudStereo", 1);
    ros::Publisher fakePointCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("fakePointCloud", 1);
//     ros::Publisher deltaTimePub = nh.advertise<std_msgs::Float64> ("deltaTime", 1);
    ros::Publisher clockPub = nh.advertise<rosgraph_msgs::Clock> ("/clock", 1);
    
    ros::Publisher markersPub = nh.advertise<visualization_msgs::MarkerArray> ("tmpMarkers", 1);
    tf::TransformBroadcaster map2odomTfBroadcaster;
    
    image_transport::ImageTransport it(nh);
    image_transport::Publisher leftImgPub = it.advertise("left/image", 1);
    image_transport::Publisher rightImgPub = it.advertise("right/image", 1);
    
    ros::Publisher leftInfoPub = nh.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
    ros::Publisher righttInfoPub = nh.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);
    sensor_msgs::CameraInfo leftCameraInfo, rightCameraInfo;
    
    uint32_t initialIdx;
    uint32_t lastIdx = 1000;
    boost::filesystem::path correspondencesPath;
    boost::filesystem::path seqName;
    string leftImagePattern;
    string rightImagePattern;
    vector<polar_grid_tracking::t_Camera_params> cameraParams;
    vector< t_ego_value > egoValues;
    
    vector< visualization_msgs::MarkerArray > markers;
    
    double posX = 0.0, posY = 0.0, posTheta = 0.0, accTime = 0.0;
    
    // Params for the fake point cloud
    double radius = 15.0;
    
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
            initialIdx = 2; //55; //260; //72; //55;
            lastIdx = 340;
            correspondencesPath = boost::filesystem::path("/local/imaged/Karlsruhe");
            seqName = boost::filesystem::path("2011_09_28/2011_09_28_drive_0038_sync");     // Campus
//             seqName = boost::filesystem::path("2011_09_26/2011_09_26_drive_0015_sync");
//             seqName = boost::filesystem::path("2011_09_26/2011_09_26_drive_0052_sync");
//             seqName = boost::filesystem::path("2011_09_26/2011_09_26_drive_0091_sync"); // Pedestrian area
            leftImagePattern = "image_02/data/%010d.png";
            rightImagePattern = "image_03/data/%010d.png";
            
            ObstaclesFromStereo::getParams("/local/imaged/Karlsruhe/2011_09_28/calib_cam_to_cam.txt", cameraParams, ObstaclesFromStereo::KARLSRUHE_V2);
            
            ObstaclesFromStereo::readEgoValues((correspondencesPath / seqName).string(), egoValues);
            
            string leftCalibFileName = "/local/imaged/Karlsruhe/2011_09_28/left_calib.yaml";
            string leftCameraName = "left_camera";
            camera_calibration_parsers::readCalibrationYml(leftCalibFileName, leftCameraName, leftCameraInfo);
            
            string rightCalibFileName = "/local/imaged/Karlsruhe/2011_09_28/right_calib.yaml";
            string rightCameraName = "right_camera";
            camera_calibration_parsers::readCalibrationYml(rightCalibFileName, rightCameraName, rightCameraInfo);
            
//             markers = ObstaclesFromStereo::readMarkerList((correspondencesPath / seqName / "tracklet_labels.xml").string(), lastIdx);
            markers = ObstaclesFromStereo::readMarkerList("/local/imaged/Karlsruhe/2011_09_26/2011_09_26_drive_0091_sync/tracklet_labels.xml", lastIdx);
            
            leftCameraInfo.header.frame_id = BASE_CAMERA_FRAME_ID;
            rightCameraInfo.header.frame_id = BASE_CAMERA_FRAME_ID;
            
            break;
        }
        case ObstaclesFromStereo::BAHNHOFSTRASSE:
        {
            initialIdx = 1; //55;
            lastIdx = 1000;
            correspondencesPath = boost::filesystem::path("/local/imaged/stixels");
            seqName = boost::filesystem::path("bahnhof");
            rightImagePattern = "seq03-img-left/image_%08d_0.png";
            leftImagePattern = "seq03-img-right/image_%08d_1.png";
            
            ObstaclesFromStereo::getParams("/local/imaged/stixels/bahnhof", cameraParams, ObstaclesFromStereo::BAHNHOFSTRASSE);
            
            string leftCalibFileName = "/local/imaged/stixels/bahnhof/left_calib.yaml";
            string leftCameraName = "left_camera";
            camera_calibration_parsers::readCalibrationYml(leftCalibFileName, leftCameraName, leftCameraInfo);
            
            string rightCalibFileName = "/local/imaged/stixels/bahnhof/right_calib.yaml";
            string rightCameraName = "right_camera";
            camera_calibration_parsers::readCalibrationYml(rightCalibFileName, rightCameraName, rightCameraInfo);
            
            leftCameraInfo.header.frame_id = BASE_CAMERA_FRAME_ID;
            rightCameraInfo.header.frame_id = BASE_CAMERA_FRAME_ID;
            
            markers = ObstaclesFromStereo::readMarkerList("/local/imaged/Karlsruhe/2011_09_26/2011_09_26_drive_0091_sync/tracklet_labels.xml", lastIdx);
            
            break;
        }
        case ObstaclesFromStereo::DAIMLER:
        {
            initialIdx = 1; //55;
            lastIdx = 16400;
            correspondencesPath = boost::filesystem::path("/local/imaged/Daimler");
            seqName = boost::filesystem::path("TestData");
            rightImagePattern = "c0/image_%05d.pgm";
            leftImagePattern = "c1/image_%05d.pgm";
            
            ObstaclesFromStereo::getParams((correspondencesPath / seqName).string(), cameraParams, ObstaclesFromStereo::DAIMLER);
            
            string leftCalibFileName = "/local/imaged/Daimler/TestData/left_calib.yaml";
            string leftCameraName = "left_camera";
            camera_calibration_parsers::readCalibrationYml(leftCalibFileName, leftCameraName, leftCameraInfo);
            
            string rightCalibFileName = "/local/imaged/Daimler/TestData/right_calib.yaml";
            string rightCameraName = "right_camera";
            camera_calibration_parsers::readCalibrationYml(rightCalibFileName, rightCameraName, rightCameraInfo);
            
            leftCameraInfo.header.frame_id = BASE_CAMERA_FRAME_ID;
            rightCameraInfo.header.frame_id = BASE_CAMERA_FRAME_ID;
            
            markers = ObstaclesFromStereo::readMarkerList("/local/imaged/Karlsruhe/2011_09_26/2011_09_26_drive_0091_sync/tracklet_labels.xml", lastIdx);
            
            break;
        }
        default:
            exit(0);
    }
        
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
    
    rosgraph_msgs::Clock clockMsg;
    clockMsg.clock = ros::Time(0.0);
    clockPub.publish(clockMsg);
    ros::spinOnce();
    
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
//             pointCloudMaker->setMethod(ObstaclesFromStereo::SGBM);
            pointCloudMaker->setMethod(ObstaclesFromStereo::ELAS);
            pointCloudMaker->setSGBMParams(sgbmParams);
            pointCloudMaker->setGroundThresh(0.3);
            pointCloudMaker->setBackGroundThresh(20);
            pointCloudMaker->setLeafSize(0.05);
            
            cout << "Params1" << endl;
            pointCloudMaker->showCameraParams(cameraParams.at(0));
            cout << "Params2" << endl;
            pointCloudMaker->showCameraParams(cameraParams.at(1));
        }
        
        cv::imshow("imgL", left);
        cv::moveWindow("imgL", 0, 0);
        
        //         cv::imshow("imgR", right);

//         pointCloudMaker->generatePointClouds(left, right, leftMask);
        
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
// //                 yaw = 0.0;
// //                 speed = 0.0;
                deltaTime = egoValues[i].deltaTime;
                
                break;
            }
            case ObstaclesFromStereo::DAIMLER:
            case ObstaclesFromStereo::BAHNHOFSTRASSE:
            {
                yaw = 0.0;
                speed = 0.0;
                deltaTime = 1.0 / 13.0;
                
                break;
            }
        }
        
        posX += speed * deltaTime * sin(yaw);
        posY += speed * deltaTime * -cos(yaw);
        posTheta += yaw; 
        accTime += deltaTime;
        
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud = pointCloudMaker->getPointCloud();
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//         for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = pointCloud->begin(); 
//              it != pointCloud->end(); it++) {
//             
//             const pcl::PointXYZRGB & point = *it;
//             pcl::PointXYZRGB newPoint;
//             
//             newPoint.x = point.x;
//             newPoint.y = point.z;
//             newPoint.z = point.y;
//             newPoint.r = point.r;
//             newPoint.g = point.g;
//             newPoint.b = point.b;
//             
//             tmpPointCloud->push_back(newPoint);
//         }
//         
//         publishPointCloud(pointCloudPub, tmpPointCloud, i);
//         publishFakePointCloud(fakePointCloudPub, radius, i);
        
        rosgraph_msgs::Clock clockMsg;
        clockMsg.clock = ros::Time(accTime);
        clockPub.publish(clockMsg);
        
        ros::spinOnce();
        
        static tf::TransformBroadcaster broadcaster;
        tf::StampedTransform transform;
        // TODO: In a real application, time should be taken from the system
        transform.stamp_ = ros::Time::now();
        transform.setOrigin(tf::Vector3(-posX, -posY, 0.0));
        transform.setRotation( tf::createQuaternionFromRPY(0.0, 0.0, posTheta) );
        
        sensor_msgs::Image msgLeft, msgRight;
        cv_bridge::CvImage tmpLeft(msgLeft.header, sensor_msgs::image_encodings::BGR8, left);
        cv_bridge::CvImage tmpRight(msgRight.header, sensor_msgs::image_encodings::BGR8, right);
        leftCameraInfo.header.stamp = tmpLeft.header.stamp = ros::Time(accTime); //ros::Time::now();
        rightCameraInfo.header.stamp = tmpRight.header.stamp = ros::Time(accTime); //ros::Time::now();
        
        leftInfoPub.publish(leftCameraInfo);
        righttInfoPub.publish(rightCameraInfo);

        leftImgPub.publish(tmpLeft.toImageMsg());
        rightImgPub.publish(tmpRight.toImageMsg());
        
        visualization_msgs::MarkerArray & markersMsg = markers[i - 1];
        if (markersMsg.markers.size() > 0) {
//             BOOST_FOREACH(visualization_msgs::Marker & marker, markersMsg.markers) {
            for (uint32_t j = 0; j < markersMsg.markers.size(); j++) {
                visualization_msgs::Marker & marker = markersMsg.markers[j];
                marker.ns = "tmpMarkers";
                marker.id = j;
                marker.header.stamp = ros::Time::now();
                marker.header.frame_id = CAMERA_FRAME_ID;
            }
            markersPub.publish(markersMsg);
        }
        
//         rosgraph_msgs::Clock clockMsg;
//         clockMsg.clock = ros::Time(accTime);
//         clockPub.publish(clockMsg);
        
//         publishPointCloud(pointCloudPub, pointCloud);
//         broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/odom"));
        
//         map2odomTfBroadcaster.sendTransform(
//             tf::StampedTransform(
//                 tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(posX, posY, 0.0)),
//                                  ros::Time::now(), "map", "odom"));
        
        // Publish point cloud
        
        uint8_t keycode;
//         if (i < 15)
//             keycode = cv::waitKey(0);
//         else
        cout << "deltaTime " << deltaTime << endl;
//             keycode = cv::waitKey((uint32_t)(deltaTime * 2000));
            keycode = cv::waitKey(0);
        if (keycode == 27) {
            break;
        }
        ros::spinOnce();
        //         }
        
        //         if (i != initialIdx)
        //             break;
//         ros::spinOnce();
    }
}

int main(int argC, char **argV) {
    ros::init(argC, argV, "stereo_and_odom");
    
    //     if (fork() == 0) {
    //         testPointCloud();
    //     }
    testStereoTracking();
}