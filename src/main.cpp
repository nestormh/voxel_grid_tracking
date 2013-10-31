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

#include <iostream>
#include <iomanip>
#include <boost/filesystem.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/opencv.hpp>

using namespace std;

// Function declarations
void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud);
void testStereo();

// Definitions
void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->addCoordinateSystem();    
    
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (pointCloud, rgb, "pointCloud");
    
    
    clock_t tIni = 0; 
    while (! viewer->wasStopped ()) {    
        viewer->spinOnce();
    }
}

void testStereo() {
    const uint32_t initialIdx = 58;
    const boost::filesystem::path correspondencesPath("/local/imaged/calibrated/cdvp_3d_pedestrian_detection_dataset_vicon_1");
    const boost::filesystem::path seqName("Rectified_Images");
    const boost::filesystem::path maskName("../resultsHierarchical/pixel");
    
    vector<polar_grid_tracking::t_Camera_params> cameraParams;
    ObstaclesFromStereo::getParamsFromDublinDataset("/local/imaged/calibrated/cdvp_3d_pedestrian_detection_dataset_vicon_1/Groundtruth3d/GroundtruthPlane.txt", cameraParams);

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
    
    for (uint32_t i = initialIdx; i < 1000; i++) {
        stringstream ss;
        ss << setfill('0') << setw(3) << i;
        stringstream ss2;
        ss2 << i;
        
        boost::filesystem::path leftPath = correspondencesPath / seqName / boost::filesystem::path("seq." + ss.str() + ".left.png");
        boost::filesystem::path rightPath = correspondencesPath / seqName / boost::filesystem::path("seq." + ss.str() + ".right.png");
        boost::filesystem::path leftMaskPath = correspondencesPath / seqName / maskName / boost::filesystem::path("Image" + ss.str() + ".png");
        
        cout << leftPath << endl;
        cout << rightPath << endl;
        cout << leftMaskPath << endl;
        
        cv::Mat left = cv::imread(leftPath.string());
        cv::Mat right = cv::imread(rightPath.string());
        
        cv::Mat leftMask;
        ObstaclesFromStereo::getFGMask(leftMaskPath.string(), leftMask, cv::Size(left.cols, left.rows));
        
        if (i == initialIdx) {
            pointCloudMaker.reset(new ObstaclesFromStereo(cv::Size(left.cols, left.rows)));
            pointCloudMaker->setCameraParams(cameraParams.at(0), cameraParams.at(1));
            pointCloudMaker->setMethod(ObstaclesFromStereo::SGBM);
            pointCloudMaker->setSGBMParams(sgbmParams);
            pointCloudMaker->setGroundThresh(-0.2);
            pointCloudMaker->setBackGroundThresh(100);
            pointCloudMaker->setLeafSize(0.05);
        }
        
        pointCloudMaker->generatePointClouds(left, right, leftMask);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud = pointCloudMaker->getPointCloud();
        
        visualizePointCloud(pointCloud);
        
        break;
    }
}

int main(int argc, char **argV) {
    testStereo();
    
    return 0;
}