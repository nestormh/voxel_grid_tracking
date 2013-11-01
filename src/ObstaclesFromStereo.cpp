/*
    Copyright (c) 2013, Néstor Morales Hernández <email>
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Néstor Morales Hernández <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Néstor Morales Hernández <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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

#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

ObstaclesFromStereo::ObstaclesFromStereo(const cv::Size & size) : m_method(SGBM), m_size(size), 
                                         m_groundThresh(0.2), m_backgroundThresh(100), m_leafSize(0.05) {

}

ObstaclesFromStereo::~ObstaclesFromStereo() {

}

void ObstaclesFromStereo::generatePointClouds(const cv::Mat& leftImg, const cv::Mat& rightImg, const cv::Mat& mask) {
    
    cv::Mat leftGray(leftImg.rows, leftImg.cols, CV_8UC1);
    cv::Mat rightGray(leftImg.rows, leftImg.cols, CV_8UC1);
    cv::Mat disp8(leftImg.rows, leftImg.cols, CV_64F);
    cvtColor(leftImg, leftGray, CV_BGR2GRAY);
    cvtColor(rightImg, rightGray, CV_BGR2GRAY);
    
    cv::Mat disp;    
    switch (m_method) {
        case SGBM: {
            cv::StereoSGBM stereo = cv::StereoSGBM(m_SGBM_params.minDisparity, m_SGBM_params.numDisparities, m_SGBM_params.SADWindowSize,
                                    m_SGBM_params.SADWindowSize * m_SGBM_params.SADWindowSize * m_SGBM_params.P1,
                                    m_SGBM_params.SADWindowSize * m_SGBM_params.SADWindowSize * m_SGBM_params.P2,
                                    m_SGBM_params.disp12MaxDiff, m_SGBM_params.preFilterCap, m_SGBM_params.uniquenessRatio,
                                    m_SGBM_params.speckleWindowSize, m_SGBM_params.speckleRange, m_SGBM_params.fullDP);
            stereo(leftGray, rightGray, disp);
            
            break;
        }
        case STEREOVAR: {
            cv::StereoVar stereo;
            stereo(leftGray, rightGray, disp);
            
            break;
        }
    }

    disp.convertTo(disp8, CV_64F, 1./16.);

    //Create point cloud and fill it
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr unmaskedPointCloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
    m_pointCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    unmaskedPointCloud->reserve(leftGray.rows * leftGray.cols);

    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> pointMat(3, 1);
    for (uint32_t i = 0; i < leftImg.rows; i++) {
        
        const char* rgb_ptr = leftImg.ptr<char>(i);
        const uchar* mask_ptr = mask.ptr<uchar>(i);        
        double* disp_ptr = disp8.ptr<double>(i);
        
        for (int j = 0; j < leftImg.cols; j++) {
            double d = disp_ptr[j];
            
            if (d <= 0) continue;
            
            double norm = (double)d / (double)(m_leftCameraParams.ku * m_rightCameraParams.baseline);

            // Get 3D coordinates
            pcl::PointXYZRGBL point;

            point.x = (((j - m_leftCameraParams.u0) / m_leftCameraParams.ku) / norm);
            point.y = (((i - m_leftCameraParams.v0) / m_leftCameraParams.kv) / norm);
            point.z = 1.0 / norm;
            
            pointMat << point.x - m_leftCameraParams.t.data()[0], point.y - m_leftCameraParams.t.data()[1], point.z - m_leftCameraParams.t.data()[2];
            pointMat = m_leftCameraParams.R * pointMat;
            
//             point.x = pointMat.data()[0];
//             point.y = pointMat.data()[1];
//             point.z = pointMat.data()[2];

            point.x = pointMat.data()[1];
            point.y = pointMat.data()[2];
            point.z = pointMat.data()[0];
            
            //Get RGB info
            point.b = rgb_ptr[3*j];
            point.g = rgb_ptr[3*j+1];
            point.r = rgb_ptr[3*j+2];
            
            point.label = ((uint32_t)mask_ptr[j]) & 255;

            unmaskedPointCloud->points.push_back(point);
        }
    }
    unmaskedPointCloud->width = (int) unmaskedPointCloud->points.size();
    unmaskedPointCloud->height = 1;
    
    removeGround(unmaskedPointCloud);
    
    filterMasked(unmaskedPointCloud, m_pointCloud);
    
    downsample(m_pointCloud);
    
    std::cout << "Points.size() = " << m_pointCloud->size() << std::endl;
        
//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//     viewer->setBackgroundColor (0, 0, 0);
//     viewer->initCameraParameters();
//     viewer->addCoordinateSystem();    
//     
//     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(m_pointCloud);
//     viewer->addPointCloud<pcl::PointXYZRGB> (m_pointCloud, rgb, "pointCloud");
//     
//     
//     clock_t tIni = 0; 
//     while (! viewer->wasStopped ()) {    
//         viewer->spinOnce();
//     }
}

inline void ObstaclesFromStereo::setParamsGeometry(t_Camera_params & params) {
    
    if ((params.width == m_size.width) && (params.height == m_size.height))
        return;
    
    // Modificamos el centro
    if (params.u0 != 0.0) {
        params.u0 *= (double) m_size.width / (double) params.width;
    } else {
        params.u0 = (double) m_size.width / 2.0;
    }

    if (params.v0 != 0.0) {
        params.v0 *= (double) m_size.height / (double) params.height;
    } else {
        params.v0 = (double) m_size.height / 2.0;
    }

    // Modificamos la focal
    params.ku *= (double) m_size.width / (double) params.width;
    params.kv *= (double) m_size.height / (double) params.height;
    
//     params.y *= (double) width / (double) params.width;    
//     params.z *= (double) height / (double) params.height;

    params.width = m_size.width;
    params.height = m_size.height;
}

void ObstaclesFromStereo::setCameraParams(const t_Camera_params& leftCameraParams, 
                                          const t_Camera_params& rightCameraParams) {

    m_leftCameraParams = leftCameraParams;
    m_rightCameraParams = rightCameraParams;
    
    setParamsGeometry(m_leftCameraParams);
    setParamsGeometry(m_rightCameraParams);
}

void ObstaclesFromStereo::filterMasked(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr & inputCloud, 
                                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & outputCloud) {
    outputCloud->resize(inputCloud->size());
    for (size_t i = 0; i < inputCloud->size(); i++) {
        if (inputCloud->points[i].label != 0) {
            outputCloud->points[i].x = inputCloud->points[i].x;
            outputCloud->points[i].y = inputCloud->points[i].y;
            outputCloud->points[i].z = inputCloud->points[i].z;
            outputCloud->points[i].r = inputCloud->points[i].r;
            outputCloud->points[i].g = inputCloud->points[i].g;
            outputCloud->points[i].b = inputCloud->points[i].b;
        }
    }
}

void ObstaclesFromStereo::removeGround(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr & pointCloud) {

    if ((m_groundThresh >= 0) && (m_backgroundThresh >= 0)) {
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tmpPointCloud(new pcl::PointCloud<pcl::PointXYZRGBL>);

        std::vector<int> inliers;
        inliers.reserve(pointCloud->size());    
        for (uint32_t i = 0; i < pointCloud->size(); i += 2) {
            if ((pointCloud->points.at(i).z < m_groundThresh) && (pointCloud->points.at(i).x < m_backgroundThresh))
                inliers.push_back(i);
        }        
        pcl::copyPointCloud<pcl::PointXYZRGBL>(*pointCloud, inliers, *tmpPointCloud);
        
        if (tmpPointCloud->size() < 4) {
            std::cerr << "There are not enough points to continue removing the ground..." << std::endl;
            return;
        }

        pcl::SampleConsensusModelPlane<pcl::PointXYZRGBL>::Ptr model_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBL> (tmpPointCloud));    
        pcl::RandomSampleConsensus<pcl::PointXYZRGBL> ransac (model_plane);
        ransac.setDistanceThreshold (0.02);
        ransac.setProbability(.99);
        ransac.setMaxIterations(500);
        
        ransac.computeModel(0);    
        inliers.clear();
        ransac.getInliers(inliers);
        pcl::copyPointCloud<pcl::PointXYZRGBL>(*tmpPointCloud, inliers, *tmpPointCloud);
        
        if (inliers.size() < 4) {
            std::cerr << "There are not enough inliers to continue removing the ground..." << std::endl;
            return;
        }
        
        model_plane->setInputCloud(tmpPointCloud);

        ransac.computeModel(0);  

        pcl::copyPointCloud<pcl::PointXYZRGBL>(*pointCloud, *tmpPointCloud);
        pointCloud->points.clear();
        Eigen::VectorXf modelCoeffs;
        ransac.getModelCoefficients(modelCoeffs);
    
        std::vector<double> distances;
        model_plane = pcl::SampleConsensusModelPlane<pcl::PointXYZRGBL>::Ptr (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGBL> (tmpPointCloud));    
        model_plane->getDistancesToModel(modelCoeffs, distances);
        
        inliers.clear();
        inliers.reserve(distances.size());
        for (uint32_t i = 0; i < distances.size(); i++) { 
            if ((distances[i] > m_groundThresh) && (tmpPointCloud->points.at(i).x < m_backgroundThresh)) {
                inliers.push_back(i);
            }
        }
        
        pcl::copyPointCloud<pcl::PointXYZRGBL>(*tmpPointCloud, inliers, *pointCloud);
    }
}

void ObstaclesFromStereo::downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud) {    
    if (m_leafSize >= 0) {
        pcl::VoxelGrid<pcl::PointXYZRGB> ds;  //create downsampling filter 
        ds.setInputCloud (pointCloud); 
        ds.setLeafSize (m_leafSize, m_leafSize, m_leafSize); 
        ds.filter (*pointCloud); 
    }
}

// More information about this method can be found at: http://nestormh.hol.es/?p=244
void ObstaclesFromStereo::getParamsFromDublinDataset(const std::string& planeFilename, std::vector<t_Camera_params> & params, 
                                                     double baseline, double focalLength, cv::Size sz) {
    
    double A, B, C, D;
    std::ifstream fin(planeFilename.c_str());
    fin >> A;
    fin >> B;
    fin >> C;
    fin >> D;
    fin.close();
    
    Eigen::Vector3d p0(0.0, -D/B, 0.0);
    Eigen::Vector3d p1(0.0, 0.0, -D/C);
    Eigen::Vector3d p2(-D/A, 0.0, 0.0);
    
    Eigen::Vector3d v1(0.0, D/B, -D/C);
    Eigen::Vector3d v2(-D/A, D/B, 0.0);
    Eigen::Vector3d n, v3;
    
    n = v1.cross(v2);
    if (n.data()[1] > 0)
        n = v2.cross(v1);
    v3 = n.cross(v1);
    
    t_Camera_params leftCameraParams;
    leftCameraParams.width = sz.width;
    leftCameraParams.height = sz.height;
    leftCameraParams.ku = focalLength;
    leftCameraParams.kv = focalLength;
    leftCameraParams.u0 = sz.width / 2.0;
    leftCameraParams.v0 = sz.height / 2.0;
    leftCameraParams.baseline = baseline;
    
    t_Camera_params rightCameraParams;
    rightCameraParams.width = sz.width;
    rightCameraParams.height = sz.height;
    rightCameraParams.ku = focalLength;
    rightCameraParams.kv = focalLength;
    rightCameraParams.u0 = sz.width / 2.0;
    rightCameraParams.v0 = sz.height / 2.0;
    rightCameraParams.baseline = baseline;
    
    v1 /= v1.norm();
    v3 /= v3.norm();
    n /= n.norm();
    
    rightCameraParams.R = Eigen::MatrixXd(3, 3);
    rightCameraParams.R << v1.data()[0], v1.data()[1], v1.data()[2],
                           v3.data()[0], v3.data()[1], v3.data()[2],
                           n.data()[0], n.data()[1], n.data()[2];
    rightCameraParams.t = Eigen::MatrixXd(3, 1);
    rightCameraParams.t << p0.data()[0], p0.data()[1], p0.data()[2];
    
    leftCameraParams.R = rightCameraParams.R;
    leftCameraParams.t = Eigen::MatrixXd(3, 1);
    leftCameraParams.t << p0.data()[0], p0.data()[1] + baseline, p0.data()[2];
    
    params.push_back(leftCameraParams);
    params.push_back(rightCameraParams);
}

// FIXME: This test will not work properly with the new version of ObstaclesFromStereo class
void ObstaclesFromStereo::readi3DPostCalibrationFile(const std::string& fileName, std::vector< t_Camera_params >& cameraParams) {
    
    std::ifstream fin(fileName.c_str());
    
    if (!fin.is_open()) {
        std::cerr << "[ERROR][" << __func__ << " (" << __FILE__ << ":" << __LINE__ << ")] \n\t"
                  << "File " << fileName << " could not be open." << std::endl;        
        exit(1);
    }
    
    uint32_t nCameras, distModel;
    fin >> nCameras;
    fin >> distModel;
    
    cameraParams.reserve(nCameras);
    
    std::cout << "nCameras = " << nCameras << std::endl;
    
    for (uint32_t i = 0; i < nCameras; i++) {
        t_Camera_params params;
        
        fin >> params.minY;
        fin >> params.height;
        fin >> params.minX;
        fin >> params.width;
        
        fin >> params.ku;
        fin >> params.kv;
        fin >> params.u0;
        fin >> params.v0;
        
        // FIXME: This information is not being used
//         fin >> params.distortion;
        
        params.R = Eigen::MatrixXd(3, 3);
        params.t = Eigen::MatrixXd(3, 1);
        
        for (uint32_t j = 0; j < 9; j++)
            fin >> params.R.data()[j];
        
        for (uint32_t j = 0; j < 3; j++)
            fin >> params.t.data()[j];
        
        cameraParams.push_back(params);
    }
    
    fin.close();   
}

// TODO: This function should dissappear once the foreground detection class is created
void ObstaclesFromStereo::getFGMask(const std::string & fileName, cv::Mat & fgMask, const cv::Size & sz) {
    cv::Mat tmpImg = cv::imread(fileName);
    cv::Mat tmpMask;
    
    cv::Mat b, g, r;
    cv::Mat channels[3];
    
    cv::split(tmpImg, channels);
    
    b = channels[0];
    g = channels[1];
    r = channels[2];
    
    cv::bitwise_not(g, g);
    cv::bitwise_not(r, r);
    
    cv::bitwise_and(b, g, b);
    cv::bitwise_and(b, r, tmpMask);
    
    cv::erode(tmpMask, tmpMask, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(tmpMask, tmpMask, cv::Mat(), cv::Point(-1, -1), 3);
    
    if ((tmpMask.cols != sz.width) || (tmpMask.rows != sz.height)) {
        resize(tmpMask, fgMask, sz, 0, 0, cv::INTER_NEAREST);
    } else {
        fgMask = tmpMask;
    }
}