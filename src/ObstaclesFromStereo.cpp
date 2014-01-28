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
#include "tracklets.h"

#include <fstream>
#include <iostream>
#include <string>
#include <limits>

#include <boost/foreach.hpp>

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <tf/transform_datatypes.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/filesystem.hpp>

#include "elas.h"

using namespace std;

ObstaclesFromStereo::ObstaclesFromStereo(const cv::Size & size, const t_CalibrationFileType  & calibrationType) : m_method(SGBM), m_size(size), 
                                         m_groundThresh(0.2), m_backgroundThresh(100), m_leafSize(0.05), m_calibrationType(calibrationType) {

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
            
            disp.convertTo(disp8, CV_64F, 1./16.);
            
            break;
        }
        case STEREOVAR: {
            cv::StereoVar stereo;
            stereo(leftGray, rightGray, disp);
            
            disp.convertTo(disp8, CV_64F, 1./16.);
            
            break;
        }
        case ELAS: {
            
            Elas elas(Elas::parameters(Elas::ROBOTICS));
            
            // matching function
            // inputs: pointers to left (I1) and right (I2) intensity image (uint8, input)
            //         pointers to left (D1) and right (D2) disparity image (float, output)
            //         dims[0] = width of I1 and I2
            //         dims[1] = height of I1 and I2
            //         dims[2] = bytes per line (often equal to width, but allowed to differ)
            //         note: D1 and D2 must be allocated before (bytes per line = width)
            //               if subsampling is not active their size is width x height,
            //               otherwise width/2 x height/2 (rounded towards zero)
            leftGray.data;
            int32_t dims[3];
            dims[0] = leftGray.cols;
            dims[1] = leftGray.rows;
            dims[2] = leftGray.cols;
    
            float * D1 = new float[leftGray.cols * leftGray.rows];
            float * D2 = new float[rightGray.cols * rightGray.rows];
            
            elas.process((uint8_t *)leftGray.data, (uint8_t *)rightGray.data, D1, D2, dims);
            
            for (uint32_t y = 0; y < leftGray.rows; y++) {
                for (uint32_t x = 0; x < leftGray.cols; x++) {
                    disp8.at<double>(y, x) = D1[y * leftGray.cols + x];
                }
            }

            delete D1;
            delete D2;
            
            break;
        }
    }

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
//             pcl::PointXYZRGBL point;
            pcl::PointXYZRGB point;

            switch(m_calibrationType) {
                case KARLSRUHE:
                case KARLSRUHE_V2:
                case BAHNHOFSTRASSE:
                case DAIMLER:
                {
                    point.x = (((m_leftCameraParams.u0 - j) / m_leftCameraParams.ku) / norm);
//                     point.y = (((m_leftCameraParams.v0 - i)/ m_leftCameraParams.kv) / norm)/* + 1.65*/;
                    point.y = -(((i - m_leftCameraParams.v0) / m_leftCameraParams.kv) / norm)/* + 1.65*/;
                    point.z = 1.0 / norm;
                    
                    pointMat << point.x - m_leftCameraParams.t.data()[0], point.y - m_leftCameraParams.t.data()[1], point.z - m_leftCameraParams.t.data()[2];
                    pointMat = m_leftCameraParams.R * pointMat;
                                
                    point.x = pointMat.data()[0];
                    point.y = pointMat.data()[1];
                    point.z = pointMat.data()[2];
                                        
                    break;
                }
                case DUBLIN: 
                {
                    point.x = (((j - m_leftCameraParams.u0) / m_leftCameraParams.ku) / norm);
                    point.y = (((i - m_leftCameraParams.v0) / m_leftCameraParams.kv) / norm);
                    point.z = 1.0 / norm;
                    
                    pointMat << point.x - m_leftCameraParams.t.data()[0], point.y - m_leftCameraParams.t.data()[1], point.z - m_leftCameraParams.t.data()[2];
                    pointMat = m_leftCameraParams.R * pointMat;
                    
                    point.x = pointMat.data()[1];
                    point.y = pointMat.data()[2];
                    point.z = pointMat.data()[0];
                }
            }
            
            //Get RGB info
            point.b = rgb_ptr[3*j];
            point.g = rgb_ptr[3*j+1];
            point.r = rgb_ptr[3*j+2];
            
//             point.label = ((uint32_t)mask_ptr[j]) & 255;

//             unmaskedPointCloud->points.push_back(point);
            m_pointCloud->points.push_back(point);
        }
    }
    m_pointCloud->width = (int) m_pointCloud->points.size();
    m_pointCloud->height = 1;

//     unmaskedPointCloud->width = (int) unmaskedPointCloud->points.size();
//     unmaskedPointCloud->height = 1;
    
//     removeGround(unmaskedPointCloud);
//     filterMasked(unmaskedPointCloud, m_pointCloud);
        
    BOOST_FOREACH(pcl::PointXYZRGB & point, m_pointCloud->points) {
        const double z = point.z;
        point.x = -point.x;
        point.z = point.y;
        point.y = z;
    }
    
//     downsample(m_pointCloud);
    
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
        cout << __LINE__ << endl;
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tmpPointCloud(new pcl::PointCloud<pcl::PointXYZRGBL>);

        std::vector<int> inliers;
        inliers.reserve(pointCloud->size());    
/*        for (uint32_t i = 0; i < pointCloud->size(); i += 2) {
            if ((pointCloud->points.at(i).y >= -m_groundThresh) && (pointCloud->points.at(i).y < m_groundThresh) && (pointCloud->points.at(i).z < m_backgroundThresh))
                inliers.push_back(i);
        } */       
// //         pcl::copyPointCloud<pcl::PointXYZRGBL>(*pointCloud, inliers, *tmpPointCloud);

        for (uint32_t i = 0; i < pointCloud->size(); i += 2) {
            if ((pointCloud->points.at(i).y > m_groundThresh) && (pointCloud->points.at(i).z < m_backgroundThresh))
                inliers.push_back(i);
        } 
        pcl::copyPointCloud<pcl::PointXYZRGBL>(*pointCloud, inliers, *pointCloud);
        
        return;
        
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
            if ((distances[i] > m_groundThresh) && (tmpPointCloud->points.at(i).z < m_backgroundThresh)) {
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

void ObstaclesFromStereo::getParams(const std::string& fileName, std::vector< t_Camera_params >& params, const ObstaclesFromStereo::t_CalibrationFileType& calibrationFileType)
{
    cout << __FUNCTION__ << ":" << __LINE__ << endl;
    switch (calibrationFileType) {
        case DUBLIN: return getParamsFromDublinDataset(fileName, params);
        case KARLSRUHE: return getParamsFromKarlsruhe(fileName, params);
        case KARLSRUHE_V2: return getParamsFromKarlsruhe_v2(fileName, params);
        case BAHNHOFSTRASSE: return getParamsFromBahnhofstrasse(fileName, params);
        case DAIMLER: return getParamsFromBahnhofstrasse(fileName, params);
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

void ObstaclesFromStereo::getParamsFromKarlsruhe(const std::string& fileName, std::vector< t_Camera_params >& params)
{
    ifstream fin(fileName.c_str());

//     // TODO: Get w, h from initial image
//     const double width = 1392;
//     const double height = 512;
    
    t_Camera_params leftCameraParams;
    leftCameraParams.width = 1344;
    leftCameraParams.height = 391;
    
    t_Camera_params rightCameraParams;
    rightCameraParams.width = 1344;
    rightCameraParams.height = 391;

    for (uint32_t i = 0; i < 9; i++)
        fin.ignore(numeric_limits<int>::max(), '\n');
    
    fin.ignore(numeric_limits<int>::max(), ' ');
    leftCameraParams.R = Eigen::MatrixXd(3, 3);
    for (uint32_t i = 0; i < 3; i++)
        for (uint32_t j = 0; j < 3; j++)
            fin >> leftCameraParams.R(i, j);
    rightCameraParams.R = leftCameraParams.R;
    
    fin.ignore(numeric_limits<int>::max(), ' ');    
    leftCameraParams.t = Eigen::MatrixXd(3, 1);
    for (uint32_t i = 0; i < 3; i++)
        fin >> leftCameraParams.t(i);
    leftCameraParams.t(1) -= 1.0; //1.65;
    rightCameraParams.t = leftCameraParams.t;
        
    fin.ignore(numeric_limits<int>::max(), '\n');
    fin.ignore(numeric_limits<int>::max(), '\n');
    fin.ignore(numeric_limits<int>::max(), ' '); 
    
    Eigen::MatrixXd P(3, 4);
    for (uint32_t i = 0; i < 3; i++)
        for (uint32_t j = 0; j < 4; j++)
            fin >> P(i, j);
    
    leftCameraParams.ku = P(0, 0);
    leftCameraParams.kv = P(1, 1);
    leftCameraParams.u0 = P(0, 2);
    leftCameraParams.v0 = P(1, 2);

    fin.ignore(numeric_limits<int>::max(), '\n');
    fin.ignore(numeric_limits<int>::max(), '\n');
    fin.ignore(numeric_limits<int>::max(), ' ');    
    
    for (uint32_t i = 0; i < 3; i++)
        for (uint32_t j = 0; j < 4; j++)
            fin >> P(i, j);
        
    rightCameraParams.ku = P(0, 0);
    rightCameraParams.kv = P(1, 1);
    rightCameraParams.u0 = P(0, 2);
    rightCameraParams.v0 = P(1, 2);
    
    rightCameraParams.baseline = - P(0, 3) / rightCameraParams.ku;
    
    leftCameraParams.baseline = rightCameraParams.baseline;
    
    fin.close();
    
    params.push_back(leftCameraParams);
    params.push_back(rightCameraParams);
}

void ObstaclesFromStereo::getParamsFromKarlsruhe_v2(ifstream& fin, t_Camera_params & params)
{
    params.width = 1244;
    params.height = 370;
    
    fin.ignore(numeric_limits<int>::max(), '\n');           // S_XX
    fin.ignore(numeric_limits<int>::max(), '\n');           // K_XX
    fin.ignore(numeric_limits<int>::max(), '\n');           // D_XX
    
    fin.ignore(numeric_limits<int>::max(), ' ');
    params.R = Eigen::MatrixXd(3, 3);
    for (uint32_t i = 0; i < 3; i++)
        for (uint32_t j = 0; j < 3; j++)
            fin >> params.R(i, j);
        
    fin.ignore(numeric_limits<int>::max(), ' ');    
    params.t = Eigen::MatrixXd(3, 1);
    for (uint32_t i = 0; i < 3; i++)
        fin >> params.t(i);
    params.t(1) -= 1.0;
    
    fin.ignore(numeric_limits<int>::max(), '\n');
    fin.ignore(numeric_limits<int>::max(), '\n');
    fin.ignore(numeric_limits<int>::max(), '\n');
    fin.ignore(numeric_limits<int>::max(), ' '); 
    
    Eigen::MatrixXd P(3, 4);
    for (uint32_t i = 0; i < 3; i++)
        for (uint32_t j = 0; j < 4; j++)
            fin >> P(i, j);
    fin.ignore(numeric_limits<int>::max(), '\n');
    
    params.ku = P(0, 0);
    params.kv = P(1, 1);
    params.u0 = P(0, 2);
    params.v0 = P(1, 2);
    
    params.baseline = - P(0, 3) / params.ku;
    
}

void ObstaclesFromStereo::getParamsFromKarlsruhe_v2(const string& fileName, vector< t_Camera_params >& params)
{
    ifstream fin(fileName.c_str());
    
    fin.ignore(numeric_limits<int>::max(), '\n');
    fin.ignore(numeric_limits<int>::max(), '\n');
    
    vector< t_Camera_params > tmpParams(4);
    for (uint32_t i = 0; i < 4; i++) {
        getParamsFromKarlsruhe_v2(fin, tmpParams[i]);
    }
    
    fin.close();
    
    tmpParams[2].baseline = tmpParams[3].baseline;
    params.push_back(tmpParams[2]);
    params.push_back(tmpParams[3]);
    
}

void ObstaclesFromStereo::getParamsFromBahnhofstrasseSingleFile(const string& fileName, t_Camera_params& params)
{
    cout << __FUNCTION__ << ":" << __LINE__ << endl;
    params.width = 640;
    params.height = 480;
    
    double dummy;
    ifstream fin(fileName.c_str());
    
    fin >> params.ku;
    fin >> dummy;
    fin >> params.u0;
    
    fin >> dummy;
    fin >> params.kv;
    fin >> params.v0;
    
    fin >> dummy;
    fin >> dummy;
    fin >> dummy;
    
    fin >> params.distortion;
    fin >> dummy;
    fin >> dummy;
    fin >> dummy;
    
    params.R = Eigen::MatrixXd(3, 3);
    for (uint32_t i = 0; i < 3; i++)
        for (uint32_t j = 0; j < 3; j++)
            fin >> params.R(i, j);
        
    params.t = Eigen::MatrixXd(3, 1);
    for (uint32_t i = 0; i < 3; i++) {
        fin >> params.t(i);
//         params.t(i) /= 100.0;
    }
        
    fin.close();
}

void ObstaclesFromStereo::getParamsFromBahnhofstrasse(const string& fileName, vector< t_Camera_params >& params)
{
    cout << __FUNCTION__ << ":" << __LINE__ << endl;
    boost::filesystem3::path path(fileName);
    params.resize(2);
    cout << __FUNCTION__ << ":" << __LINE__ << endl;
    cout << (path / boost::filesystem3::path("cam1.cal")).string() << endl;
    getParamsFromBahnhofstrasseSingleFile((path / boost::filesystem3::path("cam1.cal")).string(), params[0]);
    getParamsFromBahnhofstrasseSingleFile((path / boost::filesystem3::path("cam2.cal")).string(), params[1]);
    
        params[0].baseline = -params[1].t(0);
        params[1].baseline = -params[1].t(0);
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


void ObstaclesFromStereo::showCameraParams(const t_Camera_params& params)
{
    cout << "Params: " << endl;
    cout << "minX " << params.minX << endl;
    cout << "minY " << params.minY << endl;
    cout << "width " << params.width << endl;
    cout << "height " << params.height << endl;
    cout << "u0 " << params.u0 << endl;
    cout << "v0 " << params.v0 << endl;
    cout << "ku " << params.ku << endl;
    cout << "kv " << params.kv << endl;
    cout << "distortion " << params.distortion << endl;
    cout << "baseline " << params.baseline << endl;
    cout << "R " << endl << params.R << endl;
    cout << "t " << endl << params.t << endl;
}

void ObstaclesFromStereo::readCurrentEgoValue(ifstream& fin, t_ego_value& egoValue)
{
    double values[8];
    for (uint32_t i = 0; i <= 8; i++) {
        fin >> values[i];
    }
    egoValue.deltaYaw = values[5];
    egoValue.speed = values[8];
}

void ObstaclesFromStereo::readEgoValues(const std::string & pathName, vector< t_ego_value >& egoValues)
{
    boost::filesystem3::path basePath(pathName);
    boost::filesystem3::path timestampsPath("oxts/timestamps.txt");
    
    ifstream fin((basePath / timestampsPath).string().c_str());
    
    string date, time;
    boost::posix_time::ptime lastTime;
    bool initialized = false;
    while (! fin.eof()) {
        fin >> date;
        fin >> time;
        
        boost::posix_time::ptime currTime = boost::posix_time::time_from_string(date + " " + time);
        
        t_ego_value egoVal;
        if (initialized) {
            boost::posix_time::time_duration diff = currTime - lastTime;
            egoVal.deltaTime = diff.total_milliseconds() / 1000.0;
            
        } else {
            egoVal.deltaTime = 0.0;
            initialized = true;
        }
        egoValues.push_back(egoVal);

        lastTime = currTime;
    }
    fin.close();
    
    
    for (uint32_t i = 0; i < egoValues.size(); i++) {
        stringstream ss;
        ss << "oxts/data/";
        ss << setfill('0') << setw(10) << i;
        ss << ".txt";

        boost::filesystem3::path filePath(ss.str());
        fin.open((basePath / filePath).string().c_str());
        if (fin.is_open()) {
            
            readCurrentEgoValue(fin, egoValues[i]);
            
            fin.close();
        } else {
            cout << "Not found: " << (basePath / filePath).string() << endl;
            egoValues.erase(egoValues.begin() + i);
            i--;
        }
    }
    
    for (uint32_t i = egoValues.size() - 1; i > 0; i--) {
        const double & yaw2 = egoValues[i].deltaYaw;
        const double & yaw1 = egoValues[i - 1].deltaYaw;
        const double deltaYaw = atan2(sin(yaw2 - yaw1), cos(yaw2 - yaw1));
        
        egoValues[i].deltaYaw = deltaYaw;
        egoValues[i].deltaPos = egoValues[i].speed * egoValues[i].deltaTime;
    }
    
    egoValues[0].deltaYaw = 0;
    egoValues[0].deltaPos = 0;
}

vector< PolarGridTracking::roiArray > ObstaclesFromStereo::readROIList(const string & trackletsPath, const uint32_t & sequenceLength)
{
//     cout << "trackletsPath " << trackletsPath << endl;
//     
//     Tracklets tracklets;
//     if (! tracklets.loadFromFile(trackletsPath)) {
//         ROS_FATAL("Tracklets file could not be opened");
//         exit(0);
//     }
// 
//     vector< PolarGridTracking::roiArray > trackletList(sequenceLength);
//     BOOST_FOREACH(PolarGridTracking::roiArray & roiMsg, trackletList) {
//         roiMsg.rois2d.resize(sequenceLength);
//         roiMsg.rois3d.resize(sequenceLength);
//     }
// //     PolarGridTracking::roiArray roiMsg;
// //     roiMsg.rois3d.resize(m_obstacles.size());
// //     roiMsg.rois2d.resize(m_obstacles.size());
// //     
// //     roiMsg.id = m_currentId;
// //     roiMsg.header.frame_id = DEFAULT_BASE_FRAME;
// //     roiMsg.header.stamp = ros::Time::now();
//     
//     for (uint32_t i = 0; i < sequenceLength; i++) {
//         PolarGridTracking::roiArray & roiMsg = trackletList[i];
//         
//         for (uint32_t j = 0; j < tracklets.numberOfTracklets(); j++) {
//             Tracklets::tTracklet & tracklet =  *(tracklets.getTracklet(j));
//             Tracklets::tPose * pose;
//             if (tracklets.getPose(j, i, pose)) {
//                 roiMsg.rois3d[j].A.x = 
//             }
//         }
// //         const vector<Tracklets::tPose> & poses = tracklet.poses;
// //         for (uint32_t idx = tracklet.first_frame, j = 0; j <= poses.size(); j++, idx++) {
// //             const Tracklets::tPose & pose = poses[j];
// //             
// //             PolarGridTracking::roiArray & roiMsg = trackletList[idx];
// //             
// //             // TODO: Compensate the Velodyne Rt in order to reference obstacle w.r.t the left camera.
// //             roiMsg.rois3d.resize(tracklets.s);
// //         }
//     }
//     
//     exit(0);
}

vector< visualization_msgs::MarkerArray > ObstaclesFromStereo::readMarkerList(const string & trackletsPath, const uint32_t & sequenceLength)
{
    Tracklets tracklets;
    if (! tracklets.loadFromFile(trackletsPath)) {
        ROS_FATAL("Tracklets file could not be opened");
        exit(0);
    }
    
    vector< visualization_msgs::MarkerArray > trackletList(sequenceLength);
    BOOST_FOREACH(visualization_msgs::MarkerArray & markers, trackletList) {
        markers.markers.resize(sequenceLength);
    }
    
    for (uint32_t i = 0; i < sequenceLength; i++) {
        visualization_msgs::MarkerArray & markers = trackletList[i];
        
        for (uint32_t j = 0; j < tracklets.numberOfTracklets(); j++) {
            Tracklets::tTracklet & tracklet =  *(tracklets.getTracklet(j));
            Tracklets::tPose * pose;
            if (tracklets.getPose(j, i, pose)) {
                visualization_msgs::Marker voxelMarker;
                voxelMarker.ns = "voxels";
                voxelMarker.type = visualization_msgs::Marker::CUBE;
                voxelMarker.action = visualization_msgs::Marker::ADD;
                
                voxelMarker.pose.position.x = -pose->ty;
                voxelMarker.pose.position.y = pose->tx/* - 1.8*/;
                voxelMarker.pose.position.z = -(tracklet.h / 2.0 + pose->tz);
                
                const tf::Quaternion & quat = tf::createQuaternionFromRPY(pose->rx, pose->ry, pose->rz);
                
                voxelMarker.pose.orientation.x = quat.x();
                voxelMarker.pose.orientation.y = quat.y();
                voxelMarker.pose.orientation.z = quat.z();
                voxelMarker.pose.orientation.w = quat.w();
                voxelMarker.scale.x = tracklet.w;
                voxelMarker.scale.y = tracklet.l;
                voxelMarker.scale.z = tracklet.h;
                voxelMarker.color.r = (double)rand() / RAND_MAX;
                voxelMarker.color.g = (double)rand() / RAND_MAX;
                voxelMarker.color.b = (double)rand() / RAND_MAX;
                voxelMarker.color.a = 0.5;
                
                markers.markers.push_back(voxelMarker);
            }
        }

    }
    
    return trackletList;
}

