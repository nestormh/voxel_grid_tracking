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


#ifndef OBSTACLESFROMSTEREO_H
#define OBSTACLESFROMSTEREO_H

#include <opencv2/opencv.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

class ObstaclesFromStereo {
public:
    typedef struct {        
        uint32_t minX, minY;
        uint32_t width, height;
        double u0, v0;
        double ku, kv;
        double distortion;
        double baseline;
        Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> R;
        Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> t;
    } t_Camera_params;
    
    typedef struct {        
        int minDisparity;
        int numDisparities;
        int SADWindowSize; 
        int P1; 
        int P2; 
        int disp12MaxDiff; 
        int preFilterCap; 
        int uniquenessRatio;
        
        int speckleWindowSize; 
        int speckleRange; 
        bool fullDP;
    } t_SGBM_params;
    
    typedef enum t_Method { SGBM = 0, STEREOVAR = 1};
    
    ObstaclesFromStereo(const cv::Size & size);
    ~ObstaclesFromStereo();
    
    void generatePointClouds(const cv::Mat & leftImg, const cv::Mat & rightImg, const cv::Mat & mask);
    
    void setCameraParams(const t_Camera_params & leftCameraParams, const t_Camera_params & rightCameraParams);
    void setSGBMParams(const t_SGBM_params & params) { m_SGBM_params = params; }
    void setMethod(const t_Method & method) { m_method = method; };
    void setGroundThresh(const double & groundThresh) { m_groundThresh = groundThresh; }
    void setBackGroundThresh(const double & backgroundThresh) { m_backgroundThresh = backgroundThresh; }
    void setLeafSize(const double & leafSize) { m_leafSize = leafSize; }
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud() { return m_pointCloud; }
    
    static void readi3DPostCalibrationFile(const std::string & fileName, std::vector<t_Camera_params> & cameraParams);
    static void getParamsFromDublinDataset(const std::string & planeFilename, std::vector<t_Camera_params> & params,
                                           double baseline = 0.0995964, double focalLength = 519.546875, cv::Size sz = cv::Size(640, 480));
    static void getFGMask(const std::string & fileName, cv::Mat & fgMask, const cv::Size & sz);
    
private:
    void setParamsGeometry(t_Camera_params & params);
    void filterMasked(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr & inputCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & outputCloud);
    void removeGround(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr & pointCloud);
    void downsample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud);
    
    t_Camera_params m_leftCameraParams;
    t_Camera_params m_rightCameraParams;
    
    t_SGBM_params m_SGBM_params;
    t_Method m_method;
    
    double m_groundThresh;
    double m_backgroundThresh;
    double m_leafSize;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloud;
    
    cv::Size m_size;
};

#endif // OBSTACLESFROMSTEREO_H
