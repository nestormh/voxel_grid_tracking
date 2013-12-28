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

#include "utilspolargridtracking.h"

double calculateDifferenceBetweenAngles(const double & ang1, const double & ang2)
{
    const double vx1 = cos(ang1);
    const double vy1 = sin(ang1);
    
    const double vx2 = cos(ang2);
    const double vy2 = sin(ang2);
    
    const Eigen::Vector3f vGlobal(cos(ang1), sin(ang1), 0.0f);
    const Eigen::Vector3f vCurr(cos(ang2), sin(ang2), 0.0f);
    
    double theta = fabs(acos(vCurr.dot(vGlobal)));
}

cv::Mat getCvMatFromEigenBinary(const polar_grid_tracking::BinaryMap & map) {
    
    cv::Mat img(cv::Size(map.cols(), map.rows()), CV_8UC1);
    
    for (uint32_t i = 0; i < map.rows(); i++) {
        for (uint32_t j = 0; j < map.cols(); j++) {
            img.at<uint8_t>(i, j) = map(i, j)? 255 : 0;
        }
    }
    
    return img;
}

cv::Mat getCvMatFromProbabilityMap(/*const*/ polar_grid_tracking::CellGrid & map) {
    cv::Mat img(cv::Size(map.cols(), map.rows()), CV_8UC1);
    
    for (uint32_t i = 0; i < map.rows(); i++) {
        for (uint32_t j = 0; j < map.cols(); j++) {
            img.at<uint8_t>(i, j) = map(i, j).occupiedProb() * 255;
        }
    }
    
    return img;
}

void project3dTo2d(const pcl::PointXYZRGB& point3d, pcl::PointXYZRGB& point2d, 
                   const polar_grid_tracking::t_Camera_params & cameraParams)
{
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> pointMat(3, 1);
    
    pointMat << cameraParams.t(0) + point3d.x, cameraParams.t(1) + point3d.y, cameraParams.t(2) + point3d.z;
    
    pointMat = cameraParams.R.inverse() * pointMat;
    
    const double d = cameraParams.ku * cameraParams.baseline / pointMat(2);
    const double u = cameraParams.u0 - ((pointMat(0) * d) / cameraParams.baseline);
    const double v = cameraParams.v0 - ((pointMat(1) * cameraParams.kv * d) / (cameraParams.ku * cameraParams.baseline));
    
    point2d.x = u;
    point2d.y = v;
    point2d.z = d;
    point2d.r = point3d.r;
    point2d.g = point3d.g;
    point2d.b = point3d.b;
}