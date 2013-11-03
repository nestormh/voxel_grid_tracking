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


#ifndef POLARGRIDTRACKING_H
#define POLARGRIDTRACKING_H

#include "cell.h"
#include "params_structs.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Dense>

namespace polar_grid_tracking {
    
typedef Eigen::Matrix<Cell, Eigen::Dynamic, Eigen::Dynamic> CellGrid;
typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> BinaryMap;

class PolarGridTracking
{
public:
    
    PolarGridTracking(const uint32_t & rows, const uint32_t & cols, const double & cellSizeX, const double & cellSizeZ, 
                      const t_Camera_params & cameraParams, 
                      const double & particlesPerCell, const double & threshProbForCreation);
    
    void setDeltaYawPosAndTime(const double & deltaYaw, const double & deltaPos, const double & deltaTime);
    void compute(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr & pointCloud);
    
protected:   
    
    void getBinaryMapFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud, BinaryMap & map);
    void initialization(const BinaryMap & map);
    void getMeasurementModel(const BinaryMap & map);
    void measurementBasedUpdate();
    void prediction();
    void drawGrid(const uint32_t & pixelsPerCell, const BinaryMap & binaryMap);
    
    CellGrid m_grid;
    
    bool m_initialized;
    
    double m_deltaYaw, m_deltaPos, m_deltaTime;
    
    // Params
    t_Camera_params m_cameraParams;
    double m_cellSizeX, m_cellSizeZ;
    double m_particlesPerCell, m_threshProbForCreation;
    
};
}

#endif // POLARGRIDTRACKING_H
