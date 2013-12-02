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
#include "polarcell.h"
#include "params_structs.h"


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Dense>

namespace polar_grid_tracking {

typedef Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> BinaryMap;

class PolarGridTracking
{
public:
    
    PolarGridTracking(const uint32_t & rows, const uint32_t & cols, const double & cellSizeX, const double & cellSizeZ, 
                      const double & maxVelX, const double & maxVelZ, const t_Camera_params & cameraParams, 
                      const double & particlesPerCell, const double & threshProbForCreation, 
                      const double & gridDepthFactor, const uint32_t & gridColumnFactor, const double & yawInterval);
    
    void setDeltaYawSpeedAndTime(const double & deltaYaw, const double & deltaSpeed, const double & deltaTime);
    void compute(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr & pointCloud);
    
protected:   
    void reconstructObjects(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr & pointCloud);
    void generateObstacles();
    void extendPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr & pointCloud,
                          pcl::PointCloud< PointXYZRGBDirected >::Ptr & extendedPointCloud);
    void getPolarPositionFromCartesian(const double & z, const double & x, 
                                       uint32_t& row, uint32_t& column);
    void resetPolarGrid();
    void updatePolarGridWithPoint(const PointXYZRGBDirected & point);
    
    void getBinaryMapFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud);
    void initialization();
    void getMeasurementModel();
    void measurementBasedUpdate();
    void prediction();
    void drawGrid(const uint32_t & pixelsPerCell);
    void drawBinaryMap();
    void drawTopDownMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud);
    
    BinaryMap m_map;
    CellGrid m_grid;
    PolarCellGrid m_polarGrid;
    
    bool m_initialized;
    
    double m_deltaYaw, m_deltaSpeed, m_deltaTime;
    
    // Params
    t_Camera_params m_cameraParams;
    double m_cellSizeX, m_cellSizeZ;
    double m_maxVelX, m_maxVelZ;
    double m_particlesPerCell, m_threshProbForCreation;
    
    double m_gridDepthFactor;
    uint32_t m_gridColumnFactor;
    double m_yawInterval;
    
    typedef vector<t_gridCoordinate> t_obstacle;
    vector <t_obstacle> m_obstacles;
    
};
}

#endif // POLARGRIDTRACKING_H
