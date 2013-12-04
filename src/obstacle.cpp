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

#include "obstacle.h"
#include "utils.h"

#include <pcl-1.7/pcl/common/transforms.h>

#include <boost/foreach.hpp>

#include <limits>

using namespace std;

namespace polar_grid_tracking {

Obstacle::Obstacle(const uint32_t& obstIdx, const double & threshYaw, 
                       const double & threshMagnitude, PolarCell& cell) :
                m_idx(obstIdx), m_threshMagnitude(threshMagnitude), m_threshYaw(threshYaw)
{
        addCellToObstacle(cell);
        m_roi.reset(new pcl::PointCloud<pcl::PointXYZ>);
        m_roi->points.resize(8);
}

Obstacle::Obstacle(const uint32_t& obstIdx, const double & threshYaw, 
                   const double & threshMagnitude) :
                   m_idx(obstIdx), m_threshMagnitude(threshMagnitude), m_threshYaw(threshYaw)
{
    m_roi.reset(new pcl::PointCloud<pcl::PointXYZ>);
    m_roi->points.resize(8);
}

bool Obstacle::addCellToObstacle(PolarCell& cell)
{
    if (m_cells.size() != 0) {    // If holds motion info
        const double & diffMagnitude = fabs(m_magnitude - cell.getMagnitude());
        const double & diffAng = calculateDifferenceBetweenAngles(m_yaw, cell.getYaw());
        
        if ((diffMagnitude < m_threshMagnitude) && (diffAng < m_threshYaw)) {
            m_cells.push_back(cell);
            cell.setObstacleIdx(m_idx);
            updateMotionInformation();
            
            return true;
        }
    } else {
        m_cells.push_back(cell);
        m_magnitude = cell.getMagnitude();
        m_yaw = cell.getYaw();
        cell.setObstacleIdx(m_idx);
        
        return true;
    }
    
    return false;
}

void Obstacle::updateMotionInformation()
{
    double vx = 0.0, vy = 0.0;
    m_magnitude = 0.0;
    BOOST_FOREACH(const PolarCell & cell, m_cells) {
        vx += cos(cell.getYaw());
        vy += sin(cell.getYaw());
        m_magnitude += cell.getMagnitude();
    }
    
    vx /= m_cells.size();
    vy /= m_cells.size();
    m_magnitude /= m_cells.size();

    m_yaw = fabs(acos(vx));
    if (vy < 0.0)
        m_yaw = 2 * M_PI - m_yaw;
}

void Obstacle::setROIAndMotion(const t_Camera_params & cameraParams, 
                                const double & gridDepthFactor, const double & gridColumnFactor,
                                const double & yawInterval)
{
    uint32_t minR = std::numeric_limits< int >::max(), maxR = 0;
    uint32_t minC = std::numeric_limits< int >::max(), maxC = 0;
    double maxH = std::numeric_limits< double >::min();
    const double minH = 0.0;
    
    BOOST_FOREACH(const PolarCell & cell, m_cells) {
        if (cell.row() > maxR) maxR = cell.row();
        if (cell.row() < minR) minR = cell.row();
        if (cell.col() > maxC) maxC = cell.col();
        if (cell.col() < minC) minC = cell.col();
        if (cell.maxHeight() > maxH) maxH = cell.maxHeight();
    }

    const double z0 = (double)(cameraParams.ku * cameraParams.baseline) / cameraParams.width;
    
    const double minY = z0 * pow(1.0 + gridDepthFactor, minR - 0.5);
    const double maxY = z0 * pow(1.0 + gridDepthFactor, maxR + 0.5);
    
    const double minU = gridColumnFactor * (minC + 2.0);
    const double minXminY = minY * (cameraParams.u0 - minU) / cameraParams.ku;
    const double minXmaxY = maxY * (cameraParams.u0 - minU) / cameraParams.ku;
    const double maxU = gridColumnFactor * (maxC + 3.0);
    const double maxXminY = minY * (cameraParams.u0 - maxU) / cameraParams.ku;
    const double maxXmaxY = maxY * (cameraParams.u0 - maxU) / cameraParams.ku;
    
    
    m_roi->at(0).x = minXminY;
    m_roi->at(0).y = minY;
    m_roi->at(0).z = minH;
            
    m_roi->at(1).x = maxXminY;
    m_roi->at(1).y = minY;
    m_roi->at(1).z = minH;
        
    m_roi->at(2).x = maxXminY;
    m_roi->at(2).y = minY;
    m_roi->at(2).z = maxH;
    
    m_roi->at(3).x = minXminY;
    m_roi->at(3).y = minY;
    m_roi->at(3).z = maxH;

    m_roi->at(4).x = maxXmaxY;
    m_roi->at(4).y = maxY;
    m_roi->at(4).z = maxH;
    
    m_roi->at(5).x = minXmaxY;
    m_roi->at(5).y = maxY;
    m_roi->at(5).z = maxH;
    
    m_roi->at(6).x = minXmaxY;
    m_roi->at(6).y = maxY;
    m_roi->at(6).z = minH;
    
    m_roi->at(7).x = maxXmaxY;
    m_roi->at(7).y = maxY;
    m_roi->at(7).z = minH;
    
}


    
}