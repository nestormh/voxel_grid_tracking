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

#include <boost/foreach.hpp>

#include <limits>

using namespace std;

namespace polar_grid_tracking {

Obstacle::Obstacle(const uint32_t& obstIdx, const double & threshYaw, 
                       const double & threshMagnitude, PolarCell& cell) :
                m_idx(obstIdx), m_threshMagnitude(threshMagnitude), m_threshYaw(threshYaw)
{
        addCellToObstacle(cell);
}

Obstacle::Obstacle(const uint32_t& obstIdx, const double & threshYaw, 
                   const double & threshMagnitude) :
                   m_idx(obstIdx), m_threshMagnitude(threshMagnitude), m_threshYaw(threshYaw)
{

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

    
}