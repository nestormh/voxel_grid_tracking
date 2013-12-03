
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

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "params_structs.h"
#include "polarcell.h"

#include <vector>

using namespace std;

namespace polar_grid_tracking {
    
class Obstacle
{
public:
    Obstacle(const uint32_t & obstIdx, const double & threshYaw, const double & threshMagnitude, PolarCell & cell);
    Obstacle(const uint32_t & obstIdx, const double & threshYaw, const double & threshMagnitude);
    bool addCellToObstacle(PolarCell & cell);
    
    vector<PolarCell> cells() const { return m_cells; }
protected:
    void updateMotionInformation();
    
    uint32_t m_idx;
    double m_magnitude;
    double m_yaw;
    vector<PolarCell> m_cells;
    
    // Params
    double m_threshYaw, m_threshMagnitude;
};


}

#endif // OBSTACLE_H
