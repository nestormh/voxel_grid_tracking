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


#ifndef POLARCELL_H
#define POLARCELL_H

#include "params_structs.h"

#include <vector>

using namespace std;

namespace polar_grid_tracking {
    
class PolarCell;
typedef Eigen::Matrix<PolarCell, Eigen::Dynamic, Eigen::Dynamic> PolarCellGrid;

class PolarCell
{
public:
    PolarCell();
    PolarCell(const double & yawInterval, const uint32_t & row, const uint32_t & col);
    
    void addPointToHistogram(const PointXYZRGBDirected & point);
    void reset();
    void updateYawAndMagnitude();
    void setObstacleIdx(const int32_t & idx) { m_obstIdx = idx; }
    
    vector<t_histogram> getHistogram() { return m_histogram; }
    double getMagnitude() const { return m_magnitude; }
    double getYaw() const { return m_yaw; }
    uint32_t getNumVectors() const { return m_numVectors; }
    int32_t obstIdx() const { return m_obstIdx; }
    uint32_t row() const { return m_row; }
    uint32_t col() const { return m_col; }
    double maxHeight() const { return m_maxHeight; }
    
protected:
    
    vector<t_histogram> m_histogram;
    double m_yaw;
    double m_magnitude;
    double m_yawInterval;
    uint32_t m_numVectors;
    int32_t m_obstIdx;
    
    uint32_t m_row;
    uint32_t m_col;
    
    double m_maxHeight;
};
    
}

#endif // POLARCELL_H
