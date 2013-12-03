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

#include "polarcell.h"

#include <boost/foreach.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <limits>

using namespace std;

namespace polar_grid_tracking {

PolarCell::PolarCell()
{
    m_yaw = std::numeric_limits< double >::infinity();
    m_magnitude = 0.0;
    m_numVectors = 0;
    m_obstIdx = -1;
}

PolarCell::PolarCell(const double& yawInterval, const uint32_t & row, const uint32_t & col) : 
                        m_yawInterval(yawInterval), m_row(row), m_col(col)
{
    m_histogram.resize(2 * M_PI / m_yawInterval);
    
    m_yaw = std::numeric_limits< double >::infinity();
    m_magnitude = 0.0;
    m_numVectors = 0;
    m_obstIdx = -1;
    
    reset();
}

void PolarCell::reset()
{
    BOOST_FOREACH(t_histogram & item, m_histogram) {
        item.magnitudeSum = 0.0;
        item.numPoints = 0;
    }
    m_obstIdx = -1;
}

void PolarCell::addPointToHistogram(const PointXYZRGBDirected& point)
{
    uint32_t idx = point.yaw / m_yawInterval;
    if (point.yaw < 0)
        idx = ((2.0 * M_PI + point.yaw) / m_yawInterval);
    t_histogram & item = m_histogram[idx];
    item.numPoints++;
    item.magnitudeSum += point.magnitude;
}

void PolarCell::updateYawAndMagnitude()
{
    uint32_t maxIdx = 0;
    m_numVectors = 0;
    
    for (uint32_t i = 0; i < m_histogram.size(); i++) {
        if (m_histogram[i].numPoints > m_numVectors) {
            m_numVectors = m_histogram[i].numPoints;
            maxIdx = i;
        }
    }
    
    m_yaw = maxIdx * m_yawInterval;
    m_magnitude = m_histogram[maxIdx].magnitudeSum / m_histogram[maxIdx].numPoints;
}

}

