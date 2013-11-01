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

#include "cell.h"

#include <iostream>
#include <boost/foreach.hpp>


using namespace std;

namespace polar_grid_tracking {
    
// FIXME: Use a bigger value after implementing this stage over the GPU
#define DISPARITY_COMPUTATION_ERROR 0.075

Cell::Cell() 
{
    
}

Cell::Cell(const double & x, const double & z, const double & sizeX, const double & sizeZ, 
           const t_Camera_params & params) : 
                m_x(x), m_z(z), m_sizeX(sizeX), m_sizeZ(sizeZ)
{
    if ((z == 0) || (x == 0)) {
        m_sigmaX = 0;
        m_sigmaZ = 0;
    } else {
        const double sigmaZ = (m_z * m_z * DISPARITY_COMPUTATION_ERROR) / (params.baseline * params.ku);
        const double sigmaX = (m_x * sigmaZ) / m_z;
        
        m_sigmaZ = sigmaZ * m_sizeZ;
        m_sigmaX = sigmaX * m_sizeX;
    }
}

void Cell::createParticles(const uint32_t & numParticles)
{
    for (uint32_t i = m_particles.size(); i < numParticles; i++)
        m_particles.push_back(Particle(m_x, m_z, m_sizeX, m_sizeZ));
}
    
void Cell::draw(cv::Mat& img, const uint32_t & pixelsPerCell)
{
    const uint32_t color = m_occupiedProb * 255;
    cv::rectangle(img, cv::Point2i(m_x * pixelsPerCell, m_z * pixelsPerCell), cv::Point2i((m_x + 1) * pixelsPerCell, (m_z + 1) * pixelsPerCell), cv::Scalar::all(color), -1);
    BOOST_FOREACH(Particle particle, m_particles) {
        particle.draw(img, pixelsPerCell, m_sizeX, m_sizeZ);
    }
}

}