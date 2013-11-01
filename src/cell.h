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


#ifndef CELL_H
#define CELL_H

#include "params_structs.h"
#include "particle.h"

#include <opencv2/opencv.hpp>

#include <vector>

using namespace std;

namespace polar_grid_tracking {
    
class Cell
{
public:
    Cell();
    Cell(const double & x, const double & z, const double & sizeX, const double & sizeZ, 
         const t_Camera_params & params);
    
    void createParticles(const uint32_t & numParticles);
    
    void setOccupiedProb(const double & occupiedProb) { m_occupiedProb = occupiedProb; }
    
    double sigmaX() { return m_sigmaX; } 
    double sigmaZ() { return m_sigmaZ; }
    
    double occupiedProb() { return m_occupiedProb; }
    double freeProb() { return 1.0 - m_occupiedProb; }
    
    uint32_t numParticles() { return m_particles.size(); }
    bool empty() { return m_particles.size() == 0; }
    
    void draw(cv::Mat & img, const uint32_t & pixelsPerCell);
    
private:
    double m_x, m_z;
    double m_sigmaX, m_sigmaZ;
    double m_sizeX, m_sizeZ;
    
    double m_occupiedProb;
    
    vector <Particle> m_particles;
};

}
#endif // CELL_H