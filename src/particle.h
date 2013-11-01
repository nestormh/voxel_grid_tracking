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


#ifndef PARTICLE_H
#define PARTICLE_H

#include <opencv2/opencv.hpp>

namespace polar_grid_tracking {
    
class Particle
{
public:
    Particle(const double & cellX, const double & cellZ, const double & cellSizeX, const double & cellSizeZ);
    
    double x() { return m_x; }
    double z() { return m_z; }
    double vx() { return m_vx; }
    double vz() { return m_vz; }
    
    void draw(cv::Mat & img, const uint32_t & pixelsPerCell, const double & cellSizeX, const double & cellSizeZ);
private:
    double m_x, m_z, m_vx, m_vz;
};
    
}
#endif // PARTICLE_H