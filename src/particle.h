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
#include <iostream>
#include <Eigen/Core>

using namespace std;

namespace polar_grid_tracking {
    
class Particle
{
public:
    Particle(const double & cellX, const double & cellZ, const double & cellSizeX, const double & cellSizeZ, 
                       const double & maxVelX, const double & maxVelZ);
    Particle(const double & x, const double & z, const double & vx, const double & vz);
    
    Particle(const Particle & particle);
    
    void transform(const Eigen::Matrix4d & R, const Eigen::Vector4d & t, const Eigen::Matrix4d & stateTransition);
    
    double x() const { return m_x; }
    double z() const { return m_z; }
    double vx() const { return m_vx; }
    double vz() const { return m_vz; }
    
    void draw(cv::Mat & img, const uint32_t & pixelsPerCell, const double & cellSizeX, const double & cellSizeZ);
private:
    double m_x, m_z, m_vx, m_vz;
    
    double m_maxVelX, m_maxVelZ;
};

ostream& operator<<(ostream & stream, const Particle & in);
    
}
#endif // PARTICLE_H