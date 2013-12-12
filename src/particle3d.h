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


#ifndef PARTICLE_3D_H
#define PARTICLE_3D_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <Eigen/Core>

using namespace std;

namespace voxel_grid_tracking {
    
class Particle3d
{
public:
    Particle3d(const double & cellX, const double & cellY, const double & cellZ, 
               const double & cellSizeX, const double & cellSizeY, const double & cellSizeZ, 
               const double & maxVelX, const double & maxVelY, const double & maxVelZ);
    Particle3d(const double & x, const double & y, const double & z, 
               const double & vx, const double & vy, const double & vz);
    
    Particle3d(const Particle3d & particle);
    
    void transform(const Eigen::Matrix4d & R, const Eigen::Vector4d & t, 
                   const Eigen::MatrixXd & stateTransition);
    
    double x() const { return m_x; }
    double y() const { return m_y; }
    double z() const { return m_z; }
    double vx() const { return m_vx; }
    double vy() const { return m_vy; }
    double vz() const { return m_vz; }

private:
    double m_x, m_y, m_z, m_vx, m_vy, m_vz;
    
    double m_maxVelX, m_maxVelY, m_maxVelZ;
};

ostream& operator<<(ostream & stream, const Particle3d & in);
    
}
#endif // PARTICLE_H