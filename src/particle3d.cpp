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

#include "particle3d.h"

#include <stdlib.h>
#include <math.h>
#include <boost/graph/graph_concepts.hpp>

using namespace std;

namespace voxel_grid_tracking {

    Particle3d::Particle3d(const double & centroidX, const double & centroidY, const double & centroidZ, 
                           const double & voxelSizeX, const double & voxelSizeY, const double & voxelSizeZ, 
                           const double & maxVelX, const double & maxVelY, const double & maxVelZ)
                            : m_maxVelX(maxVelX), m_maxVelY(maxVelY), m_maxVelZ(maxVelZ)
{
    m_x = centroidX + (((double)rand() / RAND_MAX) - 0.5) * voxelSizeX;
    m_y = centroidY + (((double)rand() / RAND_MAX) - 0.5) * voxelSizeY;
    m_z = centroidZ + (((double)rand() / RAND_MAX) - 0.5) * voxelSizeZ;
    
    const double theta = ((double)rand() / RAND_MAX) * 2.0 * M_PI;
    const double gamma = ((double)rand() / RAND_MAX) * 2.0 * M_PI;
    m_vx = maxVelX * ((double)rand() / RAND_MAX) * cos(theta);
    m_vy = maxVelY * ((double)rand() / RAND_MAX) * sin(theta);
    m_vz = maxVelZ * ((double)rand() / RAND_MAX) * sin(gamma);

}

Particle3d::Particle3d(const double& x, const double& y, const double& z, 
                       const double& vx, const double& vy, const double& vz)
                        : m_x(x), m_y(y), m_z(z), m_vx(vx), m_vy(vy), m_vz(vz)
{
}

Particle3d::Particle3d(const Particle3d& particle)
                            : m_x(particle.x()), m_y(particle.y()), m_z(particle.z()), 
                            m_vx(particle.vx()), m_vy(particle.vy()), m_vz(particle.vz())
{
}

void Particle3d::transform(const Eigen::MatrixXd & R, const Eigen::VectorXd & t, const Eigen::MatrixXd& stateTransition)
{
    Eigen::VectorXd newPosAndVel(6), oldPosAndVel(6);
    Eigen::VectorXd tmpPosAndVel(6), deltaPosAndVel(6), finalPosAndVel(6);
    
    oldPosAndVel << m_x, m_y, m_z, m_vx, m_vy, m_vz;
    
    // TODO: Fill with Q covariance, as indicated in the paper
    deltaPosAndVel << 0, 0, 0, 0, 0, 0;
        
    newPosAndVel = stateTransition * oldPosAndVel + deltaPosAndVel;
    
    finalPosAndVel = R * newPosAndVel + t;
    
    m_x = finalPosAndVel(0);
    m_y = finalPosAndVel(1);
    m_z = finalPosAndVel(2);
    m_vx = finalPosAndVel(3);
    m_vy = finalPosAndVel(4);
    m_vz = finalPosAndVel(5);
    
//     m_vx = newPosAndVel(3);
//     m_vy = newPosAndVel(4);
//     m_vz = newPosAndVel(5);
}

ostream& operator<<(ostream & stream, const Particle3d & in) {
    stream << "[" << in.x() << ", " << in.y() << ", " << in.z() << ", " 
           << in.vx() << ", " << in.vy() << ", " << in.vz() << "]";
    return stream;
}

}