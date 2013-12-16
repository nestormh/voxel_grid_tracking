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

#include "voxel.h"

#include <iostream>
#include <boost/foreach.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/impl/centroid.hpp>


using namespace std;

namespace voxel_grid_tracking {
    
// FIXME: Use a bigger value after implementing this stage over the GPU
#define DISPARITY_COMPUTATION_ERROR 0.75//0.075

Voxel::Voxel() 
{
    
}
Voxel::Voxel(const double & x, const double & y, const double & z, 
             const double & centroidX, const double & centroidY, const double & centroidZ,
             const double & sizeX, const double & sizeY, const double & sizeZ, 
             const double & maxVelX, const double & maxVelY, const double & maxVelZ,
             const polar_grid_tracking::t_Camera_params & params) : 
                    m_x(x), m_y(y), m_z(z), 
                    m_centroidX(centroidX), m_centroidY(centroidY), m_centroidZ(centroidZ), 
                    m_sizeX(sizeX), m_sizeY(sizeY), m_sizeZ(sizeZ), 
                    m_maxVelX(maxVelX), m_maxVelY(maxVelY), m_maxVelZ(maxVelZ)
{
    if ((x == 0) || (y == 0) || (z == 0)) {
        m_sigmaX = 0.0;
        m_sigmaY = 0.0;
        m_sigmaZ = 0.0;
    } else {
        double xReal = m_centroidX * m_sizeX;
        double yReal = m_y * m_sizeY;
        double zReal = m_z * m_sizeZ;
        
        const double sigmaY = (m_centroidY * yReal * DISPARITY_COMPUTATION_ERROR) / (params.baseline * params.ku);
        const double sigmaX = (m_centroidX * sigmaY) / m_centroidY;
        const double sigmaZ = (m_centroidZ * sigmaY) / m_centroidY;
        
        m_sigmaX = sigmaX / m_sizeX;
        m_sigmaY = sigmaY / m_sizeY;
        m_sigmaZ = sigmaZ / m_sizeZ;
    }
    
    m_pointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void Voxel::createParticles(const uint32_t & numParticles)
{
    for (uint32_t i = m_particles.size(); i < numParticles; i++)
        m_particles.push_back(Particle3d(m_centroidX, m_centroidY, m_centroidZ, 
                                         m_sizeX, m_sizeY, m_sizeZ, m_maxVelX, m_maxVelY, m_maxVelZ));
}

void Voxel::setOccupiedPosteriorProb(const uint32_t& particlesPerVoxel)
{
    if (m_occupiedProb == 0) {
        m_occupiedPosteriorProb = 0;
    } else {
        const double weightedOccupied = m_occupiedProb * m_particles.size();
        const double freeProb = 1.0 - m_occupiedProb;
        const uint32_t numFree = particlesPerVoxel - m_particles.size();
        const double weightedFree = freeProb * numFree;
        
        m_occupiedPosteriorProb = weightedOccupied / (weightedOccupied + weightedFree);
    }    
}

void Voxel::makeCopy(const Particle3d& particle)
{
    Particle3d newParticle(particle);
    m_particles.push_back(newParticle);
}

void Voxel::addParticle(const Particle3d& particle)
{
    m_particles.push_back(particle);
}
    
void Voxel::transformParticles(const Eigen::MatrixXd & R, const Eigen::VectorXd & t, 
                               const Eigen::MatrixXd & stateTransition, vector <Particle3d> & newParticles)
{
    BOOST_FOREACH(Particle3d & particle, m_particles) {
        particle.transform(R, t, stateTransition);
        
        newParticles.push_back(particle);
        
    }
    m_particles.clear();
}

void Voxel::setMainVectors() {
    m_vx = 0.0;
    m_vy = 0.0;
    m_vz = 0.0;
    if (m_particles.size() != 0) {
        BOOST_FOREACH(Particle3d particle, m_particles) {
            m_vx += particle.vx();
            m_vy += particle.vy();
            m_vz += particle.vz();
        }
    
        m_vx /= m_particles.size();
        m_vy /= m_particles.size();
        m_vz /= m_particles.size();
    }
}

void Voxel::addPoint(const pcl::PointXYZRGB& point)
{
    m_pointCloud->push_back(point);
}

void Voxel::update()
{

}

void Voxel::reset()
{
    m_pointCloud->clear();
}

}