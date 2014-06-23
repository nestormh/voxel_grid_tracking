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
#define DISPARITY_COMPUTATION_ERROR 0.075 //0.25 //0.075//0.075

Voxel::Voxel() 
{
    
}
Voxel::Voxel(const double & x, const double & y, const double & z, 
             const double & centroidX, const double & centroidY, const double & centroidZ,
             const double & sizeX, const double & sizeY, const double & sizeZ, 
             const double & maxVelX, const double & maxVelY, const double & maxVelZ,
             const polar_grid_tracking::t_Camera_params & params, const SpeedMethod & speedMethod,
             const double & yawInterval, const double & pitchInterval) : 
                    m_x(x), m_y(y), m_z(z), 
                    m_centroidX(centroidX), m_centroidY(centroidY), m_centroidZ(centroidZ), 
                    m_sizeX(sizeX), m_sizeY(sizeY), m_sizeZ(sizeZ), 
                    m_maxVelX(maxVelX), m_maxVelY(maxVelY), m_maxVelZ(maxVelZ),
                    m_speedMethod(speedMethod), m_yawInterval(yawInterval), m_pitchInterval(pitchInterval)
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
    
    m_magnitude = 0.0;
    m_neighborOcc = 0;
    
    m_pointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void Voxel::createParticles(const uint32_t & numParticles, const tf::StampedTransform & pose2mapTransform)
{
    for (uint32_t i = m_particles.size(); i < numParticles; i++)
        m_particles.push_back(Particle3d(m_centroidX, m_centroidY, m_centroidZ, 
                                         m_sizeX, m_sizeY, m_sizeZ, m_maxVelX, m_maxVelY, m_maxVelZ, 
                                         pose2mapTransform));
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

void Voxel::setMainVectors(const double & deltaEgoX, const double & deltaEgoY, const double & deltaEgoZ) {
    
    m_vx = 0.0;
    m_vy = 0.0;
    m_vz = 0.0;
    
    switch (m_speedMethod) {
        case SPEED_METHOD_MEAN: {
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
            
            m_magnitude = sqrt(m_vx * m_vx + m_vy * m_vy + m_vz * m_vz);
            
            const double & normYaw = sqrt(m_vx * m_vx + m_vy * m_vy);
            const double & normPitch = sqrt(m_vy * m_vy + m_vz * m_vz);
            
            m_yaw = acos(m_vx / normYaw);
            if (m_vy < 0)
                m_yaw = -m_yaw;
            
            m_pitch = asin(m_vz / normPitch);
            
            break;
        }
        case SPEED_METHOD_CIRC_HIST: {
            
            typedef boost::multi_array<polar_grid_tracking::t_histogram, 2> CircularHist;
            const uint32_t totalPitchBins = 2 * M_PI / m_pitchInterval;
            const uint32_t totalYawBins = 2 * M_PI / m_yawInterval;
            CircularHist histogram(boost::extents[totalPitchBins][totalYawBins]);
            
            BOOST_FOREACH(Particle3d particle, m_particles) {
                
                    const double & normYaw = sqrt(particle.vx() * particle.vx() + particle.vy() * particle.vy());
                    const double & normPitch = sqrt(particle.vy() * particle.vy() + particle.vz() * particle.vz());
                    
                    double yaw = acos(particle.vx() / normYaw);
                    if (particle.vy() < 0)
                        yaw = 2 * M_PI - yaw;
                    double pitch = asin(particle.vz() / normPitch);
                    
                    uint32_t idxYaw = yaw / m_yawInterval;
                    uint32_t idxPitch = pitch / m_pitchInterval;
                                    
                    histogram[idxPitch][idxYaw].numPoints++;
                    histogram[idxPitch][idxYaw].magnitudeSum += sqrt(particle.vx() * particle.vx() + 
                                                                    particle.vy() * particle.vy() + 
                                                                    particle.vz() * particle.vz());
            }
             
            
            uint32_t maxIdxPitch = 0;
            uint32_t maxIdxYaw = 0;
            uint32_t numVectors = 0;
            for (uint32_t idxPitch = 0; idxPitch < totalPitchBins; idxPitch++) {
                for (uint32_t idxYaw = 0; idxYaw < totalYawBins; idxYaw++) {
                    if (histogram[idxPitch][idxYaw].numPoints > numVectors) {
                        numVectors = histogram[idxPitch][idxYaw].numPoints;
                        maxIdxPitch = idxPitch;
                        maxIdxYaw = idxYaw;
                    }
                }
            }                
            
            m_yaw = maxIdxYaw * m_yawInterval;
            m_pitch = maxIdxPitch * m_pitchInterval;
            m_magnitude = histogram[maxIdxPitch][maxIdxYaw].magnitudeSum / histogram[maxIdxPitch][maxIdxYaw].numPoints;
            
//             if (((double)numVectors / m_particles.size()) < 0.25) {
//                 m_yaw = m_pitch = m_magnitude = 0.0;
//             }
            
            m_vx = m_magnitude * cos(m_yaw) * cos(m_pitch);
            m_vy = m_magnitude * sin(m_yaw) * cos(m_pitch);
            m_vz = m_magnitude * sin(m_pitch);
            
//             m_vx = m_magnitude * cos(m_yaw) * cos(m_pitch) + deltaEgoX;
//             m_vy = m_magnitude * sin(m_yaw) * cos(m_pitch) + deltaEgoY;
//             m_vz = m_magnitude * sin(m_pitch) + deltaEgoZ;
//             
//             // New magnitudes and angles after ego-motion compensation
//             m_magnitude = sqrt(m_vx * m_vx + m_vy * m_vy + m_vz * m_vz);
//             
//             const double & normYaw = sqrt(m_vx * m_vx + m_vy * m_vy);
//             const double & normPitch = sqrt(m_vy * m_vy + m_vz * m_vz);
//             
//             m_yaw = acos(m_vx / normYaw);
//             if (m_vy < 0)
//                 m_yaw = -m_yaw;
//             
//             m_pitch = asin(m_vz / normPitch);
//             
//             if (m_yaw < 0) m_yaw += M_PI;
//             if (m_pitch < 0) m_pitch += M_PI;
            
            break;
        }
        default: {
            ROS_ERROR("Speed method not known: %d", m_speedMethod);
            exit(-1);
        }
    }
}

void Voxel::addPoint(const pcl::PointXYZRGB& point)
{
    m_pointCloud->push_back(point);
}

void Voxel::update()
{
    m_density = (double)m_pointCloud->size()/* / (m_sizeX * m_sizeY * m_sizeZ)*/;
}

void Voxel::reset()
{    
    m_pointCloud->clear();
    m_obstIdx = -1;
    m_neighborOcc = 0;
}

bool Voxel::nextTo(const Voxel& voxel) const
{
    return (abs(m_x - voxel.x()) <= 1) &&
           (abs(m_y - voxel.y()) <= 1) &&
           (abs(m_z - voxel.z()) <= 1);
}

}