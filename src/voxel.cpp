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
#include <pcl/common/impl/centroid.hpp>


using namespace std;

namespace voxel_grid_tracking {
    
// FIXME: Use a bigger value after implementing this stage over the GPU
#define DISPARITY_COMPUTATION_ERROR 0.075 //0.25 //0.075//0.075

Voxel::Voxel() 
{
    m_occupied = false;
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
    m_oldestParticle = 0;
    
    m_occupied = true;
}

void Voxel::createParticles(const uint32_t & numParticles, const tf::StampedTransform & pose2mapTransform)
{
    for (uint32_t i = m_particles.size(); i < numParticles; i++)
        m_particles.push_back(Particle3d(m_centroidX, m_centroidY, m_centroidZ, 
                                         m_sizeX, m_sizeY, m_sizeZ, m_maxVelX, m_maxVelY, m_maxVelZ, 
                                         pose2mapTransform));
}

void Voxel::createParticlesStatic(const tf::StampedTransform& pose2mapTransform)
{
    for (int32_t vx = -1; vx <= 1; vx++) {
        for (int32_t vy = -1; vy <= 1; vy++) {
            int32_t vz = 0;
//             for (int32_t vz = -1; vz <= 1; vz++) {
                for (double factorSpeed = 0.1; factorSpeed <= 1.0; factorSpeed += 0.1) {
                    Particle3d particle(m_centroidX, m_centroidY, m_centroidZ, 
                               vx * m_maxVelX * factorSpeed, 
                               vy * m_maxVelY * factorSpeed, 
                               vz * m_maxVelZ * factorSpeed,
                               pose2mapTransform, true);
                    
                    m_particles.push_back(particle);
                }
            }
//         }
        
    }
}


void Voxel::createParticlesFromOFlow(const uint32_t & numParticles)
{
    for (uint32_t i = m_particles.size(); i < numParticles; i++) {
        const Particle3d & particle = m_oFlowParticles[rand() % m_oFlowParticles.size()];
        
        m_particles.push_back(Particle3d(particle));
    }
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

void Voxel::addFlowParticle(const Particle3d& particle)
{
    m_oFlowParticles.push_back(particle);
}
    
void Voxel::transformParticles(const Eigen::MatrixXd & stateTransition, vector <Particle3d> & newParticles)
{
    BOOST_FOREACH(Particle3d & particle, m_particles) {
        particle.transform(stateTransition);
        
        newParticles.push_back(particle);
    }
    m_particles.clear();
}

void Voxel::sortParticles()
{
    std::sort(m_particles.rbegin(), m_particles.rend());
    
    m_oldestParticle = m_particles[0].age();
}

void Voxel::joinParticles()
{
    if (m_oFlowParticles.size() != 0) {
        m_particles.insert( m_particles.begin(), m_oFlowParticles.begin(), m_oFlowParticles.end() );
//         m_particles = m_oFlowParticles;
    }
}

void Voxel::reduceParticles()
{
    vector<Particle3d>::const_iterator first = m_particles.begin();
    vector<Particle3d>::const_iterator last = m_particles.begin() + std::min(30, (int)m_particles.size());
    m_particles = vector<Particle3d> (first, last);
}


void Voxel::setMainVectors(const double & deltaEgoX, const double & deltaEgoY, const double & deltaEgoZ) {
    
    m_vx = 0.0;
    m_vy = 0.0;
    m_vz = 0.0;
    
    if (m_oldestParticle == 0) {
        return;
    }
    
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
            
            cout << "totalPitchBins " << totalPitchBins << endl;
            cout << "totalYawBins " << totalYawBins << endl;
            
            BOOST_FOREACH(Particle3d particle, m_particles) {
//             for (uint32_t i = 0; i < m_particles.size() / 2.0; i++) {
//                 const Particle3d & particle = m_particles[i];
                
//                 if (particle.age() != 1) {
                
                    double yaw, pitch;
                    particle.getYawPitch(yaw, pitch);
                    
                    uint32_t idxYaw = yaw / m_yawInterval;
                    uint32_t idxPitch = pitch / m_pitchInterval;
                                    
                    histogram[idxPitch][idxYaw].numPoints++;
                    histogram[idxPitch][idxYaw].magnitudeSum += cv::norm(cv::Vec3f(particle.vx(), particle.vy(), particle.vz()));
//                 }
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
                        
                        if (numVectors != 0) {
                            cout << idxYaw * m_yawInterval * 180 / CV_PI << " = " << numVectors << endl;
                        }
                    }
                }
            }                
            
            m_yaw = maxIdxYaw * m_yawInterval;
            m_pitch = maxIdxPitch * m_pitchInterval;
            m_magnitude = histogram[maxIdxPitch][maxIdxYaw].magnitudeSum / histogram[maxIdxPitch][maxIdxYaw].numPoints;
            
            m_vx = cos(m_yaw) * m_magnitude;
            m_vy = sin(m_yaw) * m_magnitude;
            m_vz = sin(m_pitch) * m_magnitude;
            
            cout << "m_yaw " << m_yaw << endl;
            cout << "m_pitch " << m_pitch << endl;
            cout << "m_magnitude " << m_magnitude << endl;
            cout << "m_vx " << m_vx << endl;
            cout << "m_vy " << m_vy << endl;
            cout << "m_vz " << m_vz << endl;
            
            cout << "==============================" << endl;
            break;
        }
        default: {
            ROS_ERROR("Speed method not known: %d", m_speedMethod);
            exit(-1);
        }
    }
}

void Voxel::reset()
{    
    m_obstIdx = -1;
    m_neighborOcc = 0;
    m_oFlowParticles.clear();
    m_occupied = false;
}

bool Voxel::nextTo(const Voxel& voxel) const
{
    return (abs(m_x - voxel.x()) <= 1) &&
           (abs(m_y - voxel.y()) <= 1) &&
           (abs(m_z - voxel.z()) <= 1);
}

ostream& operator<<(ostream & stream, const Voxel & in) {
    stream << "Voxel = [" << in.x() << ", " << in.y() << ", " << in.z() << ", " 
    << in.vx() << ", " << in.vy() << ", " << in.vz() << "]";
    return stream;
}

}