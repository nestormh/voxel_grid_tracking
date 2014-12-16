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
#include <boost/graph/graph_concepts.hpp>
#include <pcl/common/impl/centroid.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


using namespace std;

namespace voxel_grid_tracking {
    
// FIXME: Use a bigger value after implementing this stage over the GPU
#define DISPARITY_COMPUTATION_ERROR 0.075 //0.25 //0.075//0.075

Voxel::Voxel() 
{
    m_occupied = false;
    m_obstIdx = -1;
}

Voxel::Voxel(const double & x, const double & y, const double & z, 
             const double & centroidX, const double & centroidY, const double & centroidZ,
             const double & sizeX, const double & sizeY, const double & sizeZ, 
             const double & maxVelX, const double & maxVelY, const double & maxVelZ,
             const polar_grid_tracking::t_Camera_params & params, const SpeedMethod & speedMethod,
             const double & yawInterval, const double & pitchInterval, const float & factorSpeed) : 
                    m_x(x), m_y(y), m_z(z), 
                    m_centroidX(centroidX), m_centroidY(centroidY), m_centroidZ(centroidZ), 
                    m_sizeX(sizeX), m_sizeY(sizeY), m_sizeZ(sizeZ), 
                    m_maxVelX(maxVelX), m_maxVelY(maxVelY), m_maxVelZ(maxVelZ),
                    m_speedMethod(speedMethod), m_yawInterval(yawInterval), m_pitchInterval(pitchInterval),
                    m_factorSpeed(factorSpeed)
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
    
    m_obstIdx = -1;
    
    m_speedHistogram.resize(boost::extents[3][3][3][((int)ceil(1.0 / m_factorSpeed)) + 1]);
}

void Voxel::createParticles(const uint32_t & numParticles, const tf::StampedTransform & pose2mapTransform)
{
    for (uint32_t i = m_particles.size(); i < numParticles; i++) {
        ParticlePtr particle(new Particle3d(m_centroidX, m_centroidY, m_centroidZ, 
                   m_sizeX, m_sizeY, m_sizeZ, m_maxVelX, m_maxVelY, m_maxVelZ, 
                   pose2mapTransform));
        m_particles.push_back(particle);
    }
}

ParticleList Voxel::createParticlesStatic(const tf::StampedTransform& pose2mapTransform)
{
    ParticleList particleList;
    for (int32_t vx = -1; vx <= 1; vx++) {
        for (int32_t vy = -1; vy <= 1; vy++) {
            int32_t vz = 0;
            
            if (vx == vy == vz == 0)
                continue;
//             for (int32_t vz = -1; vz <= 1; vz++) {
            for (double factorSpeed = m_factorSpeed; factorSpeed <= 1.0; factorSpeed += m_factorSpeed) {
                    ParticlePtr particle(new Particle3d(m_centroidX, m_centroidY, m_centroidZ, 
                                            vx * m_maxVelX * factorSpeed, 
                                            vy * m_maxVelY * factorSpeed, 
                                            vz * m_maxVelZ * factorSpeed,
                                            pose2mapTransform, false));
                    
                    particleList.push_back(particle);
                }
            }
//         }
        
    }
//     ParticlePtr particle(new Particle3d(m_centroidX, m_centroidY, m_centroidZ, 
//                                         0.0, 0.0, 0.0,
//                                         pose2mapTransform, false));
//     
//     particleList.push_back(particle);
    
    return particleList;
}


ParticleList Voxel::createParticlesFromOFlow(const uint32_t & numParticles)
{
    ParticleList particleList;
    for (uint32_t i = m_particles.size(); i < numParticles; i++) {
        ParticlePtr particle = m_oFlowParticles[rand() % m_oFlowParticles.size()];
        
        particleList.push_back(particle);
    }
    
    // Append particles to the local list
    m_particles.reserve(m_particles.size() + particleList.size());
    m_particles.insert(m_particles.end(), particleList.begin(), particleList.end());
    
    return particleList;
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

void Voxel::makeCopy(const ParticlePtr& particle)
{
    ParticlePtr newParticle(new Particle3d(*particle));
    m_particles.push_back(newParticle);
}

void Voxel::addParticle(const ParticlePtr& particle)
{
    m_particles.push_back(particle);
}

void Voxel::addFlowParticle(const ParticlePtr& particle)
{
    m_oFlowParticles.push_back(particle);
}
    
// TODO: I am not sorting particles anymore. Ensure that particles are inserted in the order they were inserted!!!
void Voxel::sortParticles()
{
//     std::sort(m_particles.rbegin(), m_particles.rend());
    m_oldestParticle = m_particles[0]->age();
}

void Voxel::joinParticles()
{
    if (m_oFlowParticles.size() != 0) {
        m_particles.reserve(m_particles.size() + m_oFlowParticles.size());
        m_particles.insert( m_particles.begin(), m_oFlowParticles.begin(), m_oFlowParticles.end() );
    }
}

void Voxel::reduceParticles(const uint32_t & maxNumberOfParticles)
{
    ParticleList::const_iterator first = m_particles.begin();
    ParticleList::const_iterator last = m_particles.begin() + std::min(maxNumberOfParticles, (uint32_t)m_particles.size());
    m_particles = ParticleList (first, last);
}

void Voxel::centerParticles()
{
    BOOST_FOREACH(ParticlePtr & particle, m_particles) {
        particle->updatePosition(m_centroidX, m_centroidY, m_centroidZ);
    }
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
                BOOST_FOREACH(ParticlePtr particle, m_particles) {
                    m_vx += particle->vx();
                    m_vy += particle->vy();
                    m_vz += particle->vz();
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
            
//             if ((m_x == 1) && (m_y == 3) && (m_z == 2)) {
//                 cout << "Analyzing (1, 3, 2)" << endl;
//                 cout << "===================" << endl;
            
//                 BOOST_FOREACH(ParticlePtr particle, m_particles) {
//                     cout << *particle << endl;
//                 }
//                 cout << "-------------------" << endl;
                
                SpeedHistogram histogram(boost::extents[3][3][3][((int)ceil(1.0 / m_factorSpeed)) + 1]);
                
                uint32_t totalPoints = 0;
                
                float maxSpeed = cv::norm(cv::Vec3f(m_maxVelX, m_maxVelY, m_maxVelZ));
                BOOST_FOREACH(ParticlePtr particle, m_particles) {
                    if (particle->age() > 1) {
                        const float & vx = m_centroidX - particle->xOld();
                        const float & vy = m_centroidY - particle->yOld();
                        const float & vz = m_centroidZ - particle->zOld();
                        
                        cv::Vec3f speedVector(vx, vy, vz);
                        float speed = cv::norm(speedVector);
                        if (speed != 0.0f)
                            speedVector /= speed;
                        
                        uint32_t idX = round(speedVector[0] + 1.0f);
                        uint32_t idY = round(speedVector[1] + 1.0f);
                        uint32_t idZ = round(speedVector[2] + 1.0f);
                        uint32_t idSpeed = round(speed / maxSpeed / m_factorSpeed);
                        cout << "v: " << speedVector << ", speed: " << speed << ", maxSpeed " << maxSpeed <<
                                " => " << cv::Vec3f(idX, idY, idZ) << ", idSpeed: " << idSpeed << endl;
                        
                        histogram[idX][idY][idZ][idSpeed].numPoints += particle->age();
                        totalPoints += particle->age();
                    }
                }
                
                cout << "-------------------" << endl;

                for (int32_t x = 0; x < 3; x++) {
                    for (int32_t y = 0; y < 3; y++) {
                        for (int32_t z = 0; z < 3; z++) {
                            for (int32_t s = 0; s < ceil(1.0 / m_factorSpeed) + 1; s++) {
                                if (histogram[x][y][z][s].numPoints != 0) {
                                    cout << "(" << x - 1 << ", " << y - 1 << ", " 
                                            << s * m_factorSpeed * maxSpeed << ") => " 
                                            << histogram[x][y][z][s].numPoints / (float)totalPoints << endl;
                                }
                            }
                        }
                    }
                }
//             }
            
            /*typedef boost::multi_array<polar_grid_tracking::t_histogram, 2> CircularHist;
            const uint32_t totalPitchBins = 2 * M_PI / m_pitchInterval;
            const uint32_t totalYawBins = 2 * M_PI / m_yawInterval;
            CircularHist histogram(boost::extents[totalPitchBins][totalYawBins]);
            
//             cout << "totalPitchBins " << totalPitchBins << endl;
//             cout << "totalYawBins " << totalYawBins << endl;
            
            BOOST_FOREACH(ParticlePtr particle, m_particles) {
//             for (uint32_t i = 0; i < m_particles.size() / 2.0; i++) {
//                 const Particle3d & particle = m_particles[i];
                
//                 if (particle.age() != 1) {
                
                    double yaw, pitch;
                    particle->getYawPitch(yaw, pitch);
                    
                    uint32_t idxYaw = yaw / m_yawInterval;
                    uint32_t idxPitch = pitch / m_pitchInterval;
                                    
                    histogram[idxPitch][idxYaw].numPoints++;
                    histogram[idxPitch][idxYaw].magnitudeSum += cv::norm(cv::Vec3f(particle->vx(), particle->vy(), particle->vz()));
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
                    }
                }
            }                
            
            m_yaw = maxIdxYaw * m_yawInterval;
            m_pitch = maxIdxPitch * m_pitchInterval;
            m_magnitude = histogram[maxIdxPitch][maxIdxYaw].magnitudeSum / histogram[maxIdxPitch][maxIdxYaw].numPoints;
            
            m_vx = cos(m_yaw) * m_magnitude;
            m_vy = sin(m_yaw) * m_magnitude;
            m_vz = sin(m_pitch) * m_magnitude;
            
//             cout << "m_yaw " << m_yaw << endl;
//             cout << "m_pitch " << m_pitch << endl;
//             cout << "m_magnitude " << m_magnitude << endl;
//             cout << "m_vx " << m_vx << endl;
//             cout << "m_vy " << m_vy << endl;
//             cout << "m_vz " << m_vz << endl;
//             
//             cout << "==============================" << endl;
            
            */
            break;
        }
        default: {
            ROS_ERROR("Speed method not known: %d", m_speedMethod);
            exit(-1);
        }
    }
}

void Voxel::updateHistogram()
{
//     cout << "-----------------------------------------" << endl;
//     cout << "Analyzing " << cv::Vec3f(m_x, m_y, m_z) << endl;
    
    uint32_t totalPoints = 0;
    const float & maxSpeed = cv::norm(cv::Vec3f(m_maxVelX, m_maxVelY, m_maxVelZ));
    const float & speed2IdFactor =  maxSpeed * m_factorSpeed;
    BOOST_FOREACH(ParticlePtr particle, m_particles) {
        if (particle->age() > 1) {
//             const float & vx = m_centroidX - particle->xOld();
//             const float & vy = m_centroidY - particle->yOld();
//             const float & vz = m_centroidZ - particle->zOld();
            
            const float & vx = particle->vx();
            const float & vy = particle->vy();
            const float & vz = particle->vz();
            
            cv::Vec3f speedVector(vx, vy, vz);
            float speed = cv::norm(speedVector);
            if (speed != 0.0f)
                speedVector /= speed;
            
            if (vx == vy == vz == 0.0)
                speed = 0.0f;
            
            const uint32_t & idX = round(speedVector[0] + 1.0f);
            const uint32_t & idY = round(speedVector[1] + 1.0f);
            const uint32_t & idZ = round(speedVector[2] + 1.0f);
            const uint32_t & idSpeed = round(speed / speed2IdFactor);
            
//             cout << "speed " << speed << ", idSpeed " << idSpeed << ", maxSpeed " << maxSpeed << 
//                     ", m_factorSpeed " << m_factorSpeed << ", speed2IdFactor " << speed2IdFactor << endl;
            
            m_speedHistogram[idX][idY][idZ][idSpeed].numPoints += particle->age();
            totalPoints += particle->age();
        }
    }
    
    float maxProb = 0.0f;
    int32_t posX = 0, posY = 0, posZ = 0, posS = 0;
    for (int32_t x = 0; x < 3; x++) {
        for (int32_t y = 0; y < 3; y++) {
            for (int32_t z = 0; z < 3; z++) {
                // Avoid getting (0,0,0) as max probability. This possibility will be considered later.
//                 if (x == y == z == 1) {
//                     if (m_speedHistogram[x][y][z][0].numPoints != 0) {
//                         m_speedHistogram[x][y][z][0].probability = 
//                             m_speedHistogram[x][y][z][0].numPoints / (float)totalPoints;
//                     } else {
//                         m_speedHistogram[x][y][z][0].probability = 0.0;
//                     }
//                     
//                     continue;
//                 }
                for (int32_t s = 0; s < ceil(1.0 / m_factorSpeed) + 1; s++) {
                    if (m_speedHistogram[x][y][z][s].numPoints != 0) {
//                         if (m_speedHistogram[x][y][z][s].numPoints > maxProb) {
//                             maxProb = m_speedHistogram[x][y][z][s].numPoints;
//                             posX = x;
//                             posY = y;
//                             posZ = z;
//                             posS = s;
//                         }
                        posX += x * m_speedHistogram[x][y][z][s].numPoints;
                        posY += y * m_speedHistogram[x][y][z][s].numPoints;
                        posZ += z * m_speedHistogram[x][y][z][s].numPoints;
                        posS += s * m_speedHistogram[x][y][z][s].numPoints;
                        
//                         cout << cv::Vec4f(x - 1, y - 1, z - 1, s) << 
//                             " => " << m_speedHistogram[x][y][z][s].numPoints << 
//                             " => " << cv::Vec4f(posX - 1, posY - 1, posZ - 1, posS) <<
//                             " => " << maxProb << endl;               
                    } else {
                        m_speedHistogram[x][y][z][s].numPoints = 0.0f;
                    }
                }
            }
        }
    }
    
    if (totalPoints != 0) {
        posX /= totalPoints;
        posY /= totalPoints;
        posZ /= totalPoints;
        posS /= totalPoints;
    }
    
    m_vx = posX - 1;
    m_vy = posY - 1;
    m_vz = posZ - 1;
    m_magnitude = posS * m_factorSpeed * maxSpeed;
    
//     cout << cv::Vec4f(m_vx, m_vy, m_vz, m_magnitude)  << 
//     " => " << cv::Vec4f(posX - 1, posY - 1, posZ - 1, posS) << " => " << maxProb <<  " => " << totalPoints << endl;
    
    // FIXME: This is just for debugging visualization
//     m_vx = -1;
//     m_vy = -1;
//     m_vz = 0;
//     m_magnitude = 1.0;
    {
        m_vx = m_vy = m_vz = 0;
        uint32_t totalPoints = 0;
        const float & maxSpeed = cv::norm(cv::Vec3f(m_maxVelX, m_maxVelY, m_maxVelZ));
        const float & speed2IdFactor =  maxSpeed * m_factorSpeed;
        BOOST_FOREACH(ParticlePtr particle, m_particles) {
            //             if (particle->age() >= 1) {
            const float & vx = particle->vx();
            const float & vy = particle->vy();
            const float & vz = particle->vz();
            
            uint32_t increment = particle->age(); 
            m_vx += vx * increment;
            m_vy += vy * increment;
            m_vz += vz * increment;
            totalPoints += increment;
            //             }
        }
        
        m_vx /= totalPoints;
        m_vy /= totalPoints;
        m_vz /= totalPoints;
        
        cv::Vec3f speedVector(m_vx, m_vy, m_vz);
        m_magnitude = cv::norm(speedVector);
        if (m_magnitude != 0.0f) {
            speedVector /= m_magnitude;
            
            m_vx = speedVector[0];
            m_vy = speedVector[1];
            m_vz = speedVector[2];
        }
        
//         cout << cv::Vec4f(m_vx, m_vy, m_vz, m_magnitude)  << endl;
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
    stream << "[ gridPos " << cv::Vec3f(in.x(), in.y(), in.z()) << ", "
    << "centroid " << cv::Vec3f(in.centroidX(), in.centroidY(), in.centroidZ()) << ", " 
    << "speed " << cv::Vec3f(in.vx(), in.vy(), in.vz()) << "]";
    return stream;
}

}