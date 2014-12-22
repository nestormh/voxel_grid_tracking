/*
    Copyright 2013 Néstor Morales Hernández <email>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#include "voxelobstacle.h"
#include "utilspolargridtracking.h"

#include <boost/foreach.hpp>
#include <boost/graph/graph_concepts.hpp>

namespace voxel_grid_tracking {
    
    VoxelObstacle::VoxelObstacle(const uint32_t& obstIdx, const double& threshYaw, const double& threshPitch, 
                                 const double& threshMagnitude, const double & minDensity, const SpeedMethod & speedMethod,
                                 const double & yawInterval, const double & pitchInterval,
                                 VoxelPtr& voxel) :
                                    m_idx(obstIdx), m_threshMagnitude(threshMagnitude), 
                                    m_threshYaw(threshYaw), m_threshPitch(threshPitch),
                                    m_minDensity(minDensity), m_speedMethod(speedMethod),
                                    m_yawInterval(yawInterval), m_pitchInterval(pitchInterval)
{
    
    
    addVoxelToObstacle(voxel);
    
}

VoxelObstacle::VoxelObstacle(const uint32_t& obstIdx, const double& threshYaw, 
                             const double& threshPitch, const double& threshMagnitude, 
                             const double & minDensity, const SpeedMethod & speedMethod,
                             const double & yawInterval, const double & pitchInterval) :
                                m_idx(obstIdx), m_threshMagnitude(threshMagnitude), 
                                m_threshYaw(threshYaw), m_threshPitch(threshPitch),
                                m_minDensity(minDensity), m_speedMethod(speedMethod),
                                m_yawInterval(yawInterval), m_pitchInterval(pitchInterval)
{

}

bool VoxelObstacle::addVoxelToObstacle(VoxelPtr& voxel)
{
//     if (voxel.density() < m_minDensity)
//         return false;
    
//     if (m_voxels.size() != 0) {
//         const double & diffMagnitude = fabs(m_magnitude - voxel.magnitude());
//         const double & diffYaw = fabs(calculateDifferenceBetweenAngles(m_yaw, voxel.yaw()));
//         const double & diffPitch = fabs(calculateDifferenceBetweenAngles(m_pitch, voxel.pitch()));
//         
//         if ((diffMagnitude < m_threshMagnitude) && 
//             (diffYaw < m_threshYaw) && 
//             (diffPitch < m_threshPitch)) {
//             
//             voxel.assignObstacle(m_idx);
//         
//             updateWithVoxel(voxel);    
//         
//             m_voxels.push_back(voxel);
//             
//             return true;
//         } else {
//             return false;
//         }
//     } else {
        voxel->assignObstacle(m_idx);
        
        updateWithVoxel(voxel);
        
        m_voxels.push_back(voxel);
        
        return true;
//     }
}

void VoxelObstacle::updateWithVoxel(const VoxelPtr& voxel)
{
    if (m_voxels.size() != 0) {
        m_vx += voxel->vx();
        m_vy += voxel->vy();
        m_vz += voxel->vz();

        m_minX = min(m_minX, voxel->centroidX());
        m_maxX = max(m_maxX, voxel->centroidX());
        m_minY = min(m_minY, voxel->centroidY());
        m_maxY = max(m_maxY, voxel->centroidY());
        m_minZ = min(m_minZ, voxel->centroidZ());
        m_maxZ = max(m_maxZ, voxel->centroidZ());
        
        updateMotionInformation();
    } else {
        m_magnitude = voxel->magnitude();
        m_yaw = voxel->yaw();
        m_pitch = voxel->pitch();
        m_vx = voxel->vx();
        m_vy = voxel->vy();
        m_vz = voxel->vz();
        
        m_minX = voxel->centroidX();
        m_maxX = voxel->centroidX();
        m_minY = voxel->centroidY();
        m_maxY = voxel->centroidY();
        m_minZ = voxel->centroidZ();
        m_maxZ = voxel->centroidZ();
    }
    
}

void VoxelObstacle::updateMotionInformation()
{
    double vx = m_vx / m_voxels.size();
    double vy = m_vy / m_voxels.size();
    double vz = m_vz / m_voxels.size();
    
    m_magnitude = sqrt(vx * vx + vy * vy + vz * vz);
    
    const double & normYaw = sqrt(vx * vx + vy * vy);
    const double & normPitch = sqrt(vy * vy + vz * vz);
    
    m_yaw = acos(vx / normYaw);
    if (vy < 0)
        m_yaw = -m_yaw;
    
    m_pitch = asin(vz / normPitch);
    if (vy < 0)
        m_pitch = -m_pitch;
}

void VoxelObstacle::updateHistogram(const float & maxVelX, const float & maxVelY, 
                                    const float & maxVelZ, const float & factorSpeed,
                                    const float & minVel)
{
//     cout << "-----------------------------------------" << endl;
//     cout << "Analyzing " << m_idx << endl;
    
    SpeedHistogram speedHistogram;
    speedHistogram.resize(boost::extents[3][3][3][((int)ceil(1.0 / factorSpeed)) + 1]);
    
    uint32_t totalPoints = 0;
    const float & maxSpeed = cv::norm(cv::Vec3f(maxVelX, maxVelY, maxVelZ));
    const float & speed2IdFactor =  maxSpeed * factorSpeed;
    BOOST_FOREACH(VoxelPtr voxel, m_voxels) {
        const ParticleList & particles = voxel->getParticles();
        BOOST_FOREACH(ParticlePtr particle, particles) {
//             if (particle->age() >= 1) {
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
                
//                 cout << "speed " << speed << ", idSpeed " << idSpeed << ", maxSpeed " << maxSpeed << 
//                         ", m_factorSpeed " << factorSpeed << ", speed2IdFactor " << speed2IdFactor << endl;
                
                speedHistogram[idX][idY][idZ][idSpeed].numPoints += particle->age();
                totalPoints += particle->age();
//             }
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
                for (int32_t s = 0; s < ceil(1.0 / factorSpeed) + 1; s++) {
                    if (speedHistogram[x][y][z][s].numPoints != 0) {
//                             if (speedHistogram[x][y][z][s].numPoints > maxProb) {
//                                 maxProb = speedHistogram[x][y][z][s].numPoints;
//                                 posX = x;
//                                 posY = y;
//                                 posZ = z;
//                                 posS = s;
//                             }
                        posX += x * speedHistogram[x][y][z][s].numPoints;
                        posY += y * speedHistogram[x][y][z][s].numPoints;
                        posZ += z * speedHistogram[x][y][z][s].numPoints;
                        posS += s * speedHistogram[x][y][z][s].numPoints;
                        
//                         cout << cv::Vec4f(x - 1, y - 1, z - 1, s) << 
//                         " => " << speedHistogram[x][y][z][s].numPoints << 
//                         " => " << cv::Vec4f(posX - 1, posY - 1, posZ - 1, posS) <<
//                         " => " << maxProb << endl;               
                    } else {
                        speedHistogram[x][y][z][s].numPoints = 0.0f;
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
    m_magnitude = posS * factorSpeed * maxSpeed;
    
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
        const float & maxSpeed = cv::norm(cv::Vec3f(maxVelX, maxVelY, maxVelZ));
        const float & speed2IdFactor =  maxSpeed * factorSpeed;
        BOOST_FOREACH(VoxelPtr voxel, m_voxels) {
            const ParticleList & particles = voxel->getParticles();
            BOOST_FOREACH(ParticlePtr particle, particles) {
                            if (particle->age() >= 2) {
                const float & vx = particle->vx();
                const float & vy = particle->vy();
                const float & vz = particle->vz();
                
                uint32_t increment = particle->age(); 
                m_vx += vx * increment;
                m_vy += vy * increment;
                m_vz += vz * increment;
                totalPoints += increment;
                            }
            }
        }
    
        if (totalPoints != 0) {
            m_vx /= totalPoints;
            m_vy /= totalPoints;
            m_vz /= totalPoints;
        }
            
        cv::Vec3f speedVector(m_vx, m_vy, m_vz);
        m_magnitude = cv::norm(speedVector);
        if (m_magnitude != 0.0f) {
            speedVector /= m_magnitude;
            
            m_vx = speedVector[0];
            m_vy = speedVector[1];
            m_vz = speedVector[2];
        }
        
        if (m_magnitude < minVel) {
            m_vx = m_vy = m_vz = m_magnitude = 0.0;
        }
        
//         cout << cv::Vec4f(m_vx, m_vy, m_vz, m_magnitude)  << endl;
    }
    
    
}

bool VoxelObstacle::isObstacleConnected(const VoxelObstacle & obstacle)
{
    return true;
    
    BOOST_FOREACH(const VoxelPtr & voxel1, m_voxels) {
        BOOST_FOREACH(const VoxelPtr & voxel2, obstacle.voxels()) {
            if (voxel1->nextTo(*voxel2)) {
                return true;
            }
        }
    }
    
    return false;
}

void VoxelObstacle::joinObstacles(VoxelObstacle& obstacle)
{
    BOOST_FOREACH(VoxelPtr voxel, obstacle.voxels()) {
        addVoxelToObstacle(voxel);
    }
}

void VoxelObstacle::update(const double & m_voxelSizeX, const double & m_voxelSizeY, const double & m_voxelSizeZ)
{
    updateMotionInformation();
    
    m_centerX = (m_maxX + m_minX) / 2.0;
    m_centerY = (m_maxY + m_minY) / 2.0;
    m_centerZ = (m_maxZ + m_minZ) / 2.0;
    
    m_sizeX = m_maxX - m_minX + m_voxelSizeX;
    m_sizeY = m_maxY - m_minY + m_voxelSizeY;
    m_sizeZ = m_maxZ - m_minZ + m_voxelSizeZ;
}

double VoxelObstacle::commonVolume(const VoxelObstacle& obst1, const VoxelObstacle& obst2)
{
    const double minX = max(obst1.minX(), obst2.minX());
    const double minY = max(obst1.minY(), obst2.minY());
    const double minZ = max(obst1.minZ(), obst2.minZ());
    const double maxX = min(obst1.maxX(), obst2.maxX());
    const double maxY = min(obst1.maxY(), obst2.maxY());
    const double maxZ = min(obst1.maxZ(), obst2.maxZ());
    
    const double szX = maxX - minX;
    const double szY = maxY - minY;
    const double szZ = maxZ - minZ;
    
    if ((szX < 0.0) || (szY < 0.0) || (szZ < 0.0))
        return 0.0;
    else
        return szX * szY * szZ;
}

void VoxelObstacle::updateSpeed(const double & egoDeltaX, const double & egoDeltaY, const double & egoDeltaZ)
{
    switch (m_speedMethod) {
        case SPEED_METHOD_MEAN: {
//             cout << "SPEED_METHOD_MEAN" << endl;
            m_vx = 0.0;
            m_vy = 0.0;
            m_vz = 0.0;
            m_yaw = 0.0;
            m_pitch = 0.0;
            m_magnitude = 0.0;
            
            BOOST_FOREACH(const VoxelPtr & voxel, m_voxels) {
                m_vx += voxel->vx();
                m_vy += voxel->vy();
                m_vz += voxel->vz();
                
                cv::Vec3d tmpV(voxel->vx(), voxel->vy(), voxel->vz());
                m_magnitude += cv::norm(tmpV);
                
//                 cout << "voxel.vx() " << voxel.vx() << endl;
//                 cout << "voxel.vy() " << voxel.vy() << endl;
//                 cout << "voxel.vz() " << voxel.vz() << endl;
//                 cout << "m_vx " << m_vx << endl;
//                 cout << "m_vy " << m_vy << endl;
//                 cout << "m_vz " << m_vz << endl;
//                 cout << "m_magnitude " << m_magnitude << endl;
//                 cout << "---------------------------" << endl;
            }
            
            cv::Vec3d v(m_vx, m_vy, m_vz);
            double norm = cv::norm(v);
            
//             m_vx /= norm;
//             m_vy /= norm;
//             m_vz /= norm;
//             
//             m_vx *= m_magnitude;
//             m_vy *= m_magnitude;
//             m_vz *= m_magnitude;
//             
            break;
        }
        case SPEED_METHOD_CIRC_HIST: {
//             cout << "SPEED_METHOD_CIRC_HIST" << endl;
            typedef boost::multi_array<polar_grid_tracking::t_histogram, 2> CircularHist;
            const uint32_t totalPitchBins = 2 * M_PI / m_pitchInterval;
            const uint32_t totalYawBins = 2 * M_PI / m_yawInterval;
            CircularHist histogram(boost::extents[totalPitchBins][totalYawBins]);
            
            BOOST_FOREACH(const VoxelPtr & voxel, m_voxels) {
                double yaw = voxel->yaw();
                double pitch = voxel->pitch();
                
                uint32_t idxYaw = yaw / m_yawInterval;
                uint32_t idxPitch = pitch / m_pitchInterval;
                
                histogram[idxPitch][idxYaw].numPoints++;
                histogram[idxPitch][idxYaw].magnitudeSum += voxel->magnitude();
            }
            
            uint32_t maxIdxPitch = 0;
            uint32_t maxIdxYaw = 0;
            uint32_t numVectors = 0;
            uint32_t lastNumVectors = 0;
            for (uint32_t idxPitch = 0; idxPitch < totalPitchBins; idxPitch++) {
                for (uint32_t idxYaw = 0; idxYaw < totalYawBins; idxYaw++) {
                    if (histogram[idxPitch][idxYaw].numPoints > numVectors) {
                        lastNumVectors = numVectors;
                        numVectors = histogram[idxPitch][idxYaw].numPoints;
                        maxIdxPitch = idxPitch;
                        maxIdxYaw = idxYaw;
                    }
                }
            }
            
            m_yaw = maxIdxYaw * m_yawInterval;
            m_pitch = maxIdxPitch * m_pitchInterval;
            m_magnitude = histogram[maxIdxPitch][maxIdxYaw].magnitudeSum / histogram[maxIdxPitch][maxIdxYaw].numPoints;

//             if (m_magnitude != 0.0) {
//                 m_yaw -= egoDeltaYaw;
//     //             if (m_yaw < 0)
//     //                 m_yaw += M_PI;
//                 
//                 m_pitch -= egoDeltaPitch;
//     //             if (m_pitch < 0)
//     //                 m_pitch += M_PI;
//                 
//                 m_magnitude -= egoSpeed;
//                 if (m_magnitude < 0) {
//                     m_yaw += M_PI;
//                     m_pitch += M_PI;
//                     m_magnitude = fabs(m_magnitude);
//                 }
//             }
            
            m_vx = m_magnitude * cos(m_yaw) * cos(m_pitch);
            m_vy = m_magnitude * sin(m_yaw) * cos(m_pitch);
            m_vz = m_magnitude * sin(m_pitch);
            
            break;
        }
        default: {
            ROS_ERROR("Speed method not known: %d", m_speedMethod);
            exit(-1);
        }
    }
    
//     m_vx += egoDeltaX;
//     m_vy += egoDeltaY;
//     m_vz += egoDeltaZ;

    m_magnitude = sqrt(m_vx * m_vx + m_vy * m_vy + m_vz * m_vz);
    
    const double & normYaw = sqrt(m_vx * m_vx + m_vy * m_vy);
    const double & normPitch = sqrt(m_vy * m_vy + m_vz * m_vz);
    
    m_yaw = acos(m_vx / normYaw);
    if (m_vy < 0)
        m_yaw = -m_yaw;
    
    m_pitch = asin(m_vz / normPitch);
    
//     cout << "size " << m_voxels.size() << endl;
//     cout << "m_vx " << m_vx << endl;
//     cout << "m_vy " << m_vy << endl;
//     cout << "m_vz " << m_vz << endl;
//     cout << "sum " << m_vx + m_vy + m_vz << endl;
//     cout << "m_magnitude " << m_magnitude << endl;
//     cout << "m_yaw " << m_yaw << endl;
//     cout << "m_pitch " << m_pitch << endl;
//     cout << "====================================" << endl;
}

void VoxelObstacle::updateSpeedFromParticles()
{
    switch (m_speedMethod) {
        case SPEED_METHOD_MEAN: {
            m_vx = 0.0;
            m_vy = 0.0;
            m_vz = 0.0;
            m_yaw = 0.0;
            m_pitch = 0.0;
            m_magnitude = 0.0;
            uint32_t countParticles = 0;
            
            m_centerX = m_centerY = m_centerZ = 0.0;
            BOOST_FOREACH(const VoxelPtr & voxel, m_voxels) {
                BOOST_FOREACH(const ParticlePtr & particle, voxel->getParticles()) {
                    m_vx += particle->vx();
                    m_vy += particle->vy();
                    m_vz += particle->vz();
                    
                    countParticles++;
                }
                m_centerX += voxel->centroidX();
                m_centerY += voxel->centroidY();
                m_centerZ += voxel->centroidZ();
            }
            m_centerX /= m_voxels.size();
            m_centerY /= m_voxels.size();
            m_centerZ /= m_voxels.size();
            
            m_vx /= countParticles;
            m_vy /= countParticles;
            m_vz /= countParticles;
            
            cv::Vec3d v(m_vx, m_vy, m_vz);
            cv::Vec3d vBase(1.0, 0.0, 0.0);
            
            m_magnitude = cv::norm(v);
            
            v /= m_magnitude;
            
            const double & normYaw = cv::norm(cv::Vec2d(m_vx, m_vy));
            const double & normPitch = cv::norm(cv::Vec2d(m_vy, m_vz));
            
            m_yaw = acos(m_vx / normYaw);
            if (m_vy < 0)
                m_yaw = -m_yaw;
            
            m_pitch = asin(m_vz / normPitch);
            
            // We check the results
            double stdevX = 0.0, stdevY = 0.0, stdevZ = 0.0;
            BOOST_FOREACH(const VoxelPtr & voxel, m_voxels) {
                BOOST_FOREACH(const ParticlePtr & particle, voxel->getParticles()) {
                    const double & diffX = particle->vx() - m_vx;
                    const double & diffY = particle->vy() - m_vy;
                    const double & diffZ = particle->vz() - m_vz;
                    stdevX += diffX * diffX;
                    stdevY += diffY * diffY;
                    stdevZ += diffZ * diffZ;                                  
                }
            }
            stdevX /= countParticles - 1;
            stdevY /= countParticles - 1;
            stdevZ /= countParticles - 1;
            
            stdevX = sqrt(stdevX);
            stdevY = sqrt(stdevY);
            stdevZ = sqrt(stdevZ);
            
            stringstream ss;
            ss << std::setprecision(3) << fabs(stdevX) << ", " << fabs(stdevY) << 
                                    ", " << fabs(stdevZ);
            m_winnerNumberOfParticles = ss.str();

            break;
        }
        case SPEED_METHOD_CIRC_HIST: {
            
//             cout << "SPEED_METHOD_CIRC_HIST" << endl;
            
            typedef boost::multi_array<polar_grid_tracking::t_histogram, 2> CircularHist;
            const uint32_t totalPitchBins = 2 * M_PI / m_pitchInterval;
            const uint32_t totalYawBins = 2 * M_PI / m_yawInterval;
            CircularHist histogram(boost::extents[totalPitchBins][totalYawBins]);
            
            m_centerX = m_centerY = m_centerZ = 0.0;
            BOOST_FOREACH(const VoxelPtr & voxel, m_voxels) {
                BOOST_FOREACH(const ParticlePtr & particle, voxel->getParticles()) {
                    if (particle->age() > 1) {
                        double yaw, pitch;
                        particle->getYawPitch(yaw, pitch);
                        
                        uint32_t idxYaw = yaw / m_yawInterval;
                        uint32_t idxPitch = pitch / m_pitchInterval;
                        
                        histogram[idxPitch][idxYaw].numPoints++;
                        histogram[idxPitch][idxYaw].magnitudeSum += cv::norm(cv::Vec3f(particle->vx(), particle->vy(), particle->vz()));
                    }
                }
                m_centerX += voxel->centroidX();
                m_centerY += voxel->centroidY();
                m_centerZ += voxel->centroidZ();
            }
            m_centerX /= m_voxels.size();
            m_centerY /= m_voxels.size();
            m_centerZ /= m_voxels.size();
            
            uint32_t maxIdxPitch = 0;
            uint32_t maxIdxYaw = 0;
            uint32_t numVectors = 0;
            uint32_t lastNumVectors = 0;
            for (uint32_t idxPitch = 0; idxPitch < totalPitchBins; idxPitch++) {
                for (uint32_t idxYaw = 0; idxYaw < totalYawBins; idxYaw++) {
                    if (histogram[idxPitch][idxYaw].numPoints > numVectors) {
                        lastNumVectors = numVectors;
                        numVectors = histogram[idxPitch][idxYaw].numPoints;
                        maxIdxPitch = idxPitch;
                        maxIdxYaw = idxYaw;
                    }
                }
            }

            stringstream ss;
            ss << numVectors;
            m_winnerNumberOfParticles = ss.str();
            m_yaw = maxIdxYaw * m_yawInterval;
            m_pitch = maxIdxPitch * m_pitchInterval;
            m_magnitude = histogram[maxIdxPitch][maxIdxYaw].magnitudeSum / histogram[maxIdxPitch][maxIdxYaw].numPoints;
            
            m_vx = cos(m_yaw) * m_magnitude;
            m_vy = sin(m_yaw) * m_magnitude;
            m_vz = sin(m_pitch) * m_magnitude;
            
//             cout << "maxIdxYaw " << maxIdxYaw << endl;
//             cout << "maxIdxPitch " << m_centerY << endl;
//             cout << "m_yawInterval " << m_yawInterval << endl;
//             cout << "m_pitchInterval " << m_pitchInterval << endl;
//             cout << "m_centerX " << m_centerX << endl;
//             cout << "m_centerY " << m_centerY << endl;
//             cout << "m_centerZ " << m_centerZ << endl;
//             cout << "m_yaw " << m_yaw << endl;
//             cout << "m_pitch " << m_pitch << endl;
//             cout << "m_magnitude " << m_magnitude << endl;
//             cout << "m_vx " << m_vx << endl;
//             cout << "m_vy " << m_vy << endl;
//             cout << "m_vz " << m_vz << endl;
//             cout << "numVectors " << numVectors << endl;
//             cout << "lastNumVectors " << lastNumVectors << endl;
//             cout << "(numVectors - lastNumVectors) " << (numVectors - lastNumVectors) << endl;
//             cout << "speed " << m_magnitude * 3.6 << endl;
//             
//             
//             cout << "==============================" << endl;
            
            m_yaw = maxIdxYaw * m_yawInterval;
            m_pitch = maxIdxPitch * m_pitchInterval;
            m_magnitude = histogram[maxIdxPitch][maxIdxYaw].magnitudeSum / histogram[maxIdxPitch][maxIdxYaw].numPoints;
            
//             m_vx = m_magnitude * cos(m_yaw) * cos(m_pitch);
//             m_vy = m_magnitude * sin(m_yaw) * cos(m_pitch);
//             m_vz = m_magnitude * sin(m_pitch);
            
            break;
        }
        default: {
            ROS_ERROR("Speed method not known: %d", m_speedMethod);
            exit(-1);
        }
    }
    
//     m_magnitude = sqrt(m_vx * m_vx + m_vy * m_vy + m_vz * m_vz);
//     
//     const double & normYaw = sqrt(m_vx * m_vx + m_vy * m_vy);
//     const double & normPitch = sqrt(m_vy * m_vy + m_vz * m_vz);
//     
//     m_yaw = acos(m_vx / normYaw);
//     if (m_vy < 0)
//         m_yaw = -m_yaw;
//     
//     m_pitch = asin(m_vz / normPitch);
}


}
