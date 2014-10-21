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
                                 Voxel& voxel) :
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

bool VoxelObstacle::addVoxelToObstacle(Voxel& voxel)
{
//     if (voxel.density() < m_minDensity)
//         return false;
    
    if (m_voxels.size() != 0) {
        const double & diffMagnitude = fabs(m_magnitude - voxel.magnitude());
        const double & diffYaw = fabs(calculateDifferenceBetweenAngles(m_yaw, voxel.yaw()));
        const double & diffPitch = fabs(calculateDifferenceBetweenAngles(m_pitch, voxel.pitch()));
        
        if ((diffMagnitude < m_threshMagnitude) && 
            (diffYaw < m_threshYaw) && 
            (diffPitch < m_threshPitch)) {
            
            voxel.assignObstacle(m_idx);
        
            updateWithVoxel(voxel);    
        
            m_voxels.push_back(voxel);
            
            return true;
        } else {
            return false;
        }
    } else {
        voxel.assignObstacle(m_idx);
        
        updateWithVoxel(voxel);
        
        m_voxels.push_back(voxel);
        
        return true;
    }
}

void VoxelObstacle::updateWithVoxel(const Voxel& voxel)
{
    if (m_voxels.size() != 0) {
        m_vx += voxel.vx();
        m_vy += voxel.vy();
        m_vz += voxel.vz();
        m_density += voxel.density();

        m_minX = min(m_minX, voxel.centroidX());
        m_maxX = max(m_maxX, voxel.centroidX());
        m_minY = min(m_minY, voxel.centroidY());
        m_maxY = max(m_maxY, voxel.centroidY());
        m_minZ = min(m_minZ, voxel.centroidZ());
        m_maxZ = max(m_maxZ, voxel.centroidZ());
        
        updateMotionInformation();
    } else {
        m_magnitude = voxel.magnitude();
        m_yaw = voxel.yaw();
        m_pitch = voxel.pitch();
        m_vx = voxel.vx();
        m_vy = voxel.vy();
        m_vz = voxel.vz();
        m_density = voxel.density();
        
        m_minX = voxel.centroidX();
        m_maxX = voxel.centroidX();
        m_minY = voxel.centroidY();
        m_maxY = voxel.centroidY();
        m_minZ = voxel.centroidZ();
        m_maxZ = voxel.centroidZ();
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
    
    m_density /= (double)m_voxels.size();    
}

bool VoxelObstacle::isObstacleConnected(const VoxelObstacle & obstacle)
{
    return true;
    
    BOOST_FOREACH(const Voxel & voxel1, m_voxels) {
        BOOST_FOREACH(const Voxel & voxel2, obstacle.voxels()) {
            if (voxel1.nextTo(voxel2)) {
                return true;
            }
        }
    }
    
    return false;
}

void VoxelObstacle::joinObstacles(VoxelObstacle& obstacle)
{
    BOOST_FOREACH(Voxel voxel, obstacle.voxels()) {
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
            
            BOOST_FOREACH(const Voxel & voxel, m_voxels) {
                m_vx += voxel.vx();
                m_vy += voxel.vy();
                m_vz += voxel.vz();
            }
            
            m_vx /= m_voxels.size();
            m_vy /= m_voxels.size();
            m_vz /= m_voxels.size();
            
            break;
        }
        case SPEED_METHOD_CIRC_HIST: {
//             cout << "SPEED_METHOD_CIRC_HIST" << endl;
            typedef boost::multi_array<polar_grid_tracking::t_histogram, 2> CircularHist;
            const uint32_t totalPitchBins = 2 * M_PI / m_pitchInterval;
            const uint32_t totalYawBins = 2 * M_PI / m_yawInterval;
            CircularHist histogram(boost::extents[totalPitchBins][totalYawBins]);
            
            BOOST_FOREACH(const Voxel & voxel, m_voxels) {
                double yaw = voxel.yaw();
                double pitch = voxel.pitch();
                
                uint32_t idxYaw = yaw / m_yawInterval;
                uint32_t idxPitch = pitch / m_pitchInterval;
                
                histogram[idxPitch][idxYaw].numPoints++;
                histogram[idxPitch][idxYaw].magnitudeSum += voxel.magnitude();
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
    
    m_vx += egoDeltaX;
    m_vy += egoDeltaY;
    m_vz += egoDeltaZ;
    
    m_magnitude = sqrt(m_vx * m_vx + m_vy * m_vy + m_vz * m_vz);
    
    const double & normYaw = sqrt(m_vx * m_vx + m_vy * m_vy);
    const double & normPitch = sqrt(m_vy * m_vy + m_vz * m_vz);
    
    m_yaw = acos(m_vx / normYaw);
    if (m_vy < 0)
        m_yaw = -m_yaw;
    
    m_pitch = asin(m_vz / normPitch);
}

}
