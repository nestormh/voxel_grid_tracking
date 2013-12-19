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

namespace voxel_grid_tracking {
    
    VoxelObstacle::VoxelObstacle(const uint32_t& obstIdx, const double& threshYaw, const double& threshPitch, 
                                 const double& threshMagnitude, const double & minDensity, const SpeedMethod & speedMethod,
                                 Voxel& voxel) :
                                    m_idx(obstIdx), m_threshMagnitude(threshMagnitude), 
                                    m_threshYaw(threshYaw), m_threshPitch(threshPitch),
                                    m_minDensity(minDensity), m_speedMethod(speedMethod)
{
    
    
    addVoxelToObstacle(voxel);
    
}

VoxelObstacle::VoxelObstacle(const uint32_t& obstIdx, const double& threshYaw, 
                             const double& threshPitch, const double& threshMagnitude, 
                             const double & minDensity, const SpeedMethod & speedMethod) :
                                m_idx(obstIdx), m_threshMagnitude(threshMagnitude), 
                                m_threshYaw(threshYaw), m_threshPitch(threshPitch),
                                m_minDensity(minDensity), m_speedMethod(speedMethod)
{

}

bool VoxelObstacle::addVoxelToObstacle(Voxel& voxel)
{
    if (voxel.density() < m_minDensity)
        return false;
    
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


}
