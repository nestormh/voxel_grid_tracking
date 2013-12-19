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


#ifndef VOXELOBSTACLE_H
#define VOXELOBSTACLE_H

#include "voxel.h"

namespace voxel_grid_tracking {
    
class VoxelObstacle
{
public:
    VoxelObstacle(const uint32_t & obstIdx, const double & threshYaw, const double & threshPitch, 
             const double & threshMagnitude, Voxel & voxel);
    VoxelObstacle(const uint32_t & obstIdx, const double & threshYaw, const double & threshPitch, 
             const double & threshMagnitude);
    bool addVoxelToObstacle(Voxel & voxel);
    
    vector<Voxel> voxels() const { return m_voxels; }
    
    uint32_t idx() const { return m_idx; }
    
    double density() const { return m_density; }
    
    uint32_t numVoxels() const { return m_voxels.size(); }
    
    bool isObstacleConnected(const VoxelObstacle & obstacle);
    void joinObstacles(VoxelObstacle & obstacle);
    
protected:
    void updateMotionInformation();
    
    vector<Voxel> m_voxels;
    
    uint32_t m_idx;
    double m_magnitude;
    double m_yaw;
    double m_pitch;
    
    double m_density;
    
    double m_vx, m_vy, m_vz;
    
    // Params
    double m_threshYaw, m_threshPitch, m_threshMagnitude;
};

}

#endif // VOXELOBSTACLE_H
