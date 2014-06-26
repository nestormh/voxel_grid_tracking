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
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>

using namespace std;

namespace voxel_grid_tracking {
    
class Particle3d
{
public:
    
    Particle3d(const double & centroidX, const double & centroidY, const double & centroidZ, 
               const double & voxelSizeX, const double & voxelSizeY, const double & voxelSizeZ, 
               const double & maxVelX, const double & maxVelY, const double & maxVelZ,
               const tf::StampedTransform & pose2mapTransform);
    Particle3d(const double & x, const double & y, const double & z, 
               const double & vx, const double & vy, const double & vz, 
               const tf::StampedTransform & pose2mapTransform);
    
    Particle3d(const Particle3d & particle);
    
    void transform(const Eigen::MatrixXd & stateTransition);
    
    double x() const { return m_x; }
    double y() const { return m_y; }
    double z() const { return m_z; }
    double vx() const { return m_vx; }
    double vy() const { return m_vy; }
    double vz() const { return m_vz; }
    
    uint32_t age() const { return m_age; }
    
    tf::Quaternion getQuaternion() const;
    void getYawPitch(double & yaw, double & pitch) const;
    
    tf::StampedTransform pose2mapTransform() const { return m_pose2mapTransform; }
    
    bool operator < (const Particle3d & particle) const;
    
    
private:
    double m_x, m_y, m_z, m_vx, m_vy, m_vz;
    
    uint32_t m_age;
    
    double m_maxVelX, m_maxVelY, m_maxVelZ;
    
    tf::StampedTransform m_pose2mapTransform;
};

ostream& operator<<(ostream & stream, const Particle3d & in);
    
}
#endif // PARTICLE_H