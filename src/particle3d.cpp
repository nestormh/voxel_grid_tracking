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
#include </home/nestor/Dropbox/projects/GPUCPD/src/LU-Decomposition/Libs/Cuda/include/device_launch_parameters.h>

#include <stdlib.h>
#include <math.h>
#include <boost/graph/graph_concepts.hpp>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <tf_conversions/tf_eigen.h>

using namespace std;

namespace voxel_grid_tracking {

Particle3d::Particle3d(const double & centroidX, const double & centroidY, const double & centroidZ, 
                        const double & voxelSizeX, const double & voxelSizeY, const double & voxelSizeZ, 
                       const double & maxVelX, const double & maxVelY, const double & maxVelZ, 
                       const tf::StampedTransform & pose2mapTransform)
                            : m_maxVelX(maxVelX), m_maxVelY(maxVelY), m_maxVelZ(maxVelZ), 
                              m_pose2mapTransform(pose2mapTransform)
{
    m_x = centroidX;// + (((double)rand() / RAND_MAX) - 0.5) * voxelSizeX;
    m_y = centroidY;// + (((double)rand() / RAND_MAX) - 0.5) * voxelSizeY;
    m_z = centroidZ;// + (((double)rand() / RAND_MAX) - 0.5) * voxelSizeZ;
    
    tf::Vector3 point = /*m_pose2mapTransform * */tf::Vector3(m_x, m_y, m_z);
    
    m_x = point[0];
    m_y = point[1];
    m_z = point[2];
    
//     const double theta = 0.0; //((double)rand() / RAND_MAX) * 2.0 * M_PI;
//     const double gamma = 0.0; //((double)rand() / RAND_MAX) * 2.0 * M_PI;
    m_vx = m_maxVelX - 2.0 * m_maxVelX * ((double)rand() / RAND_MAX);
    m_vy = m_maxVelY - 2.0 * m_maxVelY * ((double)rand() / RAND_MAX);
    m_vz = m_maxVelZ - 2.0 * m_maxVelZ * ((double)rand() / RAND_MAX);
    
    m_age = 0;
    
    m_xOld = m_x;
    m_yOld = m_y;
    m_zOld = m_z;
    
    m_id = (int32_t)(255 * (double)rand() / RAND_MAX);
}

Particle3d::Particle3d(const double& x, const double& y, const double& z, 
                       const double& vx, const double& vy, const double& vz, 
                       const tf::StampedTransform & pose2mapTransform, const bool & transform)
                    : m_x(x), m_y(y), m_z(z), m_vx(vx), m_vy(vy), m_vz(vz), 
                      m_pose2mapTransform(pose2mapTransform)
{
    if (transform) {
        tf::Vector3 point = /*m_pose2mapTransform * */tf::Vector3(m_x, m_y, m_z);
        
        m_x = point[0];
        m_y = point[1];
        m_z = point[2];
    }
    
    m_age = 0;
    
    m_id = (int32_t)(255 * (double)rand() / RAND_MAX);
    
}

Particle3d::Particle3d(const Particle3d& particle)
                            : m_x(particle.x()), m_y(particle.y()), m_z(particle.z()), 
                                m_vx(particle.vx()), m_vy(particle.vy()), m_vz(particle.vz()),
                                m_pose2mapTransform(particle.pose2mapTransform()), m_age(particle.age()),
                                m_id(particle.id())
{
}

tf::Quaternion Particle3d::getQuaternion() const
{
    if (m_vx == m_vy == m_vz == 0.0) {
        return tf::Quaternion(0.0, 0.0, 0.0, 0.0);
    }
    
    Eigen::Vector3d zeroVector, currVector;
    zeroVector << 1.0, 0.0, 0.0;
    currVector << m_vx, m_vy, m_vz;
    currVector.normalize();
    Eigen::Quaterniond eigenQuat;
    eigenQuat.setFromTwoVectors(zeroVector, currVector);
    
    tf::Quaternion quat(eigenQuat.x(), eigenQuat.y(), eigenQuat.z(), eigenQuat.w());

    return quat;
}

void Particle3d::getYawPitch(double & yaw, double & pitch) const
{
    yaw = atan2(m_vy, m_vx);
    if (yaw < 0.0) yaw += CV_PI * 2.0;
    
    pitch = atan2(m_vz, m_vx);
    if (pitch < 0.0) pitch += CV_PI * 2.0;
}

void Particle3d::transform(const float & t)
{

    // TODO: Fill with Q covariance, as indicated in the paper
    float deltaX, deltaY, deltaZ, deltaVx, deltaVy, deltaVz;
    deltaX = deltaY = deltaZ = deltaVx = deltaVy = deltaVz = 0.0;
    
    m_xOld = m_x;
    m_yOld = m_y;
    m_zOld = m_z;
    
    m_x += m_vx * t + deltaX;
    m_y += m_vy * t + deltaY;
    m_z += m_vz * t + deltaZ;
    m_vx += deltaVx;
    m_vy += deltaVy;
    m_vz += deltaVz;
    
    m_age++;
}

void Particle3d::updatePosition(const float& x, const float& y, const float& z)
{
    m_x = x;
    m_y = y;
    m_z = z;
}


bool Particle3d::operator<(const Particle3d& particle) const
{
    return (m_age < particle.age());
}


ostream& operator<<(ostream & stream, const Particle3d & in) {
    stream << "[ pos: " << in.x() << ", " << in.y() << ", " << in.z() << ", speed: " 
           << in.vx() << ", " << in.vy() << ", " << in.vz() << ", age: " << in.age() << "]";
    return stream;
}

}