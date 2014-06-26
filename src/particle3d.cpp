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
    
    tf::Vector3 point = m_pose2mapTransform * tf::Vector3(m_x, m_y, m_z);
    
    m_x = point[0];
    m_y = point[1];
    m_z = point[2];
    
//     const double theta = 0.0; //((double)rand() / RAND_MAX) * 2.0 * M_PI;
//     const double gamma = 0.0; //((double)rand() / RAND_MAX) * 2.0 * M_PI;
    m_vx = m_maxVelX - 2.0 * m_maxVelX * ((double)rand() / RAND_MAX);
    m_vy = m_maxVelY - 2.0 * m_maxVelY * ((double)rand() / RAND_MAX);
    m_vz = m_maxVelZ - 2.0 * m_maxVelZ * ((double)rand() / RAND_MAX);
    
    m_age = 0;
}

Particle3d::Particle3d(const double& x, const double& y, const double& z, 
                       const double& vx, const double& vy, const double& vz, 
                       const tf::StampedTransform & pose2mapTransform)
                    : m_x(x), m_y(y), m_z(z), m_vx(vx), m_vy(vy), m_vz(vz), 
                      m_pose2mapTransform(pose2mapTransform)
{
    tf::Vector3 point = m_pose2mapTransform * tf::Vector3(m_x, m_y, m_z);
    
    m_x = point[0];
    m_y = point[1];
    m_z = point[2];
    
    m_age = 0;
}

Particle3d::Particle3d(const Particle3d& particle)
                            : m_x(particle.x()), m_y(particle.y()), m_z(particle.z()), 
                                m_vx(particle.vx()), m_vy(particle.vy()), m_vz(particle.vz()),
                                m_pose2mapTransform(particle.pose2mapTransform()), m_age(particle.age())
{
}

tf::Quaternion Particle3d::getQuaternion() const
{
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

void Particle3d::transform(const Eigen::MatrixXd& stateTransition)
{
    Eigen::VectorXd newPosAndVel(6), oldPosAndVel(6);
    Eigen::VectorXd deltaPosAndVel(6);
    
    oldPosAndVel << m_x, m_y, m_z, m_vx, m_vy, m_vz;
    
    // TODO: Fill with Q covariance, as indicated in the paper
    deltaPosAndVel << 0, 0, 0, 0, 0, 0;
        
    newPosAndVel = stateTransition * oldPosAndVel + deltaPosAndVel;
    
    m_x = newPosAndVel(0);
    m_y = newPosAndVel(1);
    m_z = newPosAndVel(2);
    m_vx = newPosAndVel(3);
    m_vy = newPosAndVel(4);
    m_vz = newPosAndVel(5);
    
    m_age++;
}

bool Particle3d::operator<(const Particle3d& particle) const
{
    return (m_age < particle.age());
}


ostream& operator<<(ostream & stream, const Particle3d & in) {
    stream << "[" << in.x() << ", " << in.y() << ", " << in.z() << ", " 
           << in.vx() << ", " << in.vy() << ", " << in.vz() << ":::age: " << in.age() << "]";
    return stream;
}

}