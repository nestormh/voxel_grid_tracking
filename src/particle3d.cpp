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
}

Particle3d::Particle3d(const Particle3d& particle)
                            : m_x(particle.x()), m_y(particle.y()), m_z(particle.z()), 
                                m_vx(particle.vx()), m_vy(particle.vy()), m_vz(particle.vz()),
                                m_pose2mapTransform(particle.pose2mapTransform()) 
{
}

void Particle3d::getYawPitch(double& yaw, double& pitch) const
{
    
//     cout << "m_vx " << m_vx << endl;
//     cout << "m_vy " << m_vy << endl;
//     cout << "m_vz " << m_vz << endl;
//     
//     const cv::Vec3d zeroVec(1.0, 0.0, 0.0);
//     
//     cv::Vec3d currVecYaw(m_vx, m_vy, 0.0);
//     currVecYaw /= cv::norm(currVecYaw);
//     
//     cv::Vec3d currVecPitch(m_vx, 0.0, m_vz);
//     currVecPitch /= cv::norm(currVecPitch);
//     
//     if (cv::norm(currVecYaw) == 0.0)
//         yaw = 0.0;
//     else {
//         yaw = acos(currVecYaw.dot(zeroVec));
// //         cout << "cos " << currVecYaw.dot(zeroVec) << endl;
// //         cout << "yaw " << yaw * 180.0 / CV_PI << endl;
//         if (m_vy < 0.0)
//             yaw = /*2 * CV_PI*/ - yaw;
// //         cout << "yaw2 " << yaw * 180.0 / CV_PI  << endl;
//     }
//     if (cv::norm(currVecPitch) == 0.0)
//         pitch = 0.0;
//     else {
//         pitch = acos(currVecPitch.dot(zeroVec));
//         cout << "cos " << currVecPitch.dot(zeroVec) << endl;
//         cout << "pitch " << pitch * 180.0 / CV_PI << endl;
//         if ((m_vz < 0.0) && (m_vy >= 0.0))
//             pitch = /*2 * CV_PI*/ - pitch;
//         cout << "pitch " << pitch * 180.0 / CV_PI  << endl;
//     }
//     cv::Vec3f currVector(m_vx, m_vy, m_vz);
//     currVector /= cv::norm(currVector);
//     
//     yaw = atan2(currVector[1], currVector[0]);
//     const float padj = cv::norm(cv::Vec2f(m_vx, m_vy));
// //     const float padj = sqrt(pow(x, 2) + pow(z, 2)); 
//     pitch = atan2(fabs(m_x) * cos(yaw), m_z);
    yaw = atan2(m_x, m_y);
    pitch = 0.0;
    
//     if (m_y >= 0.0) {
//         pitch = -atan2(m_x * cos(yaw), m_y );
//     }else{
//         pitch = atan2(m_x * cos(yaw), -m_y);
//     }
//     roty = Math.atan2( x * Math.cos(rotx), z )
//     rotz = Math.atan2( Math.cos(rotx), Math.sin(rotx) * Math.sin(roty) )
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
}

ostream& operator<<(ostream & stream, const Particle3d & in) {
    stream << "[" << in.x() << ", " << in.y() << ", " << in.z() << ", " 
           << in.vx() << ", " << in.vy() << ", " << in.vz() << "]";
    return stream;
}

}