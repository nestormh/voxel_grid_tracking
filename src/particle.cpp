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

#include "particle.h"

#include <stdlib.h>
#include <math.h>

using namespace std;

namespace polar_grid_tracking {

// random = true => const double & cellX, const double & cellZ, const double & cellSizeX, const double & cellSizeZ
    // random = false => const double & x, const double & z, const double & vx, const double & vz
Particle::Particle(const double & param1, const double & param2, const double & param3, const double & param4, const bool & random)
{
    if (random) {
        m_x = (param1 + (double)rand() / RAND_MAX) * param3;
        m_z = (param2 + (double)rand() / RAND_MAX) * param4;
        const double theta = (double)rand() / RAND_MAX * 2.0 * 3.14;
        m_vx = param3 * (double)rand() / RAND_MAX * cos(theta);
        m_vz = param4 * (double)rand() / RAND_MAX * sin(theta);
    } else {
        m_x = param1;
        m_z = param2;
        m_vx = param3;
        m_vz = param4;
    }
}

Particle::Particle(const Particle& particle)
{
    m_x = particle.x();
    m_z = particle.z();
    m_vx = particle.vx();
    m_vz = particle.vz();
}

void Particle::transform(const Eigen::Matrix4d & R, const Eigen::Vector4d & t, const Eigen::Matrix4d & stateTransition)
{
//     const double m_deltaYaw = -10.0 / 180.0 * 3.14;
//     const double m_deltaSpeed = 0.0;
//     const double m_deltaTime = 1.0;
//     
//     const double dx = m_deltaSpeed * m_deltaTime * cos(m_deltaYaw); // / m_cellSizeX;
//     const double dz = m_deltaSpeed * m_deltaTime * sin(m_deltaYaw); // / m_cellSizeZ;
//     
//     
//     ///////////////////////////////
//     
//     Eigen::Matrix4d R;
//     Eigen::Vector4d t;
//     
//     R << cos(m_deltaYaw), -sin(m_deltaYaw), 0, 0,
//          sin(m_deltaYaw), cos(m_deltaYaw), 0, 0,
//          0, 0, sin(-m_deltaYaw), cos(-m_deltaYaw),
//          0, 0, cos(-m_deltaYaw), -sin(-m_deltaYaw);
//          
//     t << dx, dz, 0, 0;
    
//     m_vx = 0.0;
//     m_vz = 1.0;
    
    ///////////////////////////////////////////////////
    
    Eigen::Vector4d newPosAndVel, oldPosAndVel;
    Eigen::Vector4d tmpPosAndVel, deltaPosAndVel, finalPosAndVel;
    
//     const double s = m_deltaSpeed * m_deltaTime;
    
//     cout << *this << endl;
    
    oldPosAndVel << m_x, m_z, m_vx, m_vz;
    
    // TODO: Fill with Q covariance, as indicated in the paper
    deltaPosAndVel << 0, 0, 0, 0;
        
    newPosAndVel = stateTransition * oldPosAndVel + deltaPosAndVel;
    
//     newPosAndVel << 0, 0, 0.5, 0.5;
    
//     cout << newPosAndVel << endl;
    
    finalPosAndVel = R * newPosAndVel + t;

//     cout << "finalPosAndVel " << finalPosAndVel << endl;
    
    m_x = finalPosAndVel(0);
    m_z = finalPosAndVel(1);
    m_vx = finalPosAndVel(2);
    m_vz = finalPosAndVel(3);
    
//     m_vx = newPosAndVel(2);
//     m_vz = newPosAndVel(3);
    
//     exit(0);
    
//     Eigen::Vector4d newPos, oldPos;
//     Eigen::Vector4d tmpPosAndVel, deltaPosAndVel, finalPosAndVel;
//     oldPos << m_x, m_z, 0, 0;
//     
// //     cout << "oldPos " << oldPos << endl;
// //     cout << "velIni " << cv::Point2d(m_vx, m_vz) << endl;
//     
//     newPos = R * (oldPos - t);
//     
//     tmpPosAndVel << newPos(0), newPos(1), m_vx, m_vz;
//     
//     // TODO: Fill with Q covariance, as indicated in the paper
//     deltaPosAndVel << 0, 0, 0, 0;
//     
//     finalPosAndVel = stateTransition * tmpPosAndVel + deltaPosAndVel;
//     
// //     cout << "finalPosAndVel " << finalPosAndVel << endl;
//     
//     m_x = finalPosAndVel(0);
//     m_z = finalPosAndVel(1);
//     m_vx = finalPosAndVel(2);
//     m_vz = finalPosAndVel(3);
}


void Particle::draw(cv::Mat& img, const uint32_t& pixelsPerCell, const double & cellSizeX, const double & cellSizeZ)
{
    const double factorX = pixelsPerCell / cellSizeX;
    const double factorZ = pixelsPerCell / cellSizeZ;
    const cv::Point2i p(m_x * factorX, m_z * factorZ);
    const cv::Point2i pSpeed((m_x + m_vx) * factorX, (m_z + m_vz) * factorZ);
    
    cv::line(img, p, pSpeed, cv::Scalar(0, 0, 255));
//     img.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(255, 0, 0);
    cv::circle(img, p, 3, cv::Scalar(255, 0, 0));
}

ostream& operator<<(ostream & stream, const Particle & in) {
    stream << "[" << in.x() << ", " << in.z() << ", " << in.vx() << ", " << in.vz() << "]";
    return stream;
}



}