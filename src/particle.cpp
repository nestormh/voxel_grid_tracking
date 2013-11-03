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

    
Particle::Particle(const double & cellX, const double & cellZ, const double & cellSizeX, const double & cellSizeZ)
{
    m_x = (cellX + (double)rand() / RAND_MAX) * cellSizeX;
    m_z = (cellZ + (double)rand() / RAND_MAX) * cellSizeZ;
    const double theta = (double)rand() / RAND_MAX * 2.0 * 3.14;
    m_vx = cellSizeX * (double)rand() / RAND_MAX * cos(theta);
    m_vz = cellSizeZ * (double)rand() / RAND_MAX * sin(theta);
}

Particle::Particle(const Particle& particle)
{
    m_x = particle.x();
    m_z = particle.z();
    m_vx = particle.vx();
    m_vz = particle.vz();
}

void Particle::transform(const Eigen::Matrix2d & R, const Eigen::Vector2d & t, const Eigen::Matrix4d & stateTransition)
{
    Eigen::Vector2d newPos, oldPos;
    Eigen::Vector4d tmpPosAndVel, deltaPosAndVel, finalPosAndVel;
    oldPos << m_x, m_z;
    
    newPos = R * oldPos - t;
    
    tmpPosAndVel << newPos(0), newPos(1), m_vx, m_vz;
    
    // TODO: Fill with Q covariance, as indicated in the paper
    deltaPosAndVel << 0, 0, 0, 0;
    
    finalPosAndVel = stateTransition * tmpPosAndVel + deltaPosAndVel;
    
    m_x = finalPosAndVel(0);
    m_z = finalPosAndVel(1);
    m_vx = finalPosAndVel(2);
    m_vz = finalPosAndVel(3);

//     cout << "+++oldPos\n" << oldPos << endl;
//     cout << "+++newPos\n" << newPos << endl;
//     cout << "+++tmpPosAndVel\n" << tmpPosAndVel << endl;
//     cout << "+++deltaPosAndVel\n" << deltaPosAndVel << endl;
//     cout << "+++finalPosAndVel\n" << finalPosAndVel << endl;
//     
//     exit(0);
}


void Particle::draw(cv::Mat& img, const uint32_t& pixelsPerCell, const double & cellSizeX, const double & cellSizeZ)
{
    const double factorX = pixelsPerCell / cellSizeX;
    const double factorZ = pixelsPerCell / cellSizeZ;
    const cv::Point2i p(m_x * factorX, m_z * factorZ);
    const cv::Point2i pSpeed((m_x + m_vx) * factorX, (m_z + m_vz) * factorZ);
    cv::line(img, p, pSpeed, cv::Scalar(0, 0, 255));
    img.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(255, 0, 0);
}

ostream& operator<<(ostream & stream, const Particle & in) {
    stream << "[" << in.x() << ", " << in.z() << ", " << in.vx() << ", " << in.vz() << "]";
    return stream;
}



}