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

#include "cell.h"

#include <iostream>
#include <boost/foreach.hpp>


using namespace std;

namespace polar_grid_tracking {
    
// FIXME: Use a bigger value after implementing this stage over the GPU
#define DISPARITY_COMPUTATION_ERROR 0.075//0.075

Cell::Cell() 
{
    
}

Cell::Cell(const double & x, const double & z, const double & sizeX, const double & sizeZ, const double & maxVelX, const double & maxVelZ,
           const t_Camera_params & params) : 
                m_x(x), m_z(z), m_sizeX(sizeX), m_sizeZ(sizeZ), m_maxVelX(maxVelX), m_maxVelZ(maxVelZ)
{
    if ((z == 0) || (x == 0)) {
        m_sigmaX = 0.0;
        m_sigmaZ = 0.0;
    } else {
        double xReal = m_x * m_sizeX;
        double zReal = m_z * m_sizeZ;
        
        const double sigmaZ = (zReal * zReal * DISPARITY_COMPUTATION_ERROR) / (params.baseline * params.ku);
        const double sigmaX = (xReal * sigmaZ) / zReal;
        
        m_sigmaZ = sigmaZ / m_sizeZ;
        m_sigmaX = sigmaX / m_sizeX;
    }
}

void Cell::createParticles(const uint32_t & numParticles)
{
    for (uint32_t i = m_particles.size(); i < numParticles; i++)
        m_particles.push_back(Particle(m_x, m_z, m_sizeX, m_sizeZ, m_maxVelX, m_maxVelZ));
}

void Cell::setOccupiedPosteriorProb(const uint32_t& particlesPerCell)
{
    if (m_occupiedProb == 0) {
        m_occupiedPosteriorProb = 0;
    } else {
        const double weightedOccupied = m_occupiedProb * m_particles.size();
        const double freeProb = 1.0 - m_occupiedProb;
        const uint32_t numFree = particlesPerCell - m_particles.size();
        const double weightedFree = freeProb * numFree;
        
        m_occupiedPosteriorProb = weightedOccupied / (weightedOccupied + weightedFree);
    }
    
}

void Cell::makeCopy(const Particle& particle)
{
    Particle newParticle(particle);
    m_particles.push_back(newParticle);
}

void Cell::addParticle(const Particle& particle)
{
    m_particles.push_back(particle);
}
    
void Cell::transformParticles(const Eigen::Matrix4d & R, const Eigen::Vector4d & t, const Eigen::Matrix4d & stateTransition, CellGrid & newGrid)
{
    BOOST_FOREACH(Particle & particle, m_particles) {
        particle.transform(R, t, stateTransition);
        
        const int newCellZ = particle.z() / m_sizeZ;
        const int newCellX = particle.x() / m_sizeX;
        
        if ((newCellZ >= 0) && (newCellZ < newGrid.rows()) &&
            (newCellX >= 0) && (newCellX < newGrid.cols())) {
                
            newGrid(newCellZ, newCellX).addParticle(particle);
        }
    }
    m_particles.clear();
}

void Cell::setMainVectors() {
    m_vx = 0.0;
    m_vz = 0.0;
    if (m_particles.size() != 0) {
        BOOST_FOREACH(Particle particle, m_particles) {
            m_vx += particle.vx();
            m_vz += particle.vz();
        }
    
        m_vx /= m_particles.size();
        m_vz /= m_particles.size();
    }
}

void Cell::draw(cv::Mat& img, const uint32_t & pixelsPerCell)
{
    const uint32_t color = m_occupiedProb * 255;
    cv::rectangle(img, cv::Point2i(m_x * pixelsPerCell + 1, m_z * pixelsPerCell + 1), cv::Point2i((m_x + 1) * pixelsPerCell - 1, (m_z + 1) * pixelsPerCell - 1), cv::Scalar::all(color), -1);
}

void Cell::drawParticles(cv::Mat& img, const uint32_t & pixelsPerCell)
{
    BOOST_FOREACH(Particle particle, m_particles) {
        particle.draw(img, pixelsPerCell, m_sizeX, m_sizeZ);
    }
    
    if (m_particles.size() > 0) {
        double vx, vz;
        getMainVectors(vx, vz);
        const double factorX = pixelsPerCell / m_sizeX;
        const double factorZ = pixelsPerCell / m_sizeZ;
        cv::Point2i center(m_x * pixelsPerCell + pixelsPerCell / 2.0, m_z * pixelsPerCell + pixelsPerCell / 2.0);
        cv::Point2i mainDirection(center.x + vx * factorX, center.y + vz * factorZ);
        cv::line(img, center, mainDirection, cv::Scalar(255, 255, 0));
    }
}

}