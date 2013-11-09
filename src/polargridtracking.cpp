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


#include "polargridtracking.h"
#include "utils.h"
#include </home/nestor/Dropbox/projects/GPUCPD/src/LU-Decomposition/Libs/Cuda/include/device_launch_parameters.h>

#include <boost/foreach.hpp>

using namespace std;

namespace polar_grid_tracking {

PolarGridTracking::PolarGridTracking(const uint32_t & rows, const uint32_t & cols, const double & cellSizeX, const double & cellSizeZ, 
                                     const t_Camera_params & cameraParams, 
                                     const double & particlesPerCell, const double & threshProbForCreation) : 
                                            m_cameraParams(cameraParams), m_grid(CellGrid(rows, cols)), 
                                            m_cellSizeX(cellSizeX), m_cellSizeZ(cellSizeZ),
                                            m_particlesPerCell(particlesPerCell), m_threshProbForCreation(threshProbForCreation)
{
    for (uint32_t z = 0; z < m_grid.rows(); z++) {
        for (uint32_t x = 0; x < m_grid.cols(); x++) {
            m_grid(z, x) = Cell(x, z, m_cellSizeX, m_cellSizeZ, m_cameraParams);
        }
    }
    
    m_initialized = false;
    
}

void PolarGridTracking::setDeltaYawSpeedAndTime(const double& deltaYaw, const double& deltaSpeed, const double& deltaTime)
{
    m_deltaYaw = deltaYaw;
    m_deltaSpeed = deltaSpeed;
    m_deltaTime = deltaTime;
}

void PolarGridTracking::compute(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr & pointCloud) {
    BinaryMap map;
    getBinaryMapFromPointCloud(pointCloud, map);
    getMeasurementModel(map);

    if (m_initialized) {
        prediction();
        measurementBasedUpdate();
    }
    
    initialization(map);

    drawGrid(20, map);
    
    
    
/*    BinaryMap map = BinaryMap::Zero(m_grid.rows(), m_grid.cols());
    for (uint32_t i = 0; i < map.rows(); i++) {
        for (uint32_t j = 0; j < map.cols(); j++) {
            map(i, j) = true;
        }
    }

    if (m_initialized) {
        cout << "Predicting " << endl;
        prediction();
    }
    m_initialized = true;
    
    m_grid(10, 20).addParticle(Particle(2.05, 1.05, 0.5, 0.5, false));

    drawGrid(20, map);*/    
}
    
void PolarGridTracking::getMeasurementModel(const BinaryMap & map)
{
    
// #pragma omp for schedule(dynamic)
    for (uint32_t z = 0; z < m_grid.rows(); z++) {
        for (uint32_t x = 0; x < m_grid.cols(); x++) {
            Cell & cell = m_grid(z, x);
            const int sigmaX = cell.sigmaX();
            const int sigmaZ = cell.sigmaZ();
            
            uint32_t totalOccupied = 0;
            for (uint32_t row = max(0, (int)(z - sigmaZ)); row <= min((int)(m_grid.rows() - 1), (int)(z + sigmaZ)); row++) {
                for (uint32_t col = max(0, (int)(x - sigmaX)); col <= min((int)(m_grid.cols() - 1), (int)(x + sigmaX)); col++) {
                    totalOccupied += map(row, col)? 1 : 0;
                }
            }
            
            // p(m(x,z) | occupied)
            const double occupiedProb = (double)totalOccupied / ((2.0 * (double)sigmaZ + 1.0) * (2.0 * (double)sigmaX + 1.0));
            cell.setOccupiedProb(occupiedProb);
        }
    }
    
}

void PolarGridTracking::initialization(const BinaryMap & map) {
    for (uint32_t z = 0; z < m_grid.rows(); z++) {
        for (uint32_t x = 0; x < m_grid.cols(); x++) {
            Cell & cell = m_grid(z, x);
            
            const double & occupiedProb = cell.occupiedProb();
            if (map(z, x) && cell.empty() && (occupiedProb > m_threshProbForCreation)) {
                const uint32_t numParticles = m_particlesPerCell * occupiedProb / 2.0;
                cell.createParticles(numParticles);
            }
        }
    }
    
    m_initialized = true;
}

void PolarGridTracking::getBinaryMapFromPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud, 
                                                   BinaryMap& map)
{
    map = BinaryMap::Zero(m_grid.rows(), m_grid.cols());

    const double maxZ = m_grid.rows() * m_cellSizeZ;
    const double maxX = m_grid.cols() / 2.0 * m_cellSizeX;
    const double minX = -maxX;
    
    const double factorX = m_grid.cols() / (maxX - minX);
    const double factorZ = m_grid.rows() / maxZ;
    
    BOOST_FOREACH(pcl::PointXYZRGB& point, *pointCloud) {
        
        const uint32_t xPos = (point.x - minX) * factorX;
        const uint32_t zPos = point.z * factorZ;
        
        if ((xPos > 0) && (xPos < m_grid.cols()) && 
            (zPos > 0) && (zPos < m_grid.rows())) {
        
            map(zPos, xPos) = true;
        }
    }
}

void PolarGridTracking::measurementBasedUpdate()
{
    for (uint32_t z = 0; z < m_grid.rows(); z++) {
        for (uint32_t x = 0; x < m_grid.cols(); x++) {
            Cell & cell = m_grid(z, x);
            
            if (cell.numParticles() != 0) {
                cell.setOccupiedPosteriorProb(m_particlesPerCell);
                const double Nrc = cell.occupiedPosteriorProb() * m_particlesPerCell;
                const double fc = Nrc / cell.numParticles();
                
                if (fc > 1.0) {
                    const double Fn = floor(fc);       // Integer part
                    const double Ff = fc - Fn;         // Fractional part

                    const uint32_t numParticles = cell.numParticles();
                    for (uint32_t i = 0; i < numParticles; i++) {

                        const Particle & p = cell.getParticle(i);

                        for (uint32_t k = 1; k < Fn; k++)
                            cell.makeCopy(p);

                        const double r = (double)rand() / (double)RAND_MAX;
                        if (r < Ff)
                            cell.makeCopy(p);
                    }
                } else if (fc < 1.0) {
                    for (uint32_t i = 0; i < cell.numParticles(); i++) {
                        const double r = (double)rand() / (double)RAND_MAX;
                        if (r > fc)
                            cell.removeParticle(i);
                    }
                }
            }
        }
    }
}

void PolarGridTracking::prediction()
{
    const double dx = m_deltaSpeed * m_deltaTime * cos(m_deltaYaw); // / m_cellSizeX;
    const double dz = m_deltaSpeed * m_deltaTime * sin(m_deltaYaw); // / m_cellSizeZ;
    
    Eigen::Matrix4d R;
    Eigen::Vector4d t;
    Eigen::Matrix4d stateTransition;
    R << cos(m_deltaYaw), -sin(m_deltaYaw), 0, 0,
         sin(m_deltaYaw), cos(m_deltaYaw), 0, 0,
         0, 0, sin(-m_deltaYaw), cos(-m_deltaYaw),
         0, 0, cos(-m_deltaYaw), -sin(-m_deltaYaw);
            
    t << dx, dz, 0, 0;
    stateTransition << 1, 0, m_deltaTime, 0,
                       0, 1, 0, m_deltaTime,
                       0, 0, 1, 0,
                       0, 0, 0, 1;
                       
    // TODO: Put correct values for deltaX, deltaZ, deltaVX, deltaVZ in class Particle, based on the covariance matrix
    CellGrid newGrid(m_grid.rows(), m_grid.cols());
    for (uint32_t z = 0; z < m_grid.rows(); z++) {
        for (uint32_t x = 0; x < m_grid.cols(); x++) {
            Cell & cell = m_grid(z, x);
            cell.transformParticles(R, t, stateTransition, newGrid);
        }
    }
    
    for (uint32_t z = 0; z < m_grid.rows(); z++) {
        for (uint32_t x = 0; x < m_grid.cols(); x++) {
            Cell & cellOrig = newGrid(z, x);
            Cell & cellDest = m_grid(z, x);
            
            const vector<Particle> & particles = cellOrig.getParticles();
            
            cellDest.setParticles(particles);
        }
    }
}

void PolarGridTracking::drawGrid(const uint32_t& pixelsPerCell, const BinaryMap & binaryMap)
{
    cv::Mat gridImg = cv::Mat::zeros(cv::Size(m_grid.cols() * pixelsPerCell + 1, m_grid.rows() * pixelsPerCell + 1), CV_8UC3);
    
    for (uint32_t r = 0; r < m_grid.rows(); r++) {
        for (uint32_t c = 0; c < m_grid.cols(); c++) {
            if (binaryMap(r, c)) {
                cv::rectangle(gridImg, cv::Point2i(c * pixelsPerCell, r * pixelsPerCell), cv::Point2i((c + 1) * pixelsPerCell, (r + 1) * pixelsPerCell), cv::Scalar(0, 255, 0));
                m_grid(r,c).draw(gridImg, pixelsPerCell);
            }
        }
    }
    
    
    
    for (uint32_t r = 0; r < m_grid.rows(); r++) {
        for (uint32_t c = 0; c < m_grid.cols(); c++) {
            cv::line(gridImg, cv::Point2i(c * pixelsPerCell, 0), cv::Point2i(c * pixelsPerCell, gridImg.rows - 1), cv::Scalar(0, 255, 255));
        }
        cv::line(gridImg, cv::Point2i(0, r * pixelsPerCell), cv::Point2i(gridImg.cols - 1, r * pixelsPerCell), cv::Scalar(0, 255, 255));
    }
    cv::line(gridImg, cv::Point2i(gridImg.cols - 1, 0), cv::Point2i(gridImg.cols - 1, gridImg.rows - 1), cv::Scalar(0, 255, 255));
    
    for (uint32_t r = 0; r < m_grid.rows(); r++) {
        for (uint32_t c = 0; c < m_grid.cols(); c++) {
            if (binaryMap(r, c)) {
                m_grid(r,c).drawParticles(gridImg, pixelsPerCell);
            }
        }
    }
    
    const cv::Point2i center(gridImg.cols/ 2.0, 0);
    const cv::Point2i direction(center.x + 100.0 * sin(m_deltaYaw), center.y + 100.0 * cos(m_deltaYaw));
    cv::line(gridImg, center, direction, cv::Scalar(0, 0, 255), 5);
    
    
    stringstream ss;
//     ss << "grid" << rand();
    ss << "grid";
    
    cv::flip(gridImg, gridImg, 0);
    
    cv::imshow(ss.str(), gridImg);
}

void PolarGridTracking::drawBinaryMap(const BinaryMap& map)
{
    cv::Mat imgMap(cv::Size(map.cols(), map.rows()), CV_8UC1);
    
    for (uint32_t i = 0; i < map.rows(); i++) {
        for (uint32_t j = 0; j < map.cols(); j++) {
            imgMap.at<uint8_t>(i, j) = map(i, j)? 255 : 0;
        }
    }
    
    cv::imshow("map", imgMap);
}

    
}