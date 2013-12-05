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
#include "obstacle.h"

#include <boost/foreach.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>

using namespace std;

namespace polar_grid_tracking {

PolarGridTracking::PolarGridTracking(const uint32_t & rows, const uint32_t & cols, const double & cellSizeX, const double & cellSizeZ, 
                                     const double & maxVelX, const double & maxVelZ, const t_Camera_params & cameraParams, 
                                     const double & particlesPerCell, const double & threshProbForCreation, 
                                     const double & gridDepthFactor, const uint32_t &  gridColumnFactor, const double & yawInterval,
                                     const double & threshYaw, const double & threshMagnitude) : 
                                            m_cameraParams(cameraParams), m_grid(CellGrid(rows, cols)), 
                                            m_cellSizeX(cellSizeX), m_cellSizeZ(cellSizeZ),
                                            m_maxVelX(maxVelX), m_maxVelZ(maxVelZ),
                                            m_particlesPerCell(particlesPerCell), m_threshProbForCreation(threshProbForCreation),
                                            m_gridDepthFactor (gridDepthFactor), m_gridColumnFactor(gridColumnFactor), 
                                            m_yawInterval(yawInterval), m_threshYaw(threshYaw), m_threshMagnitude(threshMagnitude)
{
    for (uint32_t z = 0; z < m_grid.rows(); z++) {
        for (uint32_t x = 0; x < m_grid.cols(); x++) {
            m_grid(z, x) = Cell(x, z, m_cellSizeX, m_cellSizeZ, m_maxVelX, m_maxVelZ, m_cameraParams);
        }
    }
    
    const double maxDepth = rows * m_cellSizeZ;
//     const double maxWide = cols * m_cellSizeX / 2.0;
    uint32_t numRows, numCols;
    getPolarPositionFromCartesian(maxDepth, 0, 
                                  numRows, numCols);
    numCols = m_cameraParams.width / (double)m_gridColumnFactor + 1;

    m_polarGrid = PolarCellGrid(numRows + 1, numCols);
    for (uint32_t r = 0; r < m_polarGrid.rows(); r++) {
        for (uint32_t c = 0; c < m_polarGrid.cols(); c++) {
            m_polarGrid(r, c) = PolarCell(m_yawInterval, r, c);
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
    getBinaryMapFromPointCloud(pointCloud);
    
    drawBinaryMap();
    drawTopDownMap(pointCloud);
    
    getMeasurementModel();

    if (m_initialized) {
        prediction();
        measurementBasedUpdate();
        reconstructObjects(pointCloud);
    }
    initialization();

    drawGrid(10);
    drawObstaclesMap();
}
    
void PolarGridTracking::getMeasurementModel()
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
                    totalOccupied += m_map(row, col)? 1 : 0;
                }
            }
            
            // p(m(x,z) | occupied)
            const double occupiedProb = (double)totalOccupied / ((2.0 * (double)sigmaZ + 1.0) * (2.0 * (double)sigmaX + 1.0));
            cell.setOccupiedProb(occupiedProb);
        }
    }
    
}

void PolarGridTracking::initialization() {
    for (uint32_t z = 0; z < m_grid.rows(); z++) {
        for (uint32_t x = 0; x < m_grid.cols(); x++) {
            Cell & cell = m_grid(z, x);
            
            const double & occupiedProb = cell.occupiedProb();
            if (m_map(z, x) && cell.empty() && (occupiedProb > m_threshProbForCreation)) {
                const uint32_t numParticles = m_particlesPerCell * occupiedProb / 2.0;
                cell.createParticles(numParticles);
            }
        }
    }
    
    m_initialized = true;
}

void PolarGridTracking::getBinaryMapFromPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud)
{
    m_map = BinaryMap::Zero(m_grid.rows(), m_grid.cols());

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
        
            m_map(zPos, xPos) = true;
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
            
            cell.setMainVectors();
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
         0, 0, cos(-m_deltaYaw), -sin(-m_deltaYaw),
         0, 0, sin(-m_deltaYaw), cos(-m_deltaYaw);
            
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
            cell.clearParticles();
        }
    }
    
    for (uint32_t z = 0; z < m_grid.rows(); z++) {
        for (uint32_t x = 0; x < m_grid.cols(); x++) {
            Cell & cellOrig = newGrid(z, x);
            
            const vector<Particle> & particles = cellOrig.getParticles();
            
            BOOST_FOREACH(const Particle & particle, particles) {
                const uint32_t & zPos = particle.z() / m_cellSizeZ;
                const uint32_t & xPos = particle.x() / m_cellSizeX;
                
                m_grid(zPos, xPos).addParticle(particle);
            }
        }
    }
}

void PolarGridTracking::drawGrid(const uint32_t& pixelsPerCell)
{
    cv::Mat gridImg = cv::Mat::zeros(cv::Size(m_grid.cols() * pixelsPerCell + 1, m_grid.rows() * pixelsPerCell + 1), CV_8UC3);
    
    for (uint32_t r = 0; r < m_grid.rows(); r++) {
        for (uint32_t c = 0; c < m_grid.cols(); c++) {
            if (m_map(r, c)) {
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
            if (m_map(r, c)) {
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
    
    cv::flip(gridImg, gridImg, -1);
    
    cv::imshow(ss.str(), gridImg);
}

void PolarGridTracking::drawBinaryMap()
{
    cv::Mat imgMap(cv::Size(m_map.cols(), m_map.rows()), CV_8UC1);
    
    for (uint32_t i = 0; i < m_map.rows(); i++) {
        for (uint32_t j = 0; j < m_map.cols(); j++) {
            imgMap.at<uint8_t>(i, j) = m_map(i, j)? 255 : 0;
        }
    }
    
    cv::imshow("map", imgMap);
}

void PolarGridTracking::drawTopDownMap(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud)
{
    cv::Mat imgMap = cv::Mat::zeros(cv::Size(m_grid.cols(), m_grid.rows()), CV_8UC3);
    
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
            
            imgMap.at<cv::Vec3b>(zPos, xPos) = cv::Vec3b(point.b, point.g, point.r);
        }
    }
    
    cv::Mat resizedMap;
    cv::resize(imgMap, resizedMap, cv::Size(4 * m_grid.cols(), 4 * m_grid.rows()), 0, 0, cv::INTER_NEAREST);
    
    cv::flip(imgMap, imgMap, -1);
    
    cv::imshow("TopDownMap", resizedMap);
}

void PolarGridTracking::drawObstaclesMap()
{
    cv::Mat obstacleMap = cv::Mat::zeros(cv::Size(m_polarGrid.cols(), m_polarGrid.rows()), CV_8UC3);
    BOOST_FOREACH(const Obstacle & obst, m_obstacles) {
        cv::Vec3b color(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
        const vector<PolarCell> & cells = obst.cells();
        BOOST_FOREACH(const PolarCell & cell, cells) {
            obstacleMap.at<cv::Vec3b>(cell.row(), cell.col()) = color;
        }
    }
    cv::resize(obstacleMap, obstacleMap, cv::Size(), 4.0, 4.0, cv::INTER_NEAREST);
    
    cv::imshow("obstacleMap", obstacleMap);
}

void PolarGridTracking::reconstructObjects(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud)
{
    pcl::PointCloud< PointXYZRGBDirected >::Ptr extendedPointCloud;
    resetPolarGrid();
    extendPointCloud(pointCloud, extendedPointCloud);
    
    generateObstacles();
}

void PolarGridTracking::extendPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr& pointCloud, 
                                         pcl::PointCloud< PointXYZRGBDirected >::Ptr& extendedPointCloud)
{
    extendedPointCloud.reset(new pcl::PointCloud< PointXYZRGBDirected >);
    extendedPointCloud->reserve(pointCloud->size());
    
    const double maxX = ((double)m_grid.cols() / 2.0) * (double)m_cellSizeX;
    const double minX = -maxX;
    const double maxZ = m_grid.rows() * m_cellSizeZ;
    
    const double factorX = m_grid.cols() / (maxX - minX);
    const double factorZ = m_grid.rows() / maxZ;
    
    BOOST_FOREACH(const pcl::PointXYZRGB & point, pointCloud->points) {
        if ((point.x >= minX) && (point.x <= maxX) && (point.z <= maxZ)) {
            // TODO: Check the cell each particle belongs to and assign the corresponding vectors
            PointXYZRGBDirected destPoint;
            destPoint.x = point.x;
            destPoint.y = point.y;
            destPoint.z = point.z;
            
            destPoint.r = point.r;
            destPoint.g = point.g;
            destPoint.b = point.b;
            
            const uint32_t xPos = (point.x - minX) * factorX;
            const uint32_t zPos = point.z * factorZ;
            
            const Cell & cell = m_grid(zPos, xPos);
            double vx, vz;
            cell.getMainVectors(vx, vz);
            destPoint.vx = vx;
//             destPoint.vy = 0.0;
            destPoint.vz = vz;
            // TODO: Get orientation and magnitude
            
            destPoint.yaw = 0.0;
            destPoint.magnitude = 0.0;
            if ((vx != 0) || (vz != 0)) {
                destPoint.magnitude = sqrt(vx * vx + vz * vz);
                
                destPoint.yaw = acos(destPoint.vx / destPoint.magnitude);
                if (destPoint.vz < 0)
                    destPoint.yaw = -destPoint.yaw;
            }
            
            updatePolarGridWithPoint(destPoint);
            
            extendedPointCloud->push_back(destPoint);
            
        }
    }
}

void PolarGridTracking::growFromList(Obstacle & obstacle, deque<t_visitInfo> &candidates, 
                                     t_visitMatrix & assigned, t_visitMatrix & addedToList) 
{
    while (candidates.size() != 0) {
        t_visitInfo vi = candidates.front();
        candidates.pop_front();
        PolarCell & cell = m_polarGrid(vi.row, vi.col);
        
        const bool added = obstacle.addCellToObstacle(cell);
        if (added) {
            assigned(cell.row(), cell.col()) = true;
        
            for (uint32_t r1 = max(cell.row() - 1, (uint32_t)0); r1 <= min(cell.row() + 1, (uint32_t)m_polarGrid.rows() - 1); r1++) {
                for (uint32_t c1 = max(cell.col() - 1, (uint32_t)0); c1 <= min(cell.col() + 1, (uint32_t)m_polarGrid.cols() - 1); c1++) {
                    if ((r1 != cell.row()) || (c1 != cell.col())) {
                        if (m_polarGrid(r1, c1).getNumVectors() != 0) {
                            if ((! addedToList(r1, c1)) && (! assigned(r1, c1))) {
//                                 if (m_polarGrid(r1, c1).obstIdx() == -1) {
                                    candidates.push_back({r1, c1});
                                    addedToList(r1, c1) = true;
//                                 }
                            }
                        }
                    }
                }
            }
        }
    }
}

void PolarGridTracking::generateObstacles()
{
    for (uint32_t r = 0; r < m_polarGrid.rows(); r++) {
        for (uint32_t c = 0; c < m_polarGrid.cols(); c++) {
            m_polarGrid(r, c).updateYawAndMagnitude();
        }
    }
    
    m_obstacles.clear();
    
    t_visitMatrix assigned(m_polarGrid.rows(), m_polarGrid.cols());
    t_visitMatrix addedToList(m_polarGrid.rows(), m_polarGrid.cols());
    assigned.setConstant(false);
    
    for (uint32_t r = 0; r < m_polarGrid.rows(); r++) {
        for (uint32_t c = 0; c < m_polarGrid.cols(); c++) {
            
            addedToList = assigned;
            
            PolarCell & cell = m_polarGrid(r, c);
            
            deque<t_visitInfo> candidates;
            
            if (! assigned(r, c)) {
                if (cell.getNumVectors() != 0) {
                    if (cell.obstIdx() == -1) {
                        if (! addedToList(r, c)) {
                            uint32_t obstIdx = m_obstacles.size();
                            Obstacle obstacle(obstIdx, m_threshYaw, m_threshMagnitude);
                            
                            candidates.push_back({r, c});
                            addedToList(r, c) = true;
                            
                            growFromList(obstacle, candidates, assigned, addedToList);
                            
                            obstacle.setROIAndMotion(m_cameraParams, m_gridDepthFactor, m_gridColumnFactor, m_yawInterval);
                            
                            m_obstacles.push_back(obstacle);
                        }
                    }
                }
            }
        }
    }
    
}

void PolarGridTracking::getPolarPositionFromCartesian(const double & z, const double & x, 
                                                         uint32_t& row, uint32_t& column)
{
    const double z0 = (double)(m_cameraParams.ku * m_cameraParams.baseline) / m_cameraParams.width;
    
    row = LOG_BASE(1.0 + m_gridDepthFactor, z / z0);
    const double u = m_cameraParams.u0 - ((x * m_cameraParams.ku) / z);
    column = u / m_gridColumnFactor - 1;
}

void PolarGridTracking::resetPolarGrid()
{for (uint32_t r = 0; r < m_polarGrid.rows(); r++) {
        for (uint32_t c = 0; c < m_polarGrid.cols(); c++) {
            m_polarGrid(r, c).reset();
        }
    }
}

void PolarGridTracking::updatePolarGridWithPoint(const PointXYZRGBDirected& point)
{
    uint32_t row, column;
    getPolarPositionFromCartesian(point.z, point.x, row, column);
    m_polarGrid(row, column).addPointToHistogram(point);
}
    
}