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

#include <boost/foreach.hpp>

using namespace std;

namespace polar_grid_tracking {

PolarGridTracking::PolarGridTracking(const uint32_t & rows, const uint32_t & cols, const double & cellSizeX, const double & cellSizeZ, 
                                        const t_Camera_params & cameraParams) : 
                                            m_cameraParams(cameraParams), m_grid(CellGrid(rows, cols)), m_cellSizeX(cellSizeX), m_cellSizeZ(cellSizeZ)
{
//     m_grid = CellGrid(rows, cols);
    
}

    
void PolarGridTracking::getMeasurementModelFromPointCloud(const pcl::PointCloud< pcl::PointXYZRGB >::Ptr & pointCloud)
{
    BinaryMap map;
    getBinaryMapFromPointCloud(pointCloud, map);
    
    cv::Mat binaryImg = getCvMatFromEigenBinary(map);
    cv::imshow("binaryMap", binaryImg);
    
    cv::waitKey(0);
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

    
}