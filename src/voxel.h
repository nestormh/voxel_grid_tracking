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


#ifndef VOXEL_H
#define VOXEL_H

#include "params_structs.h"
#include "particle3d.h"

#include <boost/multi_array.hpp>

#include <pcl_ros/point_cloud.h>

#include <opencv2/opencv.hpp>

#include <vector>

using namespace std;

namespace voxel_grid_tracking {

class Voxel;
typedef boost::multi_array<Voxel, 3> VoxelGrid;
typedef VoxelGrid::index voxelIdx;
    
class Voxel
{
public:
    Voxel();
    Voxel(const double & x, const double & y, const double & z, 
          const double & centroidX, const double & centroidY, const double & centroidZ, 
          const double & sizeX, const double & sizeY, const double & sizeZ, 
          const double & maxVelX, const double & maxVelY, const double & maxVelZ,
          const polar_grid_tracking::t_Camera_params & params);
    
    void createParticles(const uint32_t & numParticles);
    
    void setOccupiedProb(const double & occupiedProb) { m_occupiedProb = occupiedProb; }
    void setOccupiedPosteriorProb(const uint32_t & particlesPerVoxel);
    
    double sigmaX() { return m_sigmaX; } 
    double sigmaY() { return m_sigmaY; } 
    double sigmaZ() { return m_sigmaZ; }
    
    double occupiedProb() const { return m_occupiedProb; }
    double occupiedPosteriorProb() const { return m_occupiedPosteriorProb; }
    double freeProb() { return 1.0 - m_occupiedProb; }
    
    uint32_t numParticles() const { return m_particles.size(); }
    Particle3d getParticle(const uint32_t & idx) const { return m_particles.at(idx); }
    vector <Particle3d> getParticles() { return m_particles; }
    bool empty() const { return m_particles.size() == 0; }
    void makeCopy(const Particle3d & particle);
    void addParticle(const Particle3d& particle);
    void removeParticle(const uint32_t & idx) { m_particles.erase(m_particles.begin() + idx); }
    void transformParticles(const Eigen::Matrix4d & R, const Eigen::Vector4d & t, const Eigen::MatrixXd & stateTransition, VoxelGrid & newGrid);
    void clearParticles() { m_particles.clear(); }
    void setParticles(const vector <Particle3d> & particles) { m_particles = particles; }
    
    void setMainVectors();
    void getMainVectors(double & vx, double & vy, double & vz) const { vx = m_vx; vy = m_vy; vz = m_vz; }
    
    void addPoint(const pcl::PointXYZRGB & point);
    bool occupied() const { return m_pointCloud->size() > 0; }
    
    void update();
    
    double centroidX() const { return m_centroidX; }
    double centroidY() const { return m_centroidY; }
    double centroidZ() const { return m_centroidZ; }
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPoints() const { return m_pointCloud; }
    
    void reset();
    
protected:
    double m_x, m_y, m_z;
    double m_sigmaX, m_sigmaY, m_sigmaZ;
    double m_sizeX, m_sizeY, m_sizeZ;
    double m_maxVelX, m_maxVelY, m_maxVelZ;
    
    double m_occupiedProb;
    double m_occupiedPosteriorProb;
    
    double m_vx, m_vy, m_vz;
    double m_centroidX, m_centroidY, m_centroidZ;
    
    vector <Particle3d> m_particles;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pointCloud;
};

}
#endif // CELL_H