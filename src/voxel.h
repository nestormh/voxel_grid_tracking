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

#include <image_geometry/stereo_camera_model.h>

#include <vector>
#include <tiff.h>

using namespace std;

namespace voxel_grid_tracking {

class Voxel;
typedef boost::shared_ptr<Voxel> VoxelPtr;
typedef boost::multi_array<VoxelPtr, 3> VoxelGrid;
typedef std::vector< VoxelPtr > VoxelList;
typedef VoxelGrid::index voxelIdx;

typedef boost::shared_ptr<Particle3d> ParticlePtr;
typedef vector <ParticlePtr> ParticleList;

// FIXME: Do I really need a polar_grid_tracking::t_histogram for this?
typedef boost::multi_array<polar_grid_tracking::t_histogram, 4> SpeedHistogram;
    
class Voxel
{
public:
    Voxel();
    Voxel(const double & x, const double & y, const double & z, 
          const double & centroidX, const double & centroidY, const double & centroidZ, 
          const double & sizeX, const double & sizeY, const double & sizeZ, 
          const double & maxVelX, const double & maxVelY, const double & maxVelZ,
          const image_geometry::StereoCameraModel & stereoCameraModel, const SpeedMethod & speedMethod,
          const double & yawInterval, const double & pitchInterval, const float & factorSpeed);
    
    void createParticles(const uint32_t & numParticles, const tf::StampedTransform & pose2mapTransform);
    ParticleList createParticlesStatic(const tf::StampedTransform & pose2mapTransform);
    ParticleList createParticlesFromOFlow(const uint32_t & numParticles);
    
    void setOccupiedProb(const double & occupiedProb) { m_occupiedProb = occupiedProb; }
    void setOccupiedPosteriorProb(const uint32_t & particlesPerVoxel);
    
    bool nextTo(const Voxel & voxel) const;
    
    double sigmaX() { return m_sigmaX; } 
    double sigmaY() { return m_sigmaY; } 
    double sigmaZ() { return m_sigmaZ; }
    
    double occupiedProb() const { return m_occupiedProb; }
    double occupiedPosteriorProb() const { return m_occupiedPosteriorProb; }
    double freeProb() { return 1.0 - m_occupiedProb; }
    
    uint32_t numParticles() const { return m_particles.size(); }
    ParticlePtr getParticle(const uint32_t & idx) const { return m_particles.at(idx); }
    ParticleList getParticles() const { return m_particles; }
    
    uint32_t numOFlowParticles() const { return m_oFlowParticles.size(); }
    ParticleList getOFlowParticles() { return m_oFlowParticles; }
    
    bool empty() const { return m_particles.size() == 0; }
    void makeCopy(const ParticlePtr & particle);
    void addParticle(const ParticlePtr & particle);
    void addFlowParticle(const ParticlePtr& particle);
    void removeParticle(const uint32_t & idx) { m_particles.erase(m_particles.begin() + idx); }
    void transformParticles(const Eigen::MatrixXd & stateTransition, ParticleList & newParticles);
    void clearParticles() { m_particles.clear(); }
    void setParticles(const ParticleList & particles) { m_particles = particles; }
    
    void setMainVectors(const double & deltaEgoX, const double & deltaEgoY, const double & deltaEgoZ);
    void getMainVectors(double & vx, double & vy, double & vz) const { vx = m_vx; vy = m_vy; vz = m_vz; }
    
    void updateHistogram();
    
    void addPoint(const pcl::PointXYZRGB & point);
    bool occupied() const { return m_occupied; }
    
    void update();
    
    void sortParticles();
    void joinParticles();
    void reduceParticles(const uint32_t & maxNumberOfParticles);
    void centerParticles();
    
    double centroidX() const { return m_centroidX; }
    double centroidY() const { return m_centroidY; }
    double centroidZ() const { return m_centroidZ; }
    
    double magnitude() const { return m_magnitude; }
    double yaw() const { return m_yaw; }
    double pitch() const { return m_pitch; }
    
    double vx() const { return m_vx; }
    double vy() const { return m_vy; }
    double vz() const { return m_vz; }
    
    double x() const { return m_x; }
    double y() const { return m_y; }
    double z() const { return m_z; }
    
    double sizeX() const { return m_sizeX; }
    double sizeY() const { return m_sizeY; }
    double sizeZ() const { return m_sizeZ; }
    
    uint32_t oldestParticle() { return m_oldestParticle; }
    
    uint32_t neighborOcc() const { return m_neighborOcc; };
    
    void incNeighborOcc() { m_neighborOcc++; };
    
    int32_t obstIdx() const { return m_obstIdx; }
    
    bool m_occupied;
    
    bool assignedToObstacle() { return m_obstIdx != -1; }
    
    void assignObstacle(const int32_t & obstIdx) { m_obstIdx = obstIdx; }
    
    void reset();
    
    friend ostream& operator<<(ostream & stream, const Voxel & in);
    
protected:
    double m_x, m_y, m_z;
    double m_sigmaX, m_sigmaY, m_sigmaZ;
    double m_sizeX, m_sizeY, m_sizeZ;
    double m_maxVelX, m_maxVelY, m_maxVelZ;
    
    double m_occupiedProb;
    double m_occupiedPosteriorProb;
    
    double m_vx, m_vy, m_vz;
    double m_centroidX, m_centroidY, m_centroidZ;
    double m_magnitude;
    double m_yaw, m_pitch;
    
    double m_yawInterval;
    double m_pitchInterval;
    
    float m_factorSpeed;
    
    uint32_t m_oldestParticle;
    
    SpeedMethod m_speedMethod;
    
    int32_t m_obstIdx;
    
    uint32_t m_neighborOcc;                     // Number of neighbors containing at least one point
    
    ParticleList m_particles;
    ParticleList m_oFlowParticles;

    SpeedHistogram m_speedHistogram;
};

}
#endif // CELL_H