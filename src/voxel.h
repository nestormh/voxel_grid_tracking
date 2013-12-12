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

namespace voxel_grid_tracking {

class Voxel
{

public:
    Voxel();
    Voxel(const double& x, const double& z, const double& sizeX, const double& sizeZ, const double& maxVelX, const double& maxVelZ, const polar_grid_tracking::t_Camera_params& params);
    
    void createParticles(const uint32_t & numParticles);
    
    void setOccupiedProb(const double & occupiedProb) { m_occupiedProb = occupiedProb; }
    void setOccupiedPosteriorProb(const uint32_t & particlesPerCell);
    
    double sigmaX() { return m_sigmaX; } 
    double sigmaZ() { return m_sigmaZ; }
    
    double occupiedProb() { return m_occupiedProb; }
    double occupiedPosteriorProb() { return m_occupiedPosteriorProb; }
    double freeProb() { return 1.0 - m_occupiedProb; }
    
    uint32_t numParticles() const { return m_particles.size(); }
    Particle3d getParticle(const uint32_t & idx) const { return m_particles.at(idx); }
    vector <Particle3d> getParticles() { return m_particles; }
    bool empty() { return m_particles.size() == 0; }
    void makeCopy(const Particle3d & particle);
    void addParticle(const Particle3d& particle);
    void removeParticle(const uint32_t & idx) { m_particles.erase(m_particles.begin() + idx); }
    void transformParticles(const Eigen::Matrix4d & R, const Eigen::Vector4d & t, const Eigen::Matrix4d & stateTransition, CellGrid & newGrid);
    void clearParticles() { m_particles.clear(); }
    void setParticles(const vector <Particle3d> & particles) { m_particles = particles; }
    
    void draw(cv::Mat & img, const uint32_t & pixelsPerCell);
    void drawParticles(cv::Mat& img, const uint32_t & pixelsPerCell);
    
    void setMainVectors();
    void getMainVectors(double & vx, double & vz) const { vx = m_vx; vz = m_vz; }
    
protected:
    //     double getAvgDir(double & vx, double & vz);
    
    double m_x, m_z;
    double m_sigmaX, m_sigmaZ;
    double m_sizeX, m_sizeZ;
    double m_maxVelX, m_maxVelZ;
    
    double m_occupiedProb;
    double m_occupiedPosteriorProb;
    
    double m_vx, m_vz;
    
    vector <Particle> m_particles;
};

}

#endif // VOXEL_H
