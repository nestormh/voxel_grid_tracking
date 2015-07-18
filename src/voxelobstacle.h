/*
    Copyright 2013 Néstor Morales Hernández <email>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/


#ifndef VOXELOBSTACLE_H
#define VOXELOBSTACLE_H

#include "voxel.h"
#include <voxel_grid_tracking/roiArray.h>

#include <opencv2/opencv.hpp>
#include <image_geometry/stereo_camera_model.h>

namespace voxel_grid_tracking {

class VoxelObstacle;
typedef boost::shared_ptr<VoxelObstacle> VoxelObstaclePtr;
typedef vector<VoxelObstaclePtr> VoxelObstacleList;
    
class VoxelObstacle
{
public:
    VoxelObstacle(const uint32_t & obstIdx, const double & threshYaw, const double & threshPitch, 
                  const double & threshMagnitude, const double & minDensity, const SpeedMethod & speedMethod,
                  const double & yawInterval, const double & pitchInterval, 
                  VoxelPtr & voxel);
    VoxelObstacle(const uint32_t & obstIdx, const double & threshYaw, const double & threshPitch, 
                  const double & threshMagnitude, const double & minDensity, const SpeedMethod & speedMethod,
                  const double & yawInterval, const double & pitchInterval);
    bool addVoxelToObstacle(VoxelPtr & voxel);
    void update(const double & m_voxelSizeX, const double & m_voxelSizeY, const double & m_voxelSizeZ);
    
    double centerX() const { return m_centerX; }
    double centerY() const { return m_centerY; }
    double centerZ() const { return m_centerZ; }
    
    double minX() const { return m_minX; }
    double maxX() const { return m_maxX; }
    double minY() const { return m_minY; }
    double maxY() const { return m_maxY; }
    double minZ() const { return m_minZ; }
    double maxZ() const { return m_maxZ; }
    
    double sizeX() const { return m_sizeX; }
    double sizeY() const { return m_sizeY; }
    double sizeZ() const { return m_sizeZ; }
    
    double vx() const { return m_vx; }
    double vy() const { return m_vy; }
    double vz() const { return m_vz; }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr getTrack() { return m_track; }
    void addTrackFromObstacle(const VoxelObstaclePtr & lastObstacle);
    void startTrack();
    
    bool isInObstacle(const pcl::PointXYZ & p);
    
    void getROI(const image_geometry::StereoCameraModel & stereoCameraModel,
                const tf::StampedTransform & map2CamTransform,
                voxel_grid_tracking::roi_and_speed_2d & roi2D, 
                voxel_grid_tracking::roi_and_speed_3d & roi3D);
    
    double magnitude() const { return m_magnitude; }
    
    VoxelList voxels() const { return m_voxels; }
    
    uint32_t idx() const { return m_idx; }
    
    uint32_t numVoxels() const { return m_voxels.size(); }
    
    bool isObstacleConnected(const VoxelObstacle & obstacle);
    void joinObstacles(VoxelObstacle & obstacle);
    
    void updateSpeed(const double & egoDeltaX, const double & egoDeltaY, const double & egoDeltaZ);
    void updateSpeedFromParticles();
    void updateHistogram(const float & maxVelX, const float & maxVelY, 
                        const float & maxVelZ, const float & factorSpeed,
                        const float & minVel);
    
    static double commonVolume(const VoxelObstacle & obst1, const VoxelObstacle & obst2);
    
    string winnerNumberOfParticles() const { return m_winnerNumberOfParticles; }
    
    friend ostream& operator<<(ostream & stream, const VoxelObstacle & in);
protected:
    void updateMotionInformation();
    void updateWithVoxel(const VoxelPtr & voxel);
    geometry_msgs::Point32 toPoint32(const pcl::PointXYZRGB & point) {
        geometry_msgs::Point32 newPoint;
        newPoint.x = point.x;
        newPoint.y = point.y;
        newPoint.z = point.z;
        
        return newPoint;
    }
    voxel_grid_tracking::point_2d toPoint2D(const pcl::PointXYZRGB & point) {
        voxel_grid_tracking::point_2d newPoint;
        newPoint.u = point.x;
        newPoint.v = point.y;
        
        return newPoint;
    }
    
    VoxelList m_voxels;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_track;
    
    uint32_t m_idx;
    double m_magnitude;
    double m_yaw;
    double m_pitch;
    
    double m_minDensity;
    
    double m_vx, m_vy, m_vz;
    
    double m_yawInterval;
    double m_pitchInterval;
    
    double m_centerX, m_centerY, m_centerZ;
    double m_sizeX, m_sizeY, m_sizeZ;
    double m_minX, m_maxX, m_minY, m_maxY, m_minZ, m_maxZ;
    
    string m_winnerNumberOfParticles;
    
    // Params
    double m_threshYaw, m_threshPitch, m_threshMagnitude;
    SpeedMethod m_speedMethod;
};

}

#endif // VOXELOBSTACLE_H
