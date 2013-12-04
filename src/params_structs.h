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


#ifndef PARAMS_STRUCTS_H
#define PARAMS_STRUCTS_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <stdint.h>
#include <Eigen/Core>

namespace polar_grid_tracking {
    
typedef struct {        
    uint32_t minX, minY;
    uint32_t width, height;
    double u0, v0;
    double ku, kv;
    double distortion;
    double baseline;
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> R;
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> t;
} t_Camera_params;

typedef struct {        
    int minDisparity;
    int numDisparities;
    int SADWindowSize; 
    int P1; 
    int P2; 
    int disp12MaxDiff; 
    int preFilterCap; 
    int uniquenessRatio;
    
    int speckleWindowSize; 
    int speckleRange; 
    bool fullDP;
} t_SGBM_params;


typedef struct {
    double deltaYaw;
    double deltaPos;
    double deltaTime;
    double speed;
} t_ego_value;

typedef struct {
    uint32_t numPoints;
    double magnitudeSum;
} t_histogram;

// using namespace pcl;
// 
struct PointXYZRGBDirected
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
//     union
//     {
//         struct
//         {
//             float rgb;
//         };
//         float data_c[4];
//     };
    PCL_ADD_RGB;
    union
    {
        float vData[4];
        struct
        {
            float vx;
//             float vy;
            float vz;
            float yaw;
            float magnitude;
        };
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

}

POINT_CLOUD_REGISTER_POINT_STRUCT (polar_grid_tracking::PointXYZRGBDirected,          
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, r, r)
                                   (float, g, g)
                                   (float, b, b)
                                   (float, vx, vx)
                                   (float, vz, vz)
                                   (float, yaw, yaw)
                                   (float, magnitude, magnitude)
)

#endif // PARAMS_STRUCTS_H