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


#include "libvisohelper.h"

using namespace std;

#define AT(matrix, row, col) matrix.val[row * matrix.n + col]

namespace polar_grid_tracking {
    
LibvisoHelper::LibvisoHelper(const t_Camera_params& cameraParams)
{
    VisualOdometryStereo::parameters params;
    params.calib.f  = cameraParams.ku; // focal length in pixels
    params.calib.cu = cameraParams.u0; // principal point (u-coordinate) in pixels
    params.calib.cv = cameraParams.v0; // principal point (v-coordinate) in pixels
    params.base     = cameraParams.baseline; // baseline in meters
    
    m_viso.reset(new VisualOdometryStereo(params));
}

bool LibvisoHelper::compute(const cv::Mat& imgLeft, const cv::Mat& imgRight, const double & deltaTime,
                            double & yaw, double & speed)
{
    Matrix pose = Matrix::eye(4);
    uint8_t* left_img_data = imgLeft.data;
    uint8_t* right_img_data = imgRight.data;
    
    const int32_t & width  = imgLeft.cols;
    const int32_t & height = imgLeft.rows;
    
    int32_t dims[] = {width,height,width};
    if (m_viso->process(left_img_data,right_img_data,dims)) {
        Matrix motion = Matrix::inv(m_viso->getMotion());
        
        yaw = -asin(motion.val[2][0]);
        
        // v = s / t
        const double s = sqrt(motion.val[0][3] * motion.val[0][3] + 
                              motion.val[1][3] * motion.val[1][3] + 
                              motion.val[2][3] * motion.val[2][3]);
        speed = s / deltaTime;        
        
        return true;
    } else {
        return false;
    }
}


    
}