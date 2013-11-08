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

#ifndef LIBVISOHELPER_H
#define LIBVISOHELPER_H

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

#include "libviso2/viso_stereo.h"
#include "params_structs.h"

namespace polar_grid_tracking {

class LibvisoHelper
{
public:
    LibvisoHelper(const t_Camera_params & cameraParams);
    
    bool compute(const cv::Mat& imgLeft, const cv::Mat& imgRight, const double & deltaTime,
                 double & yaw, double & speed);
protected:
    boost::shared_ptr<VisualOdometryStereo> m_viso;
};

}

#endif // LIBVISOHELPER_H
