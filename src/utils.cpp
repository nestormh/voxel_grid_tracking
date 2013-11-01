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

#include "utils.h"

cv::Mat getCvMatFromEigenBinary(const polar_grid_tracking::BinaryMap & map) {
    
    cv::Mat img(cv::Size(map.cols(), map.rows()), CV_8UC1);
    
    for (uint32_t i = 0; i < map.rows(); i++) {
        for (uint32_t j = 0; j < map.cols(); j++) {
            img.at<uint8_t>(i, j) = map(i, j)? 255 : 0;
        }
    }
    
    return img;
}