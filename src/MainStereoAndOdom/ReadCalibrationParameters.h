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

#ifndef READ_CALIBRATION_PARAMETERS_H
#define READ_CALIBRATION_PARAMETERS_H

#include <string>
#include <iostream>
#include <fstream>

#include <sensor_msgs/CameraInfo.h>
#include <eigen3/Eigen/Core>

#include <opencv2/opencv.hpp>

using namespace std;

namespace sequence_reader {
    
class ReadCalibrationParameters
{
public:
    template <class T> class point2 {
    public:
        point2() : x(0), y(0) {}
        point2(T x_in, T y_in) : x(x_in), y(y_in) {}
        T x, y;
    };
    
    ReadCalibrationParameters() {}
    
    static void readETHCalibrationParams(const string & filename1, 
                                         const string & filename2, 
                                         const string & firstFrame,
                                         const string & frameId,
                                         sensor_msgs::CameraInfo& leftCameraInfo, 
                                         sensor_msgs::CameraInfo& rightCameraInfo);
protected:
    static void readETHCalibrationFile(string filename, cv::Mat & K, cv::Mat & D, 
                                       cv::Mat & R, cv::Mat & T);
    
    static void getROSCalibration(const int & width, const int & height, 
                                  const string & base_frame,
                                  const cv::Mat & K1, cv::Mat & R1, 
                                  const cv::Mat & K2, cv::Mat & R2,
                                  const cv::Mat & D1, const cv::Mat & D2,
                                  const cv::Mat & T1, const cv::Mat & T2,
                                  sensor_msgs::CameraInfo& leftCameraInfo, 
                                  sensor_msgs::CameraInfo& rightCameraInfo);
    
};
    
}
#endif