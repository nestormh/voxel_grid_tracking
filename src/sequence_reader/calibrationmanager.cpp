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

#include "calibrationmanager.h"

#include <eigen3/Eigen/Dense>

#include <sensor_msgs/distortion_models.h>

using namespace std;

namespace sequence_reader {

void CalibrationManager::readETHZCalibrationParams(const string & filename1, 
                                                         const string & filename2, 
                                                         const string & firstFrame,
                                                         const string & frameId,
                                                         sensor_msgs::CameraInfo& leftCameraInfo, 
                                                         sensor_msgs::CameraInfo& rightCameraInfo)
{
    cv::Mat K1, R1, K2, R2;
    cv::Mat D1, D2;
    cv::Mat T1, T2;
    
    readETHZCalibrationFile(filename1, K1, D1, R1, T1);
    readETHZCalibrationFile(filename2, K2, D2, R2, T2);
    
    cv::Mat frame = cv::imread(firstFrame.c_str());
    getROSCalibration(frame.cols, frame.rows, frameId, K1, R1, K2, R2, D1, D2, T1, T2, leftCameraInfo, rightCameraInfo);
}

void CalibrationManager::readETHZCalibrationFile(string filename, cv::Mat& K, cv::Mat& D, cv::Mat& R, cv::Mat& T)
{
    ifstream f(filename.c_str());
    
    K = cv::Mat(3, 3, CV_64FC1);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float val;
            f >> val;
            K.at<double>(i, j) = val; 
        }
    }
    
    D = cv::Mat::zeros(5, 1, CV_64FC1);
    for (int i = 0; i < 4; i++) {
        float val;
        f >> val;
        D.at<double>(i, 0) = val; 
    }
    
    R = cv::Mat(3, 3, CV_64FC1);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float val;
            f >> val;
            R.at<double>(i, j) = val; 
        }
    }
    
    T = cv::Mat(3, 1, CV_64FC1);
    for (int i = 0; i < 3; i++) {
        float val;
        f >> val;
        T.at<double>(i, 0) = val; 
    }
    
    f.close();
}

void CalibrationManager::getROSCalibration(const int & width, const int & height, 
                                                  const string & base_frame,
                                                  const cv::Mat & K1, cv::Mat & R1, 
                                                  const cv::Mat & K2, cv::Mat & R2,
                                                  const cv::Mat & D1, const cv::Mat & D2,
                                                  const cv::Mat & T1, const cv::Mat & T2,
                                                  sensor_msgs::CameraInfo& leftCameraInfo, 
                                                  sensor_msgs::CameraInfo& rightCameraInfo)
{
    cv::Mat P1, P2, Q;
    
    
    cv::stereoRectify(K1, D1, K2, D2, cv::Size(width, height), R2, T2, R1, R2, P1, P2, Q);
    
    P1.at<double>(0, 3) = P1.at<double>(0, 3) / K1.at<double>(0, 0);
    P2.at<double>(0, 3) = P2.at<double>(0, 3) / K2.at<double>(0, 0);
    
    
    leftCameraInfo.header.frame_id = base_frame;
    leftCameraInfo.width = width;
    leftCameraInfo.height = height;
    leftCameraInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    for (int i = 0; i < D1.rows; i++)
        leftCameraInfo.D[i] = D1.at<double>(i, 0);
    for (int i = 0, idx = 0; i < K1.rows; i++)
        for (int j = 0; j < K1.cols; j++, idx++)
            leftCameraInfo.K[idx] = K1.at<double>(i, j);
    for (int i = 0, idx = 0; i < R1.rows; i++)
        for (int j = 0; j < R1.cols; j++, idx++)
            leftCameraInfo.R[idx] = R1.at<double>(i, j);
    for (int i = 0, idx = 0; i < P1.rows; i++)
        for (int j = 0; j < P1.cols; j++, idx++)
            leftCameraInfo.P[idx] = P1.at<double>(i, j);
                
    rightCameraInfo.header.frame_id = base_frame;
    rightCameraInfo.width = width;
    rightCameraInfo.height = height;
    rightCameraInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    for (int i = 0; i < D2.rows; i++)
        rightCameraInfo.D[i] = D2.at<double>(i, 0);
    for (int i = 0, idx = 0; i < K2.rows; i++)
        for (int j = 0; j < K2.cols; j++, idx++)
            rightCameraInfo.K[idx] = K2.at<double>(i, j);
    for (int i = 0, idx = 0; i < R2.rows; i++)
        for (int j = 0; j < R2.cols; j++, idx++)
            rightCameraInfo.R[idx] = R2.at<double>(i, j);
    for (int i = 0, idx = 0; i < P2.rows; i++)
        for (int j = 0; j < P2.cols; j++, idx++)
            rightCameraInfo.P[idx] = P2.at<double>(i, j);
}

}