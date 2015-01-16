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


#include "groundtruthmanager.h"

#include <iostream>
#include <fstream>
#include <cstdlib>

#include <opencv2/opencv.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>

#include <omp.h>

using namespace std;

namespace sequence_reader {

GroundTruthManager::GroundTruthManager()
{
    ros::NodeHandle nh("~");
    m_groundTruthPub = nh.advertise<polar_grid_tracking::roiArray> ("ground_truth", 1);
}
    
void GroundTruthManager::readETHZSequence(const string & filename, 
                                          const sensor_msgs::CameraInfo leftCameraInfo, 
                                          const sensor_msgs::CameraInfo rightCameraInfo)
{
    ifstream f(filename.c_str());
    string chunk;
    
    m_leftCameraInfo = leftCameraInfo;
    m_rightCameraInfo = rightCameraInfo;
    
    
    string s;  
    while (getline(f, s, ';')) {
        vector<string> tokens;
        boost::split(tokens, s, boost::is_any_of("("));
        polar_grid_tracking::roiArrayPtr roiArray(new polar_grid_tracking::roiArray);
        roiArray->filename = tokens[0];
        for (uint32_t i = 1; i < tokens.size(); i++) {
            string token = tokens[i];
            boost::replace_all(token, " ", "");
            
            vector<string> tokens2;
            boost::split(tokens2, token, boost::is_any_of(", "));
            
            polar_grid_tracking::roi_and_speed_2d roi2d;

            int x1, y1, x2, y2;
//               +------+
//              /|     /|
//         (x1, y1)---+ |
//             | |    | |
//             | +----|-+
//             |/     |/
//             +--(x2, y2)
            x1 = atoi(tokens2[0].c_str());
            y1 = atoi(tokens2[1].c_str());
            x2 = atoi(tokens2[2].c_str());
            vector<string> tokens3;
            boost::split(tokens3, tokens2[3], boost::is_any_of(")"));
            y2 = atoi(tokens3[0].c_str());
            
            roi2d.A.u = x1;
            roi2d.A.v = y1;
            
            roi2d.B.u = x2;
            roi2d.B.v = y1;
            
            roi2d.C.u = x1;
            roi2d.C.v = y2;
            
            roi2d.D.u = x2;
            roi2d.D.v = y2;

            roiArray->rois2d.push_back(roi2d);
        }
        m_rois.push_back(roiArray);
    }
    f.close();
    
    fillRois2D();
    
//     for (int i = 0; i < m_rois.size(); i++) {
//         boost::filesystem::path imagesPath = ("/local/imaged/stixels");
//         boost::filesystem::path seqName( "bahnhof");
//         string leftImagePattern = "seq03-img-left/image_%08d_0.png";
// 
//         boost::filesystem::path("/local/imaged/stixels/bahnhof/seq03-img-left/");
//         char imgName[1024];
//         sprintf(imgName, leftImagePattern.c_str(), i + 1);
//         
//         boost::filesystem::path imgPath = imagesPath / seqName / boost::filesystem::path(imgName);
//         
//         cout << imgPath << endl;
//         
//         cv::Mat currImg = cv::imread(imgPath.c_str());
//         
//         for (uint32_t j = 0; j < m_rois.at(i)->rois2d.size(); j++) {
//             polar_grid_tracking::roi_and_speed_2d roi = m_rois.at(i)->rois2d[j];
//             cv::Rect rect(roi.A.u, roi.A.v, roi.D.u - roi.A.u, roi.D.u - roi.A.v);
//             
//             const float MAX_SPEED = 20.0f;
//             float speedX = roi.speed.x;
//             if (speedX > MAX_SPEED) speedX = MAX_SPEED;
//             if (speedX < -MAX_SPEED) speedX = -MAX_SPEED;
//             float speedY = roi.speed.y;
//             if (speedY > MAX_SPEED) speedY = MAX_SPEED;
//             if (speedY < -MAX_SPEED) speedY = -MAX_SPEED;
//             const cv::Scalar color(speedX / MAX_SPEED * 128 + 128, 
//                                    speedY / MAX_SPEED * 128 + 128, 
//                                    128);
//             cv::rectangle(currImg, cv::Point2f(roi.A.u, roi.A.v), cv::Point2f(roi.D.u, roi.D.v),
//                           color, 3);
//         }
// 
//         cv::imshow("currImg", currImg);
//         cv::moveWindow("currImg", 1366, 0);
//         
//         uint8_t keycode;
//         keycode = cv::waitKey(0);
//         if (keycode == 27) {
//             break;
//         }
//     }
//     
//     
//     exit(0);
}

void GroundTruthManager::fillRois2D()
{
    #pragma omp parallel for
    for (int i = 1; i < m_rois.size(); i++) {
        m_rois.at(i)->prevRois.resize(m_rois.at(i)->rois2d.size());
        for (uint32_t j = 0; j < m_rois.at(i)->rois2d.size(); j++) {
            polar_grid_tracking::roi_and_speed_2d currRoi = m_rois.at(i)->rois2d[j];
            cv::Mat mask1 = cv::Mat::zeros(m_leftCameraInfo.height, m_leftCameraInfo.width, CV_8UC1);
            cv::rectangle(mask1, cv::Point2f(currRoi.A.u, 
                                             currRoi.A.v), cv::Point2f(currRoi.D.u, currRoi.D.v),
                                             cv::Scalar::all(255), -1);
            int maxIntersection = 0;
            int maxIndex = -1;
            for (uint32_t k = 0; k < m_rois.at(i - 1)->rois2d.size(); k++) {
                polar_grid_tracking::roi_and_speed_2d prevRoi = m_rois.at(i - 1)->rois2d[k];
                cv::Mat mask2 = cv::Mat::zeros(m_leftCameraInfo.height, m_leftCameraInfo.width, CV_8UC1);
                cv::rectangle(mask2, 
                              cv::Point2f(prevRoi.A.u, prevRoi.A.v), cv::Point2f(prevRoi.D.u, prevRoi.D.v),
                              cv::Scalar::all(255), -1);
                
                cv::bitwise_and(mask1, mask2, mask2);
                
                const float &x11 = currRoi.A.u;
                const float &y11 = currRoi.A.v;
                const float &x12 = currRoi.D.u;
                const float &y12 = currRoi.D.v;
                
                const float &x21 = prevRoi.A.u;
                const float &y21 = currRoi.A.v;
                const float &x22 = currRoi.D.u;
                const float &y22 = currRoi.D.v;
                
                float x_overlap = max(0.0f, min(x12,x22) - max(x12,x21));
                float y_overlap = max(0.0f, min(y12,y22) - max(y11,y21));
                
                // TODO: Check
                int intersection = x_overlap * y_overlap; //cv::countNonZero(mask2);
                if (intersection > maxIntersection ) {
                    maxIntersection = intersection;
                    maxIndex = k;
                }
            }
            float totalCovered = maxIntersection / (float)cv::countNonZero(mask1);
            if (totalCovered > 0.8) {
                if (maxIndex != -1) {
                    polar_grid_tracking::roi_and_speed_2d prevRoi = m_rois.at(i - 1)->rois2d[maxIndex];
                    
                    const cv::Point2f centerPrev((prevRoi.A.u + prevRoi.D.u) / 2.0f, 
                                                (prevRoi.A.v + prevRoi.D.v) / 2.0f);
                    const cv::Point2f centerCurr((currRoi.A.u + currRoi.D.u) / 2.0f, 
                                                (currRoi.A.v + currRoi.D.v) / 2.0f);
                    m_rois.at(i)->rois2d[j].speed.x = centerCurr.x - centerPrev.x;
                    m_rois.at(i)->rois2d[j].speed.y = centerCurr.y - centerPrev.y;
                    
                    m_rois.at(i)->prevRois[j] = maxIndex;
                }
            }
        }
    }
}

void GroundTruthManager::publishROI(const int& index)
{
    if (index > (int)(m_rois.size() - 1))
        return;
    
    polar_grid_tracking::roiArray gtMessage = *m_rois.at(index);
    gtMessage.ns = "ground_truth";
    gtMessage.id = index;
    gtMessage.header.stamp = ros::Time::now();
    gtMessage.header.frame_id = m_leftCameraInfo.header.frame_id;
    m_groundTruthPub.publish(gtMessage);
}

    
}