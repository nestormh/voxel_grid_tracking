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
            int xA, yA, xB, yB;
//               +------+
//              /|     /|
//         (x1, y1)---+ |
//             | |    | |
//             | +----|-+
//             |/     |/
//             +--(x2, y2)
            xA = atoi(tokens2[0].c_str());
            yA = atoi(tokens2[1].c_str());
            xB = atoi(tokens2[2].c_str());
            vector<string> tokens3;
            boost::split(tokens3, tokens2[3], boost::is_any_of(")"));
            yB = atoi(tokens3[0].c_str());
            
            x1 = min(xA, xB);
            x2 = max(xA, xB);
            y1 = min(yA, yB);
            y2 = max(yA, yB);
            
            roi2d.A.u = x1;
            roi2d.A.v = y1;
            
            roi2d.B.u = x2;
            roi2d.B.v = y1;
            
            roi2d.C.u = x1;
            roi2d.C.v = y2;
            
            roi2d.D.u = x2;
            roi2d.D.v = y2;
            
            roi2d.E.u = x1;
            roi2d.E.v = y1;
            
            roi2d.F.u = x2;
            roi2d.F.v = y1;
            
            roi2d.G.u = x1;
            roi2d.G.v = y2;
            
            roi2d.H.u = x2;
            roi2d.H.v = y2;
            
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
            cv::Point2d pointUL_curr, pointBR_curr;
            pointUL_curr.x = min(currRoi.A.u, currRoi.E.u);
            pointUL_curr.y = min(currRoi.A.v, currRoi.E.v);
            
            pointBR_curr.x = max(currRoi.D.u, currRoi.H.u);
            pointBR_curr.y = max(currRoi.D.v, currRoi.H.v);
            
            int maxIntersection = 0;
            int maxIndex = -1;
            for (uint32_t k = 0; k < m_rois.at(i - 1)->rois2d.size(); k++) {
                polar_grid_tracking::roi_and_speed_2d prevRoi = m_rois.at(i - 1)->rois2d[k];
                
                cv::Point2d pointUL_prev, pointBR_prev;
                pointUL_prev.x = min(prevRoi.A.u, prevRoi.E.u);
                pointUL_prev.y = min(prevRoi.A.v, prevRoi.E.v);
                
                pointBR_prev.x = max(prevRoi.D.u, prevRoi.H.u);
                pointBR_prev.y = max(prevRoi.D.v, prevRoi.H.v);
                
                float x_overlap = max(0.0, min(pointBR_prev.x, pointBR_curr.x) - max(pointUL_prev.x, pointUL_curr.x));
                float y_overlap = max(0.0, min(pointBR_prev.y, pointBR_curr.y) - max(pointUL_prev.y, pointUL_curr.y));
                
                int intersection = x_overlap * y_overlap;
                if (intersection > maxIntersection ) {
                    maxIntersection = intersection;
                    maxIndex = k;
                }
            }
            
            float x_size = max(0.0, pointBR_curr.x - pointUL_curr.x);
            float y_size = max(0.0, pointBR_curr.y - pointUL_curr.y);
            float totalCovered = maxIntersection / (x_size * y_size);
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