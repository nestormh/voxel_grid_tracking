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

using namespace std;

namespace sequence_reader {

GroundTruthManager::GroundTruthManager()
{

}
    
void GroundTruthManager::readETHZSequence(string filename)
{
    ifstream f(filename.c_str());
    string chunk;
    
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
        mp_rois.push_back(roiArray);
    }
    f.close();
    
//     for (int i = 0; i < mp_rois.size(); i++) {
// //         cout << mp_roiArray->rois2d[i] << endl;
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
//         for (uint32_t j = 0; j < mp_rois.at(i)->rois2d.size(); j++) {
//             polar_grid_tracking::roi_and_speed_2d roi = mp_rois.at(i)->rois2d[j];
//             cv::Rect rect(roi.A.u, roi.A.v, roi.D.u - roi.A.u, roi.D.u - roi.A.v);
//             
//             cout << "Drawing " << cv::Point2f(roi.A.u, roi.A.v) << " -> " << 
//                                   cv::Point2f(roi.D.u, roi.D.v) << endl;
//             
//             cv::rectangle(currImg, cv::Point2f(roi.A.u, roi.A.v), cv::Point2f(roi.D.u, roi.D.v),
//                           cv::Scalar(0x0F + rand() & 0x0F, 0x0F + rand() & 0x0F, 0x0F + rand() & 0x0F), 3);
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
    
    
    exit(0);
}

    
}