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

#ifndef GROUNDTRUTHMANAGER_H
#define GROUNDTRUTHMANAGER_H

#include <string>

#include "polar_grid_tracking/roiArray.h"

#include <sensor_msgs/CameraInfo.h>

#include <ros/ros.h>

using namespace std;

namespace sequence_reader {

class GroundTruthManager
{
public:
    GroundTruthManager();
    
    void readETHZSequence(const string & filename, 
                          const sensor_msgs::CameraInfo leftCameraInfo, 
                          const sensor_msgs::CameraInfo rightCameraInfo);
    
    void publishROI(const int & index);
    
private:
    void fillRois2D();
    
    // Properties
    vector<polar_grid_tracking::roiArrayPtr> m_rois;
    sensor_msgs::CameraInfo m_leftCameraInfo, m_rightCameraInfo;
    
    ros::Publisher m_groundTruthPub;
    
};

}
#endif // GROUNDTRUTHMANAGER_H
