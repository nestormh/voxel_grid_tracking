/*
 *    Copyright 2013 Néstor Morales Hernández <nestor@isaatc.ull.es>
 * 
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 * 
 *        http://www.apache.org/licenses/LICENSE-2.0
 * 
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include <iostream>
#include <stdio.h>
#include <fstream>

#include "utils.h"

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

#include <boost/gil/gil_all.hpp>

#include "rectification.h"
#include "stixelsapplicationros.h"
#include <omp.h>

using namespace std;
using namespace stixel_world_ros;

int main(int argC, char * argV[]) {
    ros::init(argC, argV, "stixels_publisher");
    
    StixelsApplicationROS app("/home/nestor/Dropbox/KULeuven/projects/StixelWorld/conf/test_stixel_world_lib.config.ini");
    app.runStixelsApplication();
    
    
    return 0;
}