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


#include "stixelsapplicationros.h"
#include </home/nestor/Dropbox/KULeuven/projects/StixelWorld/src/doppia/stixel3d.h>
#include <doppia/stixel3d.h>

#include "doppia/extendedstixelworldestimatorfactory.h"

#include "utils.h"
#include "fundamentalmatrixestimator.h"

#include <boost/filesystem.hpp>
#include <boost/concept_check.hpp>

#include <fstream>
#include <eigen3/Eigen/src/Core/Matrix.h>

#include <omp.h>
#include <opencv2/core/core.hpp>

#define CAMERA_FRAME_ID "left_cam"

using namespace stixel_world;

namespace stixel_world_ros {

StixelsApplicationROS::StixelsApplicationROS(const string& optionsFile)
{
    m_options = parseOptionsFile(optionsFile);
    
    mp_video_input.reset(doppia::VideoInputFactory::new_instance(m_options));
    
    if(not mp_video_input)
    {
        throw std::invalid_argument("Failed to initialize a video input module. "
        "No images to read, nothing to compute.");
    }

    m_initialFrame = mp_video_input->get_current_frame_number() + 1;
    
    mp_stixel_world_estimator.reset(StixelWorldEstimatorFactory::new_instance(m_options, *mp_video_input));
    mp_prevStixels.reset(new stixels_t);
    mp_polarCalibration.reset(new PolarCalibration());

    mp_stixel_motion_evaluator.reset(new MotionEvaluation(m_options));
    
//     NOTE: This is just for fast tuning of the motion estimators
//     mp_stixels_tests.resize(6);
//     for (uint32_t i = 0; i < mp_stixels_tests.size(); i++) {
//         mp_stixels_tests[i].reset( 
//             new StixelsTracker( m_options, mp_video_input->get_metric_camera(), 
//                                 mp_stixel_world_estimator->get_stixel_width(),
//                                 mp_polarCalibration) );
//     }
//     mp_stixels_tests[0]->set_motion_cost_factors(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, false);
//     mp_stixels_tests[1]->set_motion_cost_factors(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, false);
//     mp_stixels_tests[2]->set_motion_cost_factors(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, false);
//     mp_stixels_tests[3]->set_motion_cost_factors(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, true);
//     mp_stixels_tests[4]->set_motion_cost_factors(0.0f, 0.0f, 0.0f, 1.0f, 0.0f, true);
//     mp_stixels_tests[5]->set_motion_cost_factors(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, true);
//     
//     mp_stixel_motion_evaluator->addStixelMotionEstimator(mp_stixel_world_estimator, mp_stixels_tests[0]);
//     mp_stixel_motion_evaluator->addStixelMotionEstimator(mp_stixel_world_estimator, mp_stixels_tests[1]);
//     mp_stixel_motion_evaluator->addStixelMotionEstimator(mp_stixel_world_estimator, mp_stixels_tests[2]);
//     mp_stixel_motion_evaluator->addStixelMotionEstimator(mp_stixel_world_estimator, mp_stixels_tests[3]);
//     mp_stixel_motion_evaluator->addStixelMotionEstimator(mp_stixel_world_estimator, mp_stixels_tests[4]);
//     mp_stixel_motion_evaluator->addStixelMotionEstimator(mp_stixel_world_estimator, mp_stixels_tests[5]);
// end of NOTE
    
    if (mp_stixels_tests.size() == 0) {
        mp_stixel_motion_estimator.reset( 
        new StixelsTracker( m_options, mp_video_input->get_metric_camera(), 
                            mp_stixel_world_estimator->get_stixel_width(),
                            mp_polarCalibration) );
        mp_stixel_motion_estimator->set_motion_cost_factors(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, true);
        
        mp_stixel_motion_evaluator->addStixelMotionEstimator(mp_stixel_world_estimator, mp_stixel_motion_estimator);
    } else {
        mp_stixel_motion_estimator = mp_stixels_tests[0];
    }
    
    m_waitTime = 0;
    
    ros::NodeHandle nh("~");
    m_pointCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("pointCloudStixels", 1);
    m_accTime = 0.0;
    
    return;
}

boost::program_options::variables_map StixelsApplicationROS::parseOptionsFile(const string& optionsFile) 
{

    boost::program_options::variables_map options;
    if (optionsFile.empty() == false)
    {
        boost::filesystem::path configurationFilePath(optionsFile);
        if(boost::filesystem::exists(configurationFilePath) == false)
        {
            cout << "\033[1;31mCould not find the configuration file:\033[0m "
                 << configurationFilePath << endl;
            return options;
        }

//         init_stixel_world(configurationFilePath);
        boost::program_options::options_description desc;
        get_options_description(desc);
        desc.add(StixelsTracker::get_args_options());
        desc.add(MotionEvaluation::get_args_options());
        
//         desc.add_options()
//         ("save_stixels",
//          program_options::value<bool>()->default_value(false),
//          "save the estimated stixels in a data sequence file")
//         
//         ("save_ground_plane_corridor",
//          program_options::value<bool>()->default_value(false),
//          "save the estimated expected bottom and top of objects in the data sequence file")
//         
//         ("gui.disabled",
//          program_options::value<bool>()->default_value(false),
//          "if true, no user interface will be presented")
//         
//         ("silent_mode",
//          program_options::value<bool>()->default_value(false),
//          "if true, no status information will be printed at run time (use this for speed benchmarking)")
//         
//         ("stixel_world.motion",
//          boost::program_options::value<bool>()->default_value(false),
//          "if true the stixels motion will be estimated")
        
//         ;
        
        printf("Going to parse the configuration file: %s\n", configurationFilePath.c_str());

        try
        {
            fstream configuration_file;
            configuration_file.open(configurationFilePath.c_str(), fstream::in);
            boost::program_options::store(boost::program_options::parse_config_file(configuration_file, desc), options);
            configuration_file.close();
        }
        catch (...)
        {
            cout << "\033[1;31mError parsing the configuration file named:\033[0m "
            << configurationFilePath << endl;
            cout << desc << endl;
            throw;
        }

        cout << "Parsed the configuration file " << configurationFilePath << std::endl;
    }
    return options;   
}

void StixelsApplicationROS::runStixelsApplication()
{
//     cv::namedWindow("output");
//     cv::moveWindow("output", 1366, 0);
    
    
//     cv::namedWindow("polar");
//     if (mp_stixels_tests.size() == 0)
//         cv::moveWindow("polar", 2646, 0);
    
//     cv::namedWindow("denseTrack");
//     cv::moveWindow("denseTrack", 1366, 480);
    
//     cv::namedWindow("polarTrack");
//     cv::moveWindow("polarTrack", 1366, 0);
    
    m_prevLeftRectified = doppia::AbstractVideoInput::input_image_t(mp_video_input->get_left_image().dimensions());
    m_prevRightRectified = doppia::AbstractVideoInput::input_image_t(mp_video_input->get_right_image().dimensions());
    
    double startWallTime = omp_get_wtime();
    while (iterate()) {
//         visualize();
        publishStixels();
        update();
        cout << "Time for " << __FUNCTION__ << ": " << omp_get_wtime() - startWallTime << endl;
        startWallTime = omp_get_wtime();
        cout << "********************************" << endl;
    }
}

void StixelsApplicationROS::update()
{

    const double & startWallTime = omp_get_wtime();
    
    // Updating the rectified images
    const stixel_world::input_image_const_view_t & currLeft = mp_video_input->get_left_image();
    const stixel_world::input_image_const_view_t & currRight = mp_video_input->get_right_image();
    
    boost::gil::copy_pixels(currLeft, boost::gil::view(m_prevLeftRectified));
    boost::gil::copy_pixels(currRight, boost::gil::view(m_prevRightRectified));
    
    // TODO: Use again when speed information is needed
    mp_stixel_motion_estimator->set_estimated_stixels(mp_stixel_world_estimator->get_stixels());
    
//     for (uint32_t i = 0; i < mp_stixels_tests.size(); i++)
//         mp_stixels_tests[i]->set_estimated_stixels(mp_stixel_world_estimator->get_stixels());
    
    // Updating the stixels
    mp_prevStixels->resize(mp_stixel_world_estimator->get_stixels().size());
    std::copy(mp_stixel_world_estimator->get_stixels().begin(), 
                mp_stixel_world_estimator->get_stixels().end(), 
                mp_prevStixels->begin());
    
    cout << "Time for " << __FUNCTION__ << ": " << omp_get_wtime() - startWallTime << endl;
}

bool StixelsApplicationROS::iterate()
{
    const double & startWallTime = omp_get_wtime();
    
    
    if ((mp_video_input->get_current_frame_number() == mp_video_input->get_number_of_frames()) || (! mp_video_input->next_frame()))
        return false;
    
    doppia::AbstractVideoInput::input_image_view_t
                        left_view(mp_video_input->get_left_image()),
                        right_view(mp_video_input->get_right_image());  
            
    gil2opencv(mp_video_input->get_left_image(), m_currLeft);
    mp_stixel_world_estimator->set_rectified_images_pair(left_view, right_view);
    mp_stixel_world_estimator->compute();
    
    if (! rectifyPolar()) {
//         TODO: Do something in this case
        return true;
    }

    // TODO: Use again when speed information is needed
    mp_stixel_motion_estimator->set_new_rectified_image(left_view);
    mp_stixel_motion_estimator->updateDenseTracker(m_currLeft);
    mp_stixel_motion_estimator->set_estimated_stixels(mp_stixel_world_estimator->get_stixels());
    
    if(mp_video_input->get_current_frame_number() > m_initialFrame)
        mp_stixel_motion_estimator->compute();
    
//     for (uint32_t i = 0; i < mp_stixels_tests.size(); i++) {
//         mp_stixels_tests[i]->set_new_rectified_image(left_view);
//         mp_stixels_tests[i]->updateDenseTracker(m_currLeft);
//         mp_stixels_tests[i]->set_estimated_stixels(mp_stixel_world_estimator->get_stixels());
//         
//         if(mp_video_input->get_current_frame_number() > m_initialFrame)
//             mp_stixels_tests[i]->compute();
//     }
    
//     mp_stixel_motion_evaluator->evaluate(mp_video_input->get_current_frame_number() - 1);
    
    cout << "Time for " << __FUNCTION__ << ": " << omp_get_wtime() - startWallTime << endl;
    
    return true;
}

bool StixelsApplicationROS::rectifyPolar()
{
    const double & startWallTime = omp_get_wtime();
    
    if (mp_video_input->get_current_frame_number() == m_initialFrame)
        return true;
    
    cv::Mat prevLeft, prevRight, currRight, FL, FR;
    gil2opencv(boost::gil::view(m_prevLeftRectified), prevLeft);
    gil2opencv(boost::gil::view(m_prevRightRectified), prevRight);
//     gil2opencv(mp_video_input->get_left_image(), m_currLeft);
    gil2opencv(mp_video_input->get_right_image(), currRight);
    vector < vector < cv::Point2f > > correspondences;
    
    if (! FundamentalMatrixEstimator::findF(prevLeft, prevRight, m_currLeft, currRight, FL, FR, correspondences, 50))
        return false;
    
    //NOTE: Remove after debugging
//     {
//         char matName[1024];
//         sprintf(matName, "/tmp/results/mats/lastMat_%04d.xml", mp_video_input->get_current_frame_number());
//         cv::FileStorage file(matName, cv::FileStorage::WRITE);
//         file << "F" << FL;
//         file.release();
//     }
    // end of NOTE
    
    
    if (!  mp_polarCalibration->compute(prevLeft, m_currLeft, FL, correspondences[0], correspondences[3])) {
        cout << "Error while trying to get the polar alignment for the images in the left" << endl;
        return false;
    }
    
    mp_polarCalibration->rectifyAndStoreImages(prevLeft, m_currLeft);
    
//     transformStixels();
    
//     // NOTE: Just for debugging
//     {
//         vector<cv::Point2d> oldPoints1(mp_stixel_world_estimator->get_stixels().size());
//         vector<cv::Point2d> oldPoints2(mp_stixel_world_estimator->get_stixels().size());
//         for (uint32_t i = 0; i < mp_prevStixels->size(); i++) {
//             oldPoints1[i] = cv::Point2d((*mp_prevStixels)[i].x, 
//                                         (*mp_prevStixels)[i].bottom_y);
//         }
//         for (uint32_t i = 0; i < mp_stixel_world_estimator->get_stixels().size(); i++) {
//             oldPoints2[i] = cv::Point2d(mp_stixel_world_estimator->get_stixels()[i].x, 
//                                         mp_stixel_world_estimator->get_stixels()[i].bottom_y);
//         }
//         
//         vector<cv::Point2d> newPoints1a, newPoints2a;
//         mp_polarCalibration->transformPoints(oldPoints1, newPoints1a, 1);
//         mp_polarCalibration->transformPoints(oldPoints2, newPoints2a, 2);
//         
//         vector<cv::Point2d> newPoints1b, newPoints2b;
//         mp_polarCalibration->transformPoints(FL, correspondences[0], correspondences[3], currLeft.size(), 
//                                              oldPoints1, oldPoints2, newPoints1b, newPoints2b);
//         
//         for (uint32_t i = 0; i < oldPoints1.size(); i++) {
//             cout << newPoints1a[i] << " -- " << newPoints1b[i] << " ==== " << cv::norm(newPoints1a[i] - newPoints1b[i]) << endl;
//         }
//     }
//     // NOTE: end of note
    
    // TODO: Remove, this is just for visualization
    cv::Mat Lt0, Rt0, Lt1, Rt1;
    
    mp_polarCalibration->getRectifiedImages(prevLeft, m_currLeft, Lt0, Lt1);
    mp_polarCalibration->getRectifiedImages(prevLeft, m_currLeft, Rt0, Rt1);
    
    m_polarLt0 = doppia::AbstractVideoInput::input_image_t(Lt0.cols, Lt0.rows);
    m_polarRt0 = doppia::AbstractVideoInput::input_image_t(Rt0.cols, Rt0.rows);
    m_polarLt1 = doppia::AbstractVideoInput::input_image_t(Lt1.cols, Lt1.rows);
    m_polarRt1 = doppia::AbstractVideoInput::input_image_t(Rt1.cols, Rt1.rows);
    
    boost::gil::rgb8_view_t viewLt0 = boost::gil::view(m_polarLt0);
    boost::gil::rgb8_view_t viewRt0 = boost::gil::view(m_polarRt0);
    boost::gil::rgb8_view_t viewLt1 = boost::gil::view(m_polarLt1);
    boost::gil::rgb8_view_t viewRt1 = boost::gil::view(m_polarRt1);
    
    opencv2gil(Lt0, viewLt0);
    opencv2gil(Rt0, viewRt0);
    opencv2gil(Lt1, viewLt1);
    opencv2gil(Rt1, viewRt1);
        
    cout << "Time for " << __FUNCTION__ << ": " << omp_get_wtime() - startWallTime << endl;
    
    return true;
}

void StixelsApplicationROS::transformStixels()
{
    const stixels_t & prevStixels = *mp_prevStixels;
    const stixels_t & currStixels = mp_stixel_world_estimator->get_stixels();
    
//     stixels_t m_tStixelsLt0, m_tStixelsLt1;
    vector<cv::Point2d> basePointsLt0(prevStixels.size()), topPointsLt0(prevStixels.size());
    vector<cv::Point2d> basePointsLt1(currStixels.size()), topPointsLt1(currStixels.size());
    
//     vector<cv::Point2d> basePointsTransfLt0, topPointsTransfLt0, basePointsTransfLt1, topPointsTransfLt1;
    
    for (uint32_t i = 0; i < prevStixels.size(); i++) {
        basePointsLt0[i] = cv::Point2d(prevStixels[i].x, prevStixels[i].bottom_y);
        topPointsLt0[i] = cv::Point2d(prevStixels[i].x, prevStixels[i].top_y);
    }

    for (uint32_t i = 0; i < currStixels.size(); i++) {
        basePointsLt1[i] = cv::Point2d(currStixels[i].x, currStixels[i].bottom_y);
        topPointsLt1[i] = cv::Point2d(currStixels[i].x, currStixels[i].top_y);
    }
    
    mp_polarCalibration->transformPoints(basePointsLt0, basePointsTransfLt0, 1);
    mp_polarCalibration->transformPoints(topPointsLt0, topPointsTransfLt0, 1);
//     mp_polarCalibration->transformPoints(basePointsLt1, basePointsTransfLt1, 1);
//     mp_polarCalibration->transformPoints(topPointsLt1, topPointsTransfLt1, 1);
    
    //TODO: Use stixel_t as data type. Then store them into a global variable, so previous transformed stixels
    // are not calculated again. Change this in the visualization part
}

void drawLine(cv::Mat & img, const cv::Point2d & p1, 
              const cv::Point2d & p2, const cv::Scalar & color) {

    if ((p1 != cv::Point2d(0, 0)) && (p2 != cv::Point2d(0, 0))) {
        const cv::Point2d & pA = (p1.y < p2.y)? p1 : p2;
        const cv::Point2d & pB = (p1.y < p2.y)? p2 : p1;
        
        const double dist1 = pA.y + img.rows - pB.y;
        const double dist2 = pB.y - pA.y;
        
        if (dist1 >= dist2) {
            cv::line(img, pA, pB, color);
        } else {
            cv::line(img, pA, cv::Point2d(pA.x, 0), color);
            cv::line(img, pB, cv::Point2d(pA.x, img.rows), color);
        }
    }
}

void StixelsApplicationROS::visualize2()
{
    if (mp_video_input->get_current_frame_number() == m_initialFrame)
        return;
    
    cv::Mat img1Current, img2Current, imgTracking;
    cv::Mat img1Prev, img2Prev;
    gil2opencv(stixel_world::input_image_const_view_t(mp_video_input->get_left_image()), img1Current);
    gil2opencv(stixel_world::input_image_const_view_t(mp_video_input->get_right_image()), img2Current);
    gil2opencv(stixel_world::input_image_const_view_t(mp_video_input->get_left_image()), imgTracking);
    
    cv::Mat output = cv::Mat::zeros(600, 1200, CV_8UC3);
    
    cv::Mat scale;
    gil2opencv(boost::gil::view(m_prevLeftRectified), img1Prev);
    gil2opencv(boost::gil::view(m_prevRightRectified), img2Prev);

    cv::Mat Lt0, Lt1;
    mp_polarCalibration->getRectifiedImages(img1Prev, img1Current, Lt0, Lt1);
    
    if (!Lt0.empty() && !Lt1.empty()) {
        if (mp_video_input->get_current_frame_number() != m_initialFrame) {
            for (uint32_t i = 0; i < mp_prevStixels->size(); i++) {
                const cv::Point2d & p1bLin = cv::Point2d(mp_prevStixels->at(i).x, mp_prevStixels->at(i).bottom_y);
                const cv::Point2d & p2bLin = cv::Point2d(mp_stixel_world_estimator->get_stixels().at(i).x, 
                                                     mp_stixel_world_estimator->get_stixels().at(i).bottom_y);
                const cv::Point2d & p1tLin = cv::Point2d(mp_prevStixels->at(i).x, mp_prevStixels->at(i).top_y);
                const cv::Point2d & p2tLin = cv::Point2d(mp_stixel_world_estimator->get_stixels().at(i).x, 
                                                          mp_stixel_world_estimator->get_stixels().at(i).top_y);
                
                const cv::Point2d & p1bPolar = basePointsTransfLt0[i];
                const cv::Point2d & p2bPolar = basePointsTransfLt1[i];
                const cv::Point2d & p1tPolar = topPointsTransfLt0[i];
                const cv::Point2d & p2tPolar = topPointsTransfLt1[i];
                
                cv::Scalar color(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
                drawLine(Lt0, p1bPolar, p2bPolar, color);
                drawLine(Lt0, p1tPolar, p2tPolar, color);
                
                cv::circle(img1Prev, p1bLin, 2, color, -1);
                cv::circle(img1Prev, p1tLin, 2, color, -1);
                cv::circle(img1Current, p2bLin, 2, color, -1);
                cv::circle(img1Current, p2tLin, 2, color, -1);
                {
                    const doppia::AbstractStixelMotionEstimator::stixels_motion_t & 
                            corresp = mp_stixel_motion_estimator->get_stixels_motion();
                    int32_t idx = corresp[i];
                    if (idx >= 0) {
                        const cv::Point2d & p1bLin = cv::Point2d(mp_prevStixels->at(idx).x, 
                                                                 mp_prevStixels->at(idx).bottom_y);
                        
                        cv::circle(imgTracking, p1bLin, 2, color, -1);
                        cv::line(imgTracking, p1bLin, p2bLin, color);
                    }
                    cv::circle(imgTracking, p2bLin, 2, color, -1);
                }
            }
        }
        
        cv::resize(Lt0, scale, cv::Size(400, 600));
        scale.copyTo(output(cv::Rect(400, 0, 400, 600)));
    }
    
    
    cv::resize(img1Prev, scale, cv::Size(400, 300));
    scale.copyTo(output(cv::Rect(0, 0, 400, 300)));
    
    cv::resize(img1Current, scale, cv::Size(400, 300));
    scale.copyTo(output(cv::Rect(0, 300, 400, 300)));
    
    cv::resize(imgTracking, scale, cv::Size(400, 300));
    scale.copyTo(output(cv::Rect(800, 0, 400, 300)));
    
    cv::imshow("output", output);
        
    //NOTE: Remove after debugging
//     {
//         gil2opencv(stixel_world::input_image_const_view_t(mp_video_input->get_left_image()), img1Current);
//         gil2opencv(boost::gil::view(m_prevLeftRectified), img1Prev);
//         mp_polarCalibration->getRectifiedImages(img1Prev, img1Current, Lt0, Lt1);
//         cv::Mat saveImg(Lt0.rows, 3 * Lt0.cols, CV_8UC3);
//         Lt0.copyTo(saveImg(cv::Rect(0, 0, Lt0.cols, Lt0.rows)));
//         Lt1.copyTo(saveImg(cv::Rect(Lt0.cols, 0, Lt1.cols, Lt1.rows)));
//         saveImg(cv::Rect(Lt0.cols * 2, 0, Lt0.cols, Lt0.rows)) = Lt0 - Lt1;
//         char testName[1024];
//         sprintf(testName, "/tmp/results/img%04d.png", mp_video_input->get_current_frame_number());
//         cv::imwrite(string(testName), saveImg);
//     }
    // end of NOTE
    
    waitForKey(&m_waitTime);
}

void StixelsApplicationROS::visualize3()
{
    const double & startWallTime = omp_get_wtime();
    
    if (mp_video_input->get_current_frame_number() == m_initialFrame)
        return;
    
    cv::Mat imgCurrent, imgPrev;
    gil2opencv(stixel_world::input_image_const_view_t(mp_video_input->get_left_image()), imgCurrent);
    gil2opencv(boost::gil::view(m_prevLeftRectified), imgPrev);
    
    stixels_t prevStixels = mp_stixel_motion_estimator->get_previous_stixels();
    stixels_t currStixels = mp_stixel_motion_estimator->get_current_stixels();
    AbstractStixelMotionEstimator::stixels_motion_t corresp = mp_stixel_motion_estimator->get_stixels_motion();
    
    for (uint32_t i = 0; i < prevStixels.size(); i++) {
        const cv::Point2d p1a(prevStixels[i].x, prevStixels[i].bottom_y);
        const cv::Point2d p1b(prevStixels[corresp[i]].x, prevStixels[corresp[i]].bottom_y);
        const cv::Point2d p2(currStixels[i].x, currStixels[i].bottom_y);
        
        if (corresp[i] < 0)
            continue;
        
        const cv::Scalar color(rand() & 0xFF, rand() & 0xFF, rand() & 0xFF);
        
        cv::circle(imgPrev, p1a, 1, color);
        
        cv::line(imgCurrent, p1b, p2, color);
        
        cv::circle(imgCurrent, p1b, 1, color);
    }
    
    cv::Mat output = cv::Mat::zeros(m_prevLeftRectified.height(), 2 * m_prevLeftRectified.width(), CV_8UC3);
    cv::Mat topView;
    mp_stixel_motion_estimator->drawTracker(imgPrev, topView);
    imgPrev.copyTo(output(cv::Rect(0, 0, imgPrev.cols, imgPrev.rows)));
    topView.copyTo(output(cv::Rect(imgPrev.cols, 0, topView.cols, topView.rows)));
    
//     imgCurrent.copyTo(output(cv::Rect(imgPrev.cols, 0, imgCurrent.cols, imgCurrent.rows)));
    
    cv::imshow("output", output);
    
    cout << "Time for " << __FUNCTION__ << ": " << omp_get_wtime() - startWallTime << endl;
    
    waitForKey(&m_waitTime);
}

void StixelsApplicationROS::visualize()
{
    if (mp_video_input->get_current_frame_number() == m_initialFrame)
        return;
    
    cv::Mat polarOutput;
    cv::Mat polar1, polar2, diffPolar;
    mp_polarCalibration->getStoredRectifiedImages(polar1, polar2);
    cv::Mat inverseX, inverseY;
    mp_polarCalibration->getInverseMaps(inverseX, inverseY, 1);
    cv::absdiff(polar1, polar2, diffPolar);
    cv::Mat diffRect = cv::Mat::zeros(mp_video_input->get_left_image().height(), mp_video_input->get_left_image().width(), CV_8UC3);
    cv::remap(diffPolar, diffRect, inverseX, inverseY, cv::INTER_CUBIC, cv::BORDER_TRANSPARENT);
    cv::resize(diffPolar, polarOutput, cv::Size(640, 720));
    mp_stixel_motion_estimator->drawTracker(diffRect);
    cv::imshow("polar", diffRect);
    
//     cv::Mat outputDenseTrack;
//     mp_stixel_motion_estimator->drawDenseTracker(outputDenseTrack);
//     cv::imshow("denseTrack", outputDenseTrack);

    if (mp_stixels_tests.size() == 0) {
        visualize3();
        return;
    }
        
    const double & startWallTime = omp_get_wtime();
    
    cv::Mat imgCurrent[6], topView[6];
    gil2opencv(stixel_world::input_image_const_view_t(mp_video_input->get_left_image()), imgCurrent[0]);
    mp_stixels_tests[0]->drawTracker(imgCurrent[0], topView[0]);
    
    cv::Size topSize = cv::Size(imgCurrent[0].cols / 2, imgCurrent[0].rows / 2);
    cv::Mat output = cv::Mat::zeros(2 * imgCurrent[0].rows + topSize.height, 3 * m_prevLeftRectified.width(), CV_8UC3);
    
    cv::Mat topScaled;
    cv::resize(topView[0], topScaled, topSize);
    topScaled.copyTo(output(cv::Rect(0, 2 * imgCurrent[0].rows, topScaled.cols, topScaled.rows)));
    
    for (uint32_t i = 1; i < mp_stixels_tests.size(); i++) {
        imgCurrent[0].copyTo(imgCurrent[i]);
        imgCurrent[i] = cv::Mat(imgCurrent[0].rows, imgCurrent[0].cols, CV_8UC3);
        mp_stixels_tests[i]->drawTracker(imgCurrent[i], topView[i]);
        
        cv::resize(topView[i], topScaled, topSize);
        topScaled.copyTo(output(cv::Rect(topScaled.cols * i, 2 * imgCurrent[0].rows, topScaled.cols, topScaled.rows)));
    }
    
    imgCurrent[0].copyTo(output(cv::Rect(0, 0, imgCurrent[0].cols, imgCurrent[0].rows)));
    imgCurrent[1].copyTo(output(cv::Rect(imgCurrent[0].cols, 0, imgCurrent[1].cols, imgCurrent[1].rows)));
    imgCurrent[2].copyTo(output(cv::Rect(2 * imgCurrent[0].cols, 0, imgCurrent[2].cols, imgCurrent[2].rows)));
    imgCurrent[3].copyTo(output(cv::Rect(0, imgCurrent[0].rows, imgCurrent[3].cols, imgCurrent[3].rows)));
    imgCurrent[4].copyTo(output(cv::Rect(imgCurrent[0].cols, imgCurrent[0].rows, imgCurrent[4].cols, imgCurrent[4].rows)));
    imgCurrent[5].copyTo(output(cv::Rect(2 * imgCurrent[0].cols, imgCurrent[0].rows, imgCurrent[5].cols, imgCurrent[5].rows)));
    
    cv::imshow("output", output);
    
    cout << "Time for " << __FUNCTION__ << ": " << omp_get_wtime() - startWallTime << endl;
    
//     waitForKey(&m_waitTime);
        
}

void StixelsApplicationROS::publishPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointCloud) {
    
    cout << "Publishing Stixels" << endl;   
    
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg (*pointCloud, cloudMsg);
    cloudMsg.header.frame_id = CAMERA_FRAME_ID;
    cloudMsg.header.stamp = ros::Time();
         
    m_pointCloudPub.publish(cloudMsg);
         
    ros::spinOnce();
}

void StixelsApplicationROS::publishStixels()
{
    cv::Mat imgLeft;
    gil2opencv(stixel_world::input_image_const_view_t(mp_video_input->get_left_image()), imgLeft);
    
//     const stixels_t & stixels = mp_stixel_world_estimator->get_stixels();
    const stixels3d_t & stixels = mp_stixel_motion_estimator->getLastStixelsAfterTracking();
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (vector<Stixel3d>::const_iterator it = stixels.begin(); it != stixels.end(); it++) {
        cv::Point2i p1(it->x, it->bottom_y);
        cv::Point2i p2(it->x, it->top_y);

        const doppia::MetricStereoCamera& camera = mp_video_input->get_metric_camera();
        const double & camera_height = mp_video_input->camera_height;
        
        double depth = std::numeric_limits<double>::max();
        if (it->disparity > 0.0f)
            depth = camera.disparity_to_depth(it->disparity );
            
        for (uint32_t y = it->top_y; y <= it->bottom_y; y++) {
            
            Eigen::Vector2f point2d;
            point2d << it->x, y;
            
            
            Eigen::Vector3f point3d = camera.get_left_camera().back_project_2d_point_to_3d(point2d, depth);
            
            cv::Vec3b pixel = imgLeft.at<cv::Vec3b>(y, it->x);
            
            pcl::PointXYZRGB point;
            point.x = point3d(0);
            point.y = point3d(2);
            point.z = camera_height - point3d(1);
            point.r = pixel[2];
            point.g = pixel[1];
            point.b = pixel[0];
            pointCloud->push_back(point);
        }
        
//         cv::line(imgLeft, p1, p2, cv::Scalar(0, 255, 0));
        imgLeft.at<cv::Vec3b>(p1.y, p1.x) = cv::Vec3b(0, 255, 0);
        imgLeft.at<cv::Vec3b>(p2.y, p2.x) = cv::Vec3b(0, 255, 0);
    }
    
    
    // TODO: Get these values from somewhere
    double deltaTime = 0.1;
    
    double posX = 0.0;
    double posY = 0.0;
    double posTheta = 0.0; 
    m_accTime += deltaTime;
    
    cout << "accTime " << m_accTime << endl;

    static tf::TransformBroadcaster broadcaster;
    tf::StampedTransform transform;
    // TODO: In a real application, time should be taken from the system
    transform.stamp_ = ros::Time();
    transform.setOrigin(tf::Vector3(posX, posY, m_accTime));
    transform.setRotation( tf::createQuaternionFromRPY(0.0, 0.0, posTheta) );

    publishPointCloud(pointCloud);
    const tf::StampedTransform stamped = tf::StampedTransform(transform, ros::Time::now(), "/map", "/odom");
    cout << "stamped " << stamped.stamp_ << endl;
    broadcaster.sendTransform(stamped);
    
    
    cv::imshow("imgLeft", imgLeft);
    
    waitForKey(&m_waitTime);
}

}