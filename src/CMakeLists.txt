set(VISO_FILES 
    libviso2/filter.cpp
    libviso2/matcher.cpp
    libviso2/matrix.cpp
    libviso2/reconstruction.cpp
    libviso2/triangle.cpp
    libviso2/viso.cpp
    libviso2/viso_mono.cpp
    libviso2/viso_stereo.cpp
)

add_executable(polar_grid_tracking 
#     main.cpp
    obstacle.cpp 
    polarcell.cpp 
    polargridtrackingros.cpp
    ObstaclesFromStereo.cpp
    polargridtracking.cpp
    particle.cpp
    cell.cpp
    utilspolargridtracking.cpp
#     libvisohelper.cpp
#     ${VISO_FILES}
    ${LIBELAS_SRC_FILES}
    main_ros.cpp
)

include_directories(polar_grid_tracking
#     GPULibSVM
#     ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}/pcl-1.7
    ${OpenCV_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${LIBELAS_INCLUDE_DIRS}
    /opt/ros/indigo/include
#     emon.a
#     ${CUDA_INCLUDE_DIRS}
#     /usr/local/dislin
)

link_directories(polar_grid_tracking
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu/
)

target_link_libraries(polar_grid_tracking
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
#   ${CUDA_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
#   ${PROTOBUF_LIBRARIES}
#   ${CMAKE_THREAD_LIBS_INIT}
#   ${Boost_
#   ${CGAL_CGAL_LIBRARY}
#   ${GMP_LIBRARIES}
#   ${CUDA_CURAND_LIBRARIES}
#   ${OPENGL_LIBRARIES}
#   ${GLUT_LIBRARIES}
#   emon
#     /usr/local/dislin/lib/dislin-10.3.a
#     /usr/local/dislin/libdislin.a
#     /usr/local/dislin/libdislin.so
#     /usr/local/dislin/libdislnc.a
#     /usr/local/dislin/lib/dislin_d-10.3.so
#     /usr/local/dislin/lib/discpp-10.3.so
)

#################################################################
# stereo_and_odom
#################################################################
add_executable(stereo_and_odom
#     obstacle.cpp 
#     polarcell.cpp 
#     polargridtrackingros.cpp
    ObstaclesFromStereo.cpp
    sequence_reader/calibrationmanager.cpp
    sequence_reader/groundtruthmanager.cpp
#     polargridtracking.cpp
#     particle.cpp
#     cell.cpp
#     utils.cpp
#     libvisohelper.cpp
#     ${VISO_FILES}
    ${POLAR_CALIBRATION_CPP_FILES}
    ${LIBELAS_SRC_FILES}
    mainStereoAndOdom.cpp
)

include_directories(stereo_and_odom
    ${POLAR_CALIBRATION_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}/pcl-1.7
    ${OpenCV_INCLUDE_DIR}
    ${LIBELAS_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    /opt/ros/indigo/include
)

link_directories(stereo_and_odom
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu/
)

target_link_libraries(stereo_and_odom
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(stereo_and_odom ${PROJECT_NAME}_generate_messages_cpp)

###########################################################
# voxel_grid_tracking
###########################################################
add_executable(voxel_grid_tracking 
    voxelobstacle.cpp 
    utilspolargridtracking.cpp
    voxel.cpp 
    particle3d.cpp
    voxelgridtracking.cpp
    main_voxel.cpp
)

include_directories(voxel_grid_tracking
#     GPULibSVM
#     ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${OCTOMAP_INCLUDE_DIRS}
    /opt/ros/indigo/include
    /home/nestor/Dropbox/ros/indigo/nestormh/devel/include
    /home/nestor/Dropbox/ros/indigo/nestormh/src/octomap_msgs/include
#     emon.a
#     ${CUDA_INCLUDE_DIRS}
#     /usr/local/dislin
)

link_directories(voxel_grid_tracking
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu/
)

target_link_libraries(voxel_grid_tracking
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
#   ${CUDA_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
#   ${PROTOBUF_LIBRARIES}
#   ${CMAKE_THREAD_LIBS_INIT}
#   ${Boost_
#   ${CGAL_CGAL_LIBRARY}
#   ${GMP_LIBRARIES}
#   ${CUDA_CURAND_LIBRARIES}
#   ${OPENGL_LIBRARIES}
#   ${GLUT_LIBRARIES}
#   emon
#     /usr/local/dislin/lib/dislin-10.3.a
#     /usr/local/dislin/libdislin.a
#     /usr/local/dislin/libdislin.so
#     /usr/local/dislin/libdislnc.a
#     /usr/local/dislin/lib/dislin_d-10.3.so
#     /usr/local/dislin/lib/discpp-10.3.so
)