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

add_executable(PolarGridTracking 
#     main.cpp
    obstacle.cpp 
    polarcell.cpp 
    polargridtrackingros.cpp
    ObstaclesFromStereo.cpp
    polargridtracking.cpp
    particle.cpp
    cell.cpp
    utilspolargridtracking.cpp
    libvisohelper.cpp
    ${VISO_FILES}
    main_ros.cpp
)

include_directories(PolarGridTracking
#     GPULibSVM
#     ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}/pcl-1.7
    ${OpenCV_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    /opt/ros/hydro/include
#     emon.a
#     ${CUDA_INCLUDE_DIRS}
#     /usr/local/dislin
)

# link_directories(PolarGridTracking
#     /usr/local/dislin/lib
# )

target_link_libraries(PolarGridTracking
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
#     polargridtracking.cpp
#     particle.cpp
#     cell.cpp
#     utils.cpp
#     libvisohelper.cpp
#     ${VISO_FILES}
    mainStereoAndOdom.cpp
)

include_directories(stereo_and_odom
    ${PCL_INCLUDE_DIRS}/pcl-1.7
    ${OpenCV_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    /opt/ros/hydro/include
)

# link_directories(PolarGridTracking
#     /usr/local/dislin/lib
# )

target_link_libraries(stereo_and_odom
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

#################################################################
# stixels_publisher
#################################################################
add_executable(stixels_publisher
    ${STIXEL_WORLD_SRC}
    stixelsapplicationros.cpp
    mainStixels.cpp
)

include_directories(stixels_publisher
    ${PCL_INCLUDE_DIRS}/pcl-1.7
    ${OpenCV_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${STIXEL_WORLD_INCLUDE_DIRS}
    /opt/ros/hydro/include
)

target_link_libraries(stixels_publisher
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${STIXEL_WORLD_LIBRARIES}
  ${catkin_LIBRARIES}
)

###########################################################
# VoxelGridTracking
###########################################################
add_executable(VoxelGridTracking 
    voxelgridtracking.cpp
    main_voxel.cpp
)

include_directories(VoxelGridTracking
#     GPULibSVM
#     ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}/pcl-1.7
    ${OpenCV_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${OCTOMAP_INCLUDE_DIRS}
    /opt/ros/hydro/include
    /home/nestor/Dropbox/ros/hydro/nestormh/devel/include
    /home/nestor/Dropbox/ros/hydro/nestormh/src/octomap_msgs/include
#     emon.a
#     ${CUDA_INCLUDE_DIRS}
#     /usr/local/dislin
)

# link_directories(PolarGridTracking
#     /usr/local/dislin/lib
# )

target_link_libraries(VoxelGridTracking
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