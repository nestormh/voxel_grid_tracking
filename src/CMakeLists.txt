add_executable(PolarGridTracking 
    main.cpp
    ObstaclesFromStereo.cpp
    polargridtracking.cpp
    particle.cpp
    cell.cpp
    utils.cpp
)
  
include_directories(PolarGridTracking
#     GPULibSVM
#     ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
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