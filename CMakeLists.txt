cmake_minimum_required(VERSION 2.8.11)
project(CALIB_CAM)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenCV REQUIRED)
include_directories($(OpenCV_INCLUDE_DIRS))

add_executable(calibrate_pinhole calib_pinhole.cpp popt_pp.h)
target_link_libraries(calibrate_pinhole ${OpenCV_LIBS} "-lpopt")

#add_executable(read read_images.cpp)
#target_link_libraries(read ${OpenCV_LIBS} "-lpopt")

add_executable(calibrate_pinhole_stereo calib_pinhole_stereo.cpp)
target_link_libraries(calibrate_pinhole_stereo ${OpenCV_LIBS} "-lpopt")

add_executable(undistort_rectify undistort_rectify.cpp)
target_link_libraries(undistort_rectify ${OpenCV_LIBS} "-lpopt")

add_executable(calibrate_fisheye_stereo calib_fisheye_stereo.cpp)
target_link_libraries(calibrate_fisheye_stereo ${OpenCV_LIBS} "-lpopt")
