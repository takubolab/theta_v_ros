cmake_minimum_required(VERSION 3.5)
project(theta_v_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(Boost 1.71 REQUIRED COMPONENTS system thread timer)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER_1_0 REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_1_0_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(LIBUVC REQUIRED libuvc)

include_directories(
    /usr/include/libuvc
    ${GSTREAMER_1_0_INCLUDE_DIRS}
    ${GSTREAMER_1_0_APP_INCLUDE_DIRS}
    ${LIBUVC_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${compressed_image_transport_INCLUDE_DIRS}
    /usr/include/libavcodec
    /usr/include/libavformat
    /usr/include/libswscale
)

add_executable(driver
    src/node.cpp          # main() はここにある
    src/driver.cpp        # クラス定義だけ
    src/ffmpeg/H264Decoder.cpp
    src/thetauvc/thetauvc.c
)

target_link_libraries(driver
    ${GSTREAMER_1_0_LIBRARIES}
    ${GSTREAMER_1_0_APP_LIBRARIES}
    ${LIBUVC_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libavcodec.so
    /usr/lib/x86_64-linux-gnu/libavformat.so
    /usr/lib/x86_64-linux-gnu/libswscale.so
    /usr/lib/x86_64-linux-gnu/libavutil.so
)

ament_target_dependencies(driver
    rclcpp
    std_srvs
    sensor_msgs
    cv_bridge
    OpenCV
    Boost
    image_transport
    compressed_image_transport
)

install(TARGETS driver
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
