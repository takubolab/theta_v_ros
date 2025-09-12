#ifndef DRIVER_ROS2_H
#define DRIVER_ROS2_H

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
// #include <compressed_image_transport/compression_common.h>
#include <image_transport/image_transport.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <libuvc/libuvc.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "thetauvc/thetauvc.h"
#include "ffmpeg/H264Decoder.h"

#include <iostream>
#include <vector>
#include <sstream>

#define MAX_PIPELINE_LEN 1000

namespace enc = sensor_msgs::image_encodings;
// namespace cit = compressed_image_transport;

struct gst_src {
    GstElement* pipeline;
    GstElement* appsrc;

    GMainLoop* loop;
    GTimer* timer;
    guint framecount;
    guint id;
    guint bus_watch_id;
    uint32_t dwFrameInterval;
    uint32_t dwClockFrequency;
};

class thetaVDriver : public rclcpp::Node
{
public:
    thetaVDriver();
    ~thetaVDriver();

    void publishImage(const cv::Mat& image);
    void setPipeProc();
    bool gst_src_init(gst_src& srcIn);
    int findDevList(uvc_context_t* ctx);
    bool set4k();
    bool isOff();
    void cv2sensorImg(const cv::Mat& mat, sensor_msgs::msg::Image& sensorImg);
    void cv2sensorImgComp(const cv::Mat& mat, sensor_msgs::msg::CompressedImage& sensorImg);

private:
    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub_image;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr m_pub_imageComp;

    // Service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_service_turnoff;

    // パラメータ
    float m_param_timeOffset;
    bool m_param_useNVdec;
    bool m_param_use4K;
    int m_param_thetaIndex;
    bool m_param_rawOn;
    bool m_param_compressOn;
    int m_param_pngLevel;

    bool m_turnoff = false;

    std::string pipe_proc;

    // メソッド
    void getparam();
    bool turnOff(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response
    );
};

#endif
