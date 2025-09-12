// theta_v_driver_ros2.cpp
#include "driver.h"

thetaVDriver::thetaVDriver()
: Node("theta_v_driver")
{
    // パラメータ取得
    declare_parameter("nvdec", false);
    declare_parameter("use4k", false);
    declare_parameter("image/raw/on", false);
    declare_parameter("image/compress/on", false);
    declare_parameter("image/compress/level", 3);

    getparam();
    setPipeProc();

    // Publisher
    if(m_param_rawOn)
        m_pub_image = create_publisher<sensor_msgs::msg::Image>("/image/raw", 10);
    if(m_param_compressOn)
        m_pub_imageComp = create_publisher<sensor_msgs::msg::CompressedImage>("/image/compressed", 10);
}

thetaVDriver::~thetaVDriver()
{
}

void thetaVDriver::getparam()
{
    m_param_useNVdec    = this->get_parameter("nvdec").as_bool();
    m_param_use4K       = this->get_parameter("use4k").as_bool();
    m_param_rawOn       = this->get_parameter("image/raw/on").as_bool();
    m_param_compressOn  = this->get_parameter("image/compress/on").as_bool();
    m_param_pngLevel    = this->get_parameter("image/compress/level").as_int();
}

bool thetaVDriver::turnOff(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response
)
{
    (void)request;
    (void)response;
    m_turnoff = true;
    return true;
}

void thetaVDriver::setPipeProc()
{
    if(m_param_useNVdec)
        pipe_proc = "nvdec ! glimagesink qos=false sync=false";
    else
        pipe_proc = "decodebin ! autovideosink qos=false sync=false";
}

bool thetaVDriver::gst_src_init(gst_src& srcIn)
{
    GstCaps *caps;
    char pipeline_str[MAX_PIPELINE_LEN];

    snprintf(pipeline_str, MAX_PIPELINE_LEN, "appsrc name=ap ! queue ! h264parse ! queue ! %s ", pipe_proc.c_str());

    int argc1 = 0;
    char **argv1 = nullptr;
    gst_init(&argc1, &argv1);

    srcIn.timer = g_timer_new();
    srcIn.loop = g_main_loop_new(NULL, TRUE);
    srcIn.pipeline = gst_parse_launch(pipeline_str, NULL);

    if (!srcIn.pipeline)
        return false;

    gst_pipeline_set_clock(GST_PIPELINE(srcIn.pipeline), gst_system_clock_obtain());
    srcIn.appsrc = gst_bin_get_by_name(GST_BIN(srcIn.pipeline), "ap");

    caps = gst_caps_new_simple("video/x-h264",
                               "framerate", GST_TYPE_FRACTION, 30000, 1001,
                               "stream-format", G_TYPE_STRING, "byte-stream",
                               "profile", G_TYPE_STRING, "constrained-baseline", NULL);
    gst_app_src_set_caps(GST_APP_SRC(srcIn.appsrc), caps);
    return true;
}

int thetaVDriver::findDevList(uvc_context_t* ctx)
{
    uvc_error_t res;
    uvc_device_t **devlist;

    std::cout << "[THETAV] FOUND THETA V NOW" << std::endl;
    res = thetauvc_find_devices(ctx, &devlist);
    if (res != UVC_SUCCESS) {
        uvc_perror(res, "");
        uvc_exit(ctx);
        return -1;
    }

    int idx = 0;
    printf("No : %-18s : %-10s\n", "Product", "Serial");
    while (devlist[idx] != nullptr) {
        uvc_device_descriptor_t *desc;
        if (uvc_get_device_descriptor(devlist[idx], &desc) != UVC_SUCCESS) {
            idx++;
            continue;
        }
        printf("%2d : %-18s : %-10s\n", idx, desc->product, desc->serialNumber);
        uvc_free_device_descriptor(desc);
        idx++;
    }

    uvc_free_device_list(devlist, 1);
    std::cout << idx-1 << std::endl;
    if (idx-1 < m_param_thetaIndex) return -1;
    return m_param_thetaIndex;
}

bool thetaVDriver::set4k()
{
    return m_param_use4K;
}

bool thetaVDriver::isOff()
{
    return m_turnoff;
}

void thetaVDriver::cv2sensorImg(const cv::Mat& mat, sensor_msgs::msg::Image& sensorImg)
{
    cv_bridge::CvImage bridge;
    mat.copyTo(bridge.image);
    bridge.header.frame_id = "theta";
    bridge.header.stamp = this->now();

    if(mat.type() == CV_8UC1) bridge.encoding = sensor_msgs::image_encodings::MONO8;
    else if(mat.type() == CV_8UC3) bridge.encoding = sensor_msgs::image_encodings::BGR8;
    else if(mat.type() == CV_32FC1) bridge.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    else if(mat.type() == CV_16UC1) bridge.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    else return;

    bridge.toImageMsg(sensorImg);
}

void thetaVDriver::cv2sensorImgComp(const cv::Mat& mat, sensor_msgs::msg::CompressedImage& sensorImg)
{
    sensorImg.header.frame_id = "theta";
    sensorImg.header.stamp = now();

    std::vector<int> params;
    params.resize(3, 0);

    sensor_msgs::msg::Image dummy;
    if(mat.type() == CV_8UC1) dummy.encoding = enc::MONO8;
    else if(mat.type() == CV_8UC3) dummy.encoding = enc::BGR8;
    else if(mat.type() == CV_32FC1) dummy.encoding = enc::TYPE_32FC1;
    else if(mat.type() == CV_16UC1) dummy.encoding = enc::TYPE_16UC1;
    else return;

    sensorImg.format = dummy.encoding;
    int bitDepth = enc::bitDepth(dummy.encoding);
    int numChannels = enc::numChannels(dummy.encoding);

    params[0] = cv::IMWRITE_PNG_COMPRESSION;
    params[1] = m_param_pngLevel;
    sensorImg.format += "; png compressed";

    if (((bitDepth == 16) || (bitDepth == 8)) && ((numChannels == 1) || (numChannels == 3))) {
        std::stringstream targetFormat;
        if (enc::isColor(dummy.encoding)) {
            cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
            targetFormat << "rgb" << bitDepth;
        }
        try {
            if(cv::imencode(".png", mat, sensorImg.data, params)) {
                float cRatio = (float)(mat.rows * mat.cols * mat.elemSize()) / (float)sensorImg.data.size();
                RCLCPP_DEBUG(get_logger(), "Compressed Image Transport - Codec: png, Compression Ratio: 1:%.2f (%lu bytes)", cRatio, (long unsigned int)sensorImg.data.size());
            } else {
                RCLCPP_ERROR(get_logger(), "cv::imencode (png) failed on input image");
            }
        } catch(cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
        } catch(cv::Exception& e) {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
        }
    } else {
        RCLCPP_ERROR(get_logger(), "PNG compression requires 8/16-bit, 1/3-channel images (input format is: %s)", dummy.encoding.c_str());
    }
}

void thetaVDriver::publishImage(const cv::Mat& image)
{
    if(m_param_rawOn)
    {
        sensor_msgs::msg::Image imgOut;
        cv2sensorImg(image, imgOut);
        m_pub_image->publish(imgOut);
    }

    if(m_param_compressOn)
    {
        sensor_msgs::msg::CompressedImage imgOut;
        cv2sensorImgComp(image, imgOut);
        m_pub_imageComp->publish(imgOut);
    }
}
