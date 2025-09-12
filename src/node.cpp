#include "driver.h"
#include <rclcpp/rclcpp.hpp>
#include <unistd.h> // usleep

gst_src gsrc;
thetaVDriver* driverCLS;
H264Decoder HDecoder;

gboolean gst_bus_cb(GstBus* bus, GstMessage* message, gpointer data);
void cb(uvc_frame_t* frame, void* ptr);

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // ノード作成
    driverCLS = new thetaVDriver();

    rclcpp::spin_some(driverCLS->get_node_base_interface());

    if(!driverCLS->gst_src_init(gsrc)) return -1;

    uvc_error_t res;
    GstBus* bus;
    uvc_device_t* dev;
    uvc_device_handle_t* devh;
    uvc_stream_ctrl_t ctrl;
    uvc_context_t* ctx;

    bus = gst_pipeline_get_bus(GST_PIPELINE(gsrc.pipeline));
    gsrc.bus_watch_id = gst_bus_add_watch(bus, gst_bus_cb, nullptr);
    gst_object_unref(bus);

    RCLCPP_INFO(driverCLS->get_logger(), "[THETAV] DEVICE CONNECTED");

    res = uvc_init(&ctx, nullptr);
    if (res != UVC_SUCCESS) {
        RCLCPP_ERROR(driverCLS->get_logger(), "[THETAV] UVC INIT FAIL");
        return -1;
    }

    int selected = driverCLS->findDevList(ctx);
    if (selected < 0) {
        RCLCPP_ERROR(driverCLS->get_logger(), "[THETAV] CANNOT SELECT DEVICE");
        return -1;
    }

    RCLCPP_INFO(driverCLS->get_logger(), "[THETAV] SELECT NO %d", selected);
    thetauvc_find_device(ctx, &dev, selected);
    uvc_open(dev, &devh);
    RCLCPP_INFO(driverCLS->get_logger(), "[THETAV] DEVICE CONNECTED");

    gsrc.framecount = 0;

    RCLCPP_INFO(driverCLS->get_logger(), "[THETAV] SIZE SETTING NOW");
    if(driverCLS->set4k()){
        thetauvc_get_stream_ctrl_format_size(devh, THETAUVC_MODE_UHD_2997, &ctrl);
        RCLCPP_INFO(driverCLS->get_logger(), "[THETAV] SIZE SET TO 4K");
    } else {
        thetauvc_get_stream_ctrl_format_size(devh, THETAUVC_MODE_FHD_2997, &ctrl);
        RCLCPP_INFO(driverCLS->get_logger(), "[THETAV] SIZE SET TO 2K");
    }

    RCLCPP_INFO(driverCLS->get_logger(), "[THETAV] START STREAMING");
    res = uvc_start_streaming(devh, &ctrl, cb, &gsrc, 0);
    if (res == UVC_SUCCESS) {
        g_main_loop_run(gsrc.loop);

        // Stop streaming
        uvc_stop_streaming(devh);
        gst_element_set_state(gsrc.pipeline, GST_STATE_NULL);
        g_source_remove(gsrc.bus_watch_id);
        g_main_loop_unref(gsrc.loop);
    }

    rclcpp::shutdown();
    return 0;
}

gboolean gst_bus_cb(GstBus* bus, GstMessage* message, gpointer data)
{
    GError* err;
    gchar* dbg;

    switch (GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_ERROR:
            gst_message_parse_error(message, &err, &dbg);
            g_print("Error: %s\n", err->message);
            g_error_free(err);
            g_free(dbg);
            g_main_loop_quit(gsrc.loop);
            break;
        default:
            break;
    }
    return TRUE;
}

void cb(uvc_frame_t* frame, void* ptr)
{
    if(frame->data_bytes > 5000) {
        cv::Mat cvimg;
        if(HDecoder.decode((uchar*)frame->data, frame->data_bytes, cvimg)) {
            driverCLS->publishImage(cvimg);
        }
    }

    rclcpp::spin_some(driverCLS->get_node_base_interface());
    rclcpp::Rate rate(100);
    rate.sleep();

    gst_src* s = static_cast<gst_src*>(ptr);
    s->framecount++;
    usleep(1);

    if(driverCLS->isOff()) {
        RCLCPP_INFO(driverCLS->get_logger(), "STOP PUB");
        rclcpp::shutdown();
        exit(0);
    }
}
