#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "image_transmission/live_stream_broadcaster.hpp"
#include "i420_creator.h"
#include "cyberdog_common/cyberdog_log.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    LOGGER_MAIN_INSTANCE("test_webrtc_video_sender");
    auto ros_node = std::make_shared<rclcpp::Node>("test_webrtc_video_sender");
    //auto vt = new rtc::RefCountedObject<VideoTrack>();
    //cyberdog::WebRTCManager webrtc_manager(ros_node, new rtc::RefCountedObject<VideoTrack>());
    //std::unique_ptr<cyberdog::WebRTCManager> webrtc_manager = std::make_unique<cyberdog::WebRTCManager>(ros_node, vt);
    //vt = nullptr;
    cyberdog::interaction::LiveStreamBroadcaster broadcaster(ros_node.get(), 1280, 960);
    I420Creator i420_creator;
    i420_creator.set_resolution(1280, 960);
    i420_creator.SetOnFrame(broadcaster.Init());
    i420_creator.run(50);
    INFO_STREAM("node start");
    rclcpp::spin(ros_node);
    std::cout << "ros_node terminated!" << std::endl;
    std::cout << "webrtc_manager has been destructed" << std::endl;
    return 0;
}
