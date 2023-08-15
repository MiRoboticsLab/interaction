// Copyright (c) 2022 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WEBRTC_CONDUCTOR__CONDUCTOR_HPP_
#define WEBRTC_CONDUCTOR__CONDUCTOR_HPP_

#include <unistd.h>
#include <shared_mutex>
#include <memory>
#include <iostream>
#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <queue>

#include "jsoncpp/json/json.h"

#include "api/video/i420_buffer.h"
#include "api/create_peerconnection_factory.h"
#include "api/audio_codecs/audio_decoder_factory.h"
#include "api/audio_codecs/audio_encoder_factory.h"
#include "api/audio_codecs/builtin_audio_decoder_factory.h"
#include "api/audio_codecs/builtin_audio_encoder_factory.h"
#include "api/video_codecs/builtin_video_decoder_factory.h"
#include "api/video_codecs/builtin_video_encoder_factory.h"
#include "api/video_codecs/video_decoder_factory.h"
#include "api/video_codecs/video_encoder_factory.h"
#include "media/base/video_adapter.h"
#include "media/base/video_broadcaster.h"
#include "modules/video_coding/codecs/nvidia/NvVideoEncoderFactory.h"
#include "pc/video_track_source.h"
#include "examples/peerconnection/client/defaults.h"
#include "rtc_base/strings/json.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "protocol/srv/camera_service.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

#include "image_transmission/live_stream_broadcaster.hpp"

namespace cyberdog
{
namespace interaction
{
class PCConductor;

class DummySetSessionDescriptionObserver
  : public webrtc::SetSessionDescriptionObserver
{
public:
  explicit DummySetSessionDescriptionObserver(PCConductor * conductor)
  : conductor_(conductor) {}
  static DummySetSessionDescriptionObserver * Create(PCConductor * conductor)
  {
    return new rtc::RefCountedObject<DummySetSessionDescriptionObserver>(conductor);
  }
  void OnSuccess() override {}
  void OnFailure(webrtc::RTCError error) override;

private:
  PCConductor * conductor_;
};

class WebRTCManager;

class PCConductor : public webrtc::PeerConnectionObserver,
  public webrtc::CreateSessionDescriptionObserver
{
public:
  PCConductor(const std::string & uid, WebRTCManager * manager);
  ~PCConductor() override;
  void SetPC(rtc::scoped_refptr<webrtc::PeerConnectionInterface> peer_connection)
  {
    peer_connection_ = peer_connection;
  }
  void SetVideoParam(int height, int width, const std::string & alignment);
  void OnReceiveSDP(webrtc::SessionDescriptionInterface * desc);
  void OnReceiveCandidate(webrtc::IceCandidateInterface * candidate);
  bool IsDisconnected()
  {
    return gather_stage_complete_ && connection_stage_disconnect_;
  }
  void SetDisconnected()
  {
    gather_stage_complete_ = true;
    connection_stage_disconnect_ = true;
  }
  void Start();
  int IsConnected();

protected:
  void OnSuccess(webrtc::SessionDescriptionInterface * desc) override;
  void OnIceCandidate(const webrtc::IceCandidateInterface * candidate) override;
  void OnFailure(webrtc::RTCError error) override
  {
    WARN_STREAM("create sdp error: " << error.message());
  }
  void OnSignalingChange(webrtc::PeerConnectionInterface::SignalingState new_state);
  void OnRenegotiationNeeded() override
  {
    DEBUG_STREAM(__FUNCTION__);
  }
  void OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) override {}
  void OnIceGatheringChange(webrtc::PeerConnectionInterface::IceGatheringState new_state) override;
  void OnIceConnectionChange(
    webrtc::PeerConnectionInterface::IceConnectionState new_state) override;

private:
  rtc::scoped_refptr<webrtc::PeerConnectionInterface> peer_connection_;
  std::string uid_;
  WebRTCManager * manager_;
  std::atomic_bool gather_stage_complete_ {false};
  std::atomic_bool connection_stage_disconnect_ {false};
  std::queue<webrtc::IceCandidateInterface *> candidate_buff_;
  bool sdp_has_received_ {false};
  mutable std::mutex sdp_order_mutex_;
  std::atomic_bool is_connected_ {false};
  int disconnected_counts_ {0};
  int height_ {0}, width_ {0};
  std::string alignment_;

  LOGGER_MINOR_INSTANCE("PCConductor");
};

class WebRTCManager : public WebRTCManagerBase
{
public:
  WebRTCManager(
    rclcpp::Node * ros_node,
    rtc::scoped_refptr<webrtc::VideoTrackSource> video_source);
  ~WebRTCManager() override;
  void StartOffer();
  bool CallNotifyService(
    bool activate, int height = 1280, int width = 720, const std::string & alignment = "middle");
  bool PublishSdp(
    webrtc::SessionDescriptionInterface * sdp_ptr,
    const std::string & uid);
  bool PublishCandidate(
    const webrtc::IceCandidateInterface * candidate,
    const std::string & uid);
  void PublishStop(const std::string & uid);
  void PublishError(int error_code, const std::string & msg, const std::string & uid = "");
  void SetOnFrame(std::function<void(uint8_t *, int64_t, uint16_t, int, int)> on_frame)
  {
    on_frame_ = on_frame;
  }
  void OnFrame(
    uint8_t * i420_ptr, int64_t time_stamp, uint16_t frame_id,
    int height, int width) override;

private:
  void msgCallback(const std_msgs::msg::String::SharedPtr msg);
  void timerCallback();
  void parseMsg(const std_msgs::msg::String::SharedPtr msg, const std::string & uid);
  bool addNewPC(
    const std::string & uid, int height = 0, int width = 0,
    const std::string & alignment = "middle");  // add a new peer_connection to the map
  bool killPC(const std::string & uid);
  rclcpp::Node * ros_node_;  // ros2 node to create signal publisher, subscriber and service client
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr signal_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signal_subscription_;
  rclcpp::Client<protocol::srv::CameraService>::SharedPtr status_notify_client_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;  // monitor timer for killing conductors
  std::unique_ptr<rtc::Thread> signal_thread_ {rtc::Thread::Create()};
  rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> peer_connection_factory_;
  webrtc::PeerConnectionInterface::RTCConfiguration pc_config_;
  std::map<std::string, rtc::scoped_refptr<PCConductor>> pc_conductors_;
  mutable std::shared_mutex pc_mutex_;
  mutable std::mutex video_mutex_;
  rtc::scoped_refptr<webrtc::VideoTrackSource> video_source_;
  std::function<void(uint8_t *, int64_t, uint16_t, int, int)> on_frame_;
  bool use_camera_service_ {true};
  bool is_streaming_ {false};
  mutable std::mutex service_mutex_;

  LOGGER_MINOR_INSTANCE("WebRTCManager");
};  // class WebRTCConductor
}  // namespace interaction
}  // namespace cyberdog

#endif  // WEBRTC_CONDUCTOR__CONDUCTOR_HPP_
