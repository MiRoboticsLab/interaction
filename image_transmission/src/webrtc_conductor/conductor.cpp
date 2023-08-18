// Copyright (c) 2022  Xiaomi Corporation
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

#include <string>
#include <memory>
#include <list>
#include <chrono>

#include "webrtc_conductor/conductor.hpp"

using namespace std::chrono_literals;

namespace cyberdog
{
namespace interaction
{
absl::optional<webrtc::SdpType> SdpTypeFromString(const std::string & type_str)
{
  if (type_str == webrtc::SessionDescriptionInterface::kOffer) {
    return webrtc::SdpType::kOffer;
  } else if (type_str == webrtc::SessionDescriptionInterface::kPrAnswer) {
    return webrtc::SdpType::kPrAnswer;
  } else if (type_str == webrtc::SessionDescriptionInterface::kAnswer) {
    return webrtc::SdpType::kAnswer;
  } else {
    return absl::nullopt;
  }
}

void DummySetSessionDescriptionObserver::OnFailure(webrtc::RTCError error)
{
  INFO_STREAM("set sdp failed:" << error.message());
  conductor_->SetDisconnected();
}

PCConductor::PCConductor(const std::string & uid, WebRTCManager * manager)
: uid_(uid), manager_(manager)
{
}

PCConductor::~PCConductor()
{
  INFO_STREAM("pc_conductor destructor");
  if (peer_connection_) {
    INFO_STREAM("pc close");
    peer_connection_->Close();
    INFO_STREAM("pc closed");
  }
  peer_connection_ = nullptr;
  INFO_STREAM("pc_conductor has been destructed");
}

void PCConductor::SetVideoParam(int height, int width, const std::string & alignment)
{
  height_ = height;
  width_ = width;
  alignment_ = alignment;
}

void PCConductor::OnReceiveSDP(webrtc::SessionDescriptionInterface * desc)
{
  std::unique_lock<std::mutex> sdp_order_lock(sdp_order_mutex_);
  if (sdp_has_received_) {
    WARN("sdp has already received, ignore this one");
    return;
  }
  peer_connection_->SetRemoteDescription(DummySetSessionDescriptionObserver::Create(this), desc);
  INFO_STREAM("set remote description");
  if (desc->GetType() == webrtc::SdpType::kOffer) {
    webrtc::PeerConnectionInterface::RTCOfferAnswerOptions options;
    options.offer_to_receive_video = 0;  // send video only
    options.offer_to_receive_audio = 0;
    peer_connection_->CreateAnswer(this, options);
    INFO_STREAM("sdp answer has been created");
  } else {
    INFO_STREAM("sdp is an answer");
  }
  sdp_has_received_ = true;
  while (!candidate_buff_.empty()) {
    peer_connection_->AddIceCandidate(candidate_buff_.front());
    INFO_STREAM("add ice candidate");
    candidate_buff_.pop();
  }
}

void PCConductor::OnReceiveCandidate(webrtc::IceCandidateInterface * candidate)
{
  std::unique_lock<std::mutex> sdp_order_lock(sdp_order_mutex_);
  if (sdp_has_received_) {
    peer_connection_->AddIceCandidate(candidate);
    INFO_STREAM("add ice candidate");
  } else {
    candidate_buff_.push(candidate);
    INFO_STREAM("sdp has not received, buffer the candidate first");
  }
}

void PCConductor::OnIceConnectionChange(
  webrtc::PeerConnectionInterface::IceConnectionState new_state)
{
  INFO_STREAM(__FUNCTION__ << new_state);
  if (new_state == webrtc::PeerConnectionInterface
    ::IceConnectionState::kIceConnectionDisconnected ||
    new_state == webrtc::PeerConnectionInterface
    ::IceConnectionState::kIceConnectionFailed ||
    new_state == webrtc::PeerConnectionInterface
    ::IceConnectionState::kIceConnectionClosed)
  {
    connection_stage_disconnect_ = true;
  } else if (new_state == 2) {  // kIceConnectionConnected
    is_connected_ = true;
    if (height_ != 0 && width_ != 0) {
      if (!manager_->CallNotifyService(true, height_, width_, alignment_)) {
        manager_->PublishError(1001, "Fail to connect to camera service", uid_);
      }
    } else {
      WARN("Not recieve video param, use default height width and alignment");
      if (!manager_->CallNotifyService(true, 720, 1280, "middle")) {
        manager_->PublishError(1001, "Fail to connect to camera service", uid_);
      }
    }
  }
}

void PCConductor::OnIceGatheringChange(webrtc::PeerConnectionInterface::IceGatheringState new_state)
{
  INFO_STREAM(__FUNCTION__ << new_state);
  if (new_state == webrtc::PeerConnectionInterface
    ::IceGatheringState::kIceGatheringComplete)
  {
    gather_stage_complete_ = true;
  }
}

void PCConductor::OnSignalingChange(webrtc::PeerConnectionInterface::SignalingState new_state)
{
  INFO_STREAM(__FUNCTION__ << new_state);
}

int PCConductor::IsConnected()
{
  if (!is_connected_) {
    ++disconnected_counts_;
  } else {
    disconnected_counts_ = -1;
  }
  return disconnected_counts_;
}

void PCConductor::OnSuccess(webrtc::SessionDescriptionInterface * desc)
{
  INFO_STREAM(__FUNCTION__);
  peer_connection_->SetLocalDescription(DummySetSessionDescriptionObserver::Create(this), desc);
  manager_->PublishSdp(desc, uid_);
}

void PCConductor::OnIceCandidate(const webrtc::IceCandidateInterface * candidate)
{
  INFO_STREAM(__FUNCTION__);
  manager_->PublishCandidate(candidate, uid_);
}

void PCConductor::Start()
{
  webrtc::PeerConnectionInterface::RTCOfferAnswerOptions options;
  options.offer_to_receive_video = 0;  // only send
  options.offer_to_receive_audio = 0;
  peer_connection_->CreateOffer(
    this, options);
}

WebRTCManager::WebRTCManager(
  rclcpp::Node * ros_node,
  rtc::scoped_refptr<webrtc::VideoTrackSource> video_source)
: ros_node_(ros_node), video_source_(video_source)
{
  // ros initialization
  callback_group_ = ros_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_option;
  rclcpp::PublisherOptions pub_option;
  sub_option.callback_group = callback_group_;
  pub_option.callback_group = callback_group_;
  signal_publisher_ = ros_node_->create_publisher<std_msgs::msg::String>(
    "img_trans_signal_out", 200, pub_option);
  signal_subscription_ = ros_node_->create_subscription<std_msgs::msg::String>(
    "img_trans_signal_in", 200, std::bind(
      &WebRTCManager::msgCallback,
      this, std::placeholders::_1), sub_option);
  monitor_timer_ = ros_node_->create_wall_timer(
    std::chrono::duration<int, std::milli>(100), std::bind(&WebRTCManager::timerCallback, this),
    callback_group_);
  monitor_timer_->cancel();
  status_notify_client_ = ros_node_->create_client<protocol::srv::CameraService>("camera_service");
  // webrtc initialization
  signal_thread_->SetName("pc_signal_thread", nullptr);
  signal_thread_->Start();
  pc_config_.sdp_semantics = webrtc::SdpSemantics::kUnifiedPlan;
  pc_config_.enable_dtls_srtp = true;
  webrtc::PeerConnectionInterface::IceServer server;
  server.uri = GetPeerConnectionString();
  pc_config_.servers.push_back(server);
  peer_connection_factory_ = webrtc::CreatePeerConnectionFactory(
    nullptr /* network_thread */, nullptr /* worker_thread */,
    signal_thread_.get() /* signaling_thread */, nullptr /* default_adm */,
    webrtc::CreateBuiltinAudioEncoderFactory(),
    webrtc::CreateBuiltinAudioDecoderFactory(),
    webrtc::CreateBuiltinVideoEncoderFactory(),
    webrtc::CreateBuiltinVideoDecoderFactory(),
    nullptr /* audio_mixer */, nullptr /* audio_processing */);
  INFO_STREAM("WebRTCManager has been created");
}

WebRTCManager::~WebRTCManager()
{
  pc_conductors_.clear();
  INFO_STREAM("clear peer_connections");
  peer_connection_factory_ = nullptr;
  INFO_STREAM("factory destructed");
  signal_thread_->Stop();
  INFO_STREAM("signal thread stoped");
  signal_thread_.reset();
  INFO_STREAM("signal thread destructed");
  {
    std::unique_lock<std::mutex> video_lock(video_mutex_);
    video_source_ = nullptr;
    INFO_STREAM("video_track_ destructed");
  }
}

void WebRTCManager::StartOffer()
{
  if (addNewPC("default_uid")) {
    signal_thread_->Invoke<void>(
      RTC_FROM_HERE, [this]()
      {
        pc_conductors_["default_uid"]->Start();
      });
  }
}

void WebRTCManager::parseMsg(const std_msgs::msg::String::SharedPtr msg, const std::string & uid)
{
  Json::Value root, core_msg;
  Json::Reader reader;
  reader.parse(msg->data, root);
  webrtc::SdpParseError error;
  if (root.isMember("c_sdp")) {
    INFO_STREAM("It's a candidate");
    std::string sdp_mid;
    int sdp_mlineindex = 0;
    std::string sdp;
    core_msg = root["c_sdp"];
    if (!rtc::GetStringFromJsonObject(core_msg, "sdpMid", &sdp_mid) ||
      !rtc::GetIntFromJsonObject(core_msg, "sdpMLineIndex", &sdp_mlineindex) ||
      !rtc::GetStringFromJsonObject(core_msg, "candidate", &sdp))
    {
      WARN_STREAM("Not able to parse ice candicate message.");
      return;
    }
    std::unique_ptr<webrtc::IceCandidateInterface> candidate(
      webrtc::CreateIceCandidate(sdp_mid, sdp_mlineindex, sdp, &error));
    if (candidate) {
      INFO_STREAM("create candidate");
      pc_conductors_[uid]->OnReceiveCandidate(candidate.release());
    } else {
      WARN_STREAM("Not able to create candidate");
    }
  } else if (root.isMember("answer_sdp") || root.isMember("offer_sdp")) {
    INFO_STREAM("It's an sdp");
    std::string type_str;
    std::string sdp_str;
    core_msg = root.isMember("answer_sdp") ? root["answer_sdp"] : root["offer_sdp"];
    if (!rtc::GetStringFromJsonObject(core_msg, "type", &type_str)) {
      WARN_STREAM("Not able to parse sdp type.");
      return;
    }
    absl::optional<webrtc::SdpType> maybe_type = SdpTypeFromString(type_str);
    if (!maybe_type) {
      WARN_STREAM("Unknown SDP type: " << type_str);
      return;
    }
    if (!rtc::GetStringFromJsonObject(core_msg, "sdp", &sdp_str)) {
      WARN_STREAM("Not able to parse received session description message.");
      return;
    }
    std::unique_ptr<webrtc::SessionDescriptionInterface> session_description =
      webrtc::CreateSessionDescription(*maybe_type, sdp_str, &error);
    if (session_description) {
      INFO_STREAM("create session description");
      pc_conductors_[uid]->OnReceiveSDP(session_description.release());
    } else {
      WARN_STREAM("Not able to create sdp");
    }
  } else if (root.isMember("error")) {
    DEBUG_STREAM("error msg");
  }
}

void WebRTCManager::msgCallback(const std_msgs::msg::String::SharedPtr msg)
{
  INFO_STREAM("Got msg: " << msg->data);
  Json::Value root;
  Json::Reader reader;
  std::string uid;
  if (!reader.parse(msg->data, root)) {
    WARN_STREAM("json reader not able to parse.");
    return;
  }
  if (!rtc::GetStringFromJsonObject(root, "uid", &uid) || uid.empty()) {
    WARN_STREAM("There is no uid in this msg, set default_uid");
    uid = "default_uid";
  }
  int h = 0, w = 0;
  std::string alg;
  bool set_options = false;
  if (root.isMember("offer_sdp") || root.isMember("answer_sdp")) {
    set_options = true;
    if (root.isMember("height") && root.isMember("width") && root.isMember("alignment")) {
      if (!rtc::GetIntFromJsonObject(root, "height", &h) ||
        !rtc::GetIntFromJsonObject(root, "width", &w) ||
        !rtc::GetStringFromJsonObject(root, "alignment", &alg))
      {
        WARN_STREAM("Not able to parse param information.");
        return;
      }
    }
  } else if (!root.isMember("c_sdp")) {
    WARN("Stop signal");
    if (killPC(uid)) {
      PublishStop(uid);
    } else {
      WARN("uid: %s is not connected.", uid.c_str());
    }
    return;
  }
  std::unique_lock<std::shared_mutex> pc_lock(pc_mutex_);  // write lock
  bool got_pc = false;
  if (set_options) {
    if (h == 0 || w == 0) {
      h = 720;
      w = 1280;
      alg = "middle";
    } else {
      h = 640 * (static_cast<double>(h) / static_cast<double>(w));
      w = 640;
    }
    got_pc = addNewPC(uid, h, w, alg);
  } else {
    got_pc = addNewPC(uid);
  }
  if (got_pc) {
    parseMsg(msg, uid);
  } else {
    WARN_STREAM("Fail to create a new PC for uid : " << uid);
  }
}

bool WebRTCManager::addNewPC(
  const std::string & uid, int height, int width, const std::string & alignment)
{
  if (pc_conductors_.find(uid) != pc_conductors_.end()) {
    INFO_STREAM("Got an existed pc.");
  } else {
    bool no_conductors = pc_conductors_.empty();
    pc_conductors_[uid] = new rtc::RefCountedObject<PCConductor>(
      uid,
      this);
    auto new_pc = peer_connection_factory_->CreatePeerConnection(
      pc_config_, nullptr, nullptr, pc_conductors_[uid].get());
    if (!new_pc) {
      WARN_STREAM("Failed to create PeerConnection!");
      pc_conductors_.erase(uid);
      return false;
    }
    auto video_track = peer_connection_factory_->CreateVideoTrack("video_label", video_source_);
    auto error = new_pc->AddTrack(video_track, {"stream_id"});
    if (!error.ok()) {
      WARN_STREAM(
        "Failed to add video track to PeerConnection: " <<
          error.error().message());
      pc_conductors_.erase(uid);
      return false;
    }
    pc_conductors_[uid]->SetPC(new_pc);
    INFO_STREAM("A new pc is created.");
    monitor_timer_->reset();
  }
  if (height != 0 && width != 0) {
    pc_conductors_[uid]->SetVideoParam(height, width, alignment);
  }
  return true;
}

bool WebRTCManager::killPC(const std::string & uid)
{
  int closed_counts = pc_conductors_.erase(uid);
  if (closed_counts != 0 && pc_conductors_.empty()) {
    if (!CallNotifyService(false)) {
      PublishError(1001, "Fail to connect to camera service", uid);
    }
  }
  return closed_counts != 0 ? true : false;
}

bool WebRTCManager::PublishSdp(
  webrtc::SessionDescriptionInterface * sdp_ptr,
  const std::string & uid)
{
  std::string sdp_str;
  if (!sdp_ptr->ToString(&sdp_str)) {
    WARN_STREAM("Failed to serialize sdp");
    return false;
  }
  Json::Value jmessage, jm_wrapper;
  std::string type_str = webrtc::SdpTypeToString(sdp_ptr->GetType());
  jmessage["type"] = type_str;
  jmessage["sdp"] = sdp_str;
  if (type_str == "offer") {
    jm_wrapper["offer_sdp"] = jmessage;
  } else {
    jm_wrapper["answer_sdp"] = jmessage;
  }
  jm_wrapper["uid"] = uid;
  std_msgs::msg::String sdp_msg;
  Json::StreamWriterBuilder factory;
  sdp_msg.data = Json::writeString(factory, jm_wrapper);
  signal_publisher_->publish(sdp_msg);
  INFO_STREAM("send sdp");
  INFO_STREAM(sdp_msg.data);
  return true;
}

bool WebRTCManager::PublishCandidate(
  const webrtc::IceCandidateInterface * candidate,
  const std::string & uid)
{
  std::string candidate_str;
  Json::Value jmessage, jm_wrapper;
  jmessage["sdpMid"] = candidate->sdp_mid();
  jmessage["sdpMLineIndex"] = candidate->sdp_mline_index();
  jm_wrapper["uid"] = uid;
  if (!candidate->ToString(&candidate_str)) {
    WARN_STREAM("Failed to serialize candidate");
    return false;
  }
  jmessage["candidate"] = candidate_str;
  jm_wrapper["c_sdp"] = jmessage;
  std_msgs::msg::String candidate_msg;
  Json::StreamWriterBuilder factory;
  candidate_msg.data = Json::writeString(factory, jm_wrapper);
  signal_publisher_->publish(candidate_msg);
  INFO_STREAM("send candidate");
  INFO_STREAM(candidate_msg.data);
  return true;
}

void WebRTCManager::PublishStop(const std::string & uid)
{
  Json::Value jm_wrapper;
  jm_wrapper["is_closed"] = true;
  jm_wrapper["uid"] = uid;
  std_msgs::msg::String stop_msg;
  Json::StreamWriterBuilder factory;
  stop_msg.data = Json::writeString(factory, jm_wrapper);
  signal_publisher_->publish(stop_msg);
  INFO_STREAM("send stop response");
}

void WebRTCManager::PublishError(int error_code, const std::string & msg, const std::string & uid)
{
  Json::Value jmessage, jm_wrapper;
  jmessage["code"] = error_code;
  jmessage["msg"] = msg;
  jm_wrapper["error"] = jmessage;
  if (!uid.empty()) {
    jm_wrapper["uid"] = uid;
  }
  std_msgs::msg::String error_msg;
  Json::StreamWriterBuilder factory;
  error_msg.data = Json::writeString(factory, jm_wrapper);
  signal_publisher_->publish(error_msg);
  WARN_STREAM("Send error code" << error_code << " msg: " << msg);
}

void WebRTCManager::timerCallback()
{
  std::list<std::string> peer_uid_list;
  {
    std::shared_lock<std::shared_mutex> pc_lock(pc_mutex_);
    for (auto & conductor : pc_conductors_) {
      if (conductor.second->IsDisconnected()) {
        peer_uid_list.push_back(conductor.first);
      } else if (conductor.second->IsConnected() == 99) {
        PublishError(1002, "ICE connection timeout", conductor.first);
      } else if (conductor.second->IsConnected() > 100) {
        ERROR_STREAM(
          "PeerConnection " << conductor.first <<
            " failed to connect with another peer. Destructing!");
        peer_uid_list.push_back(conductor.first);
      }
    }
  }
  if (peer_uid_list.empty()) {
    return;
  }
  std::unique_lock<std::shared_mutex> pc_lock(pc_mutex_);
  for (auto & uid : peer_uid_list) {
    killPC(uid);
  }
}

void WebRTCManager::OnFrame(
  uint8_t * i420_ptr, int64_t time_stamp, uint16_t frame_id,
  int height, int width)
{
  std::unique_lock<std::mutex> video_lock(video_mutex_);
  if (video_source_) {
    on_frame_(i420_ptr, time_stamp, frame_id, height, width);
  } else {
    WARN_STREAM("video source has been destructed!");
  }
}

bool WebRTCManager::CallNotifyService(
  bool activate, int height, int width, const std::string & alignment)
{
  std::unique_lock<std::mutex> lock(service_mutex_);
  if (activate) {
    if (is_streaming_) {
      return true;
    }
  } else {
    monitor_timer_->cancel();
  }
  if (!use_camera_service_) {
    return false;
  }
  if (!status_notify_client_->wait_for_service(1s)) {
    WARN_STREAM("camera_service is not activated.");
    use_camera_service_ = false;
    return false;
  }
  auto request = std::make_shared<protocol::srv::CameraService::Request>();
  request->command = activate ?
    protocol::srv::CameraService::Request::START_LIVE_STREAM :
    protocol::srv::CameraService::Request::STOP_LIVE_STREAM;
  if (activate) {
    request->height = height;
    request->width = width;
    request->args = alignment;
  }
  auto result = status_notify_client_->async_send_request(request);
  is_streaming_ = activate;
  return true;
}
}  // namespace interaction
}  // namespace cyberdog
