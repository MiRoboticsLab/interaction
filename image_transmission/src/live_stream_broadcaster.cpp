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

#include "image_transmission/live_stream_broadcaster.hpp"
#include "webrtc_conductor/video_track_input.hpp"
#include "webrtc_conductor/conductor.hpp"

namespace cyberdog
{
namespace interaction
{
LiveStreamBroadcaster::LiveStreamBroadcaster(rclcpp::Node * ros_node)
{
  I420DataInterface * source = new I420DataInterface;
  std::function<void(uint8_t *, int64_t, uint16_t, int, int)> on_frame =
    std::bind(
    &I420DataInterface::on_frame, source,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
    std::placeholders::_4, std::placeholders::_5);
  auto video_track_source = new rtc::RefCountedObject<VideoTrack>(source);
  WebRTCManager * webrtc_manager = new WebRTCManager(ros_node, video_track_source);
  webrtc_manager->SetOnFrame(on_frame);
  manager_.reset(webrtc_manager);
}

std::function<void(uint8_t *, int64_t, uint16_t, int, int)> LiveStreamBroadcaster::Init()
{
  return std::bind(
    &WebRTCManagerBase::OnFrame, manager_.get(),
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
    std::placeholders::_4, std::placeholders::_5);
}
}  // namespace interaction
}  // namespace cyberdog
