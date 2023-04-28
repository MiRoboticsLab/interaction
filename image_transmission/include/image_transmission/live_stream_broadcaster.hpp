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

#ifndef IMAGE_TRANSMISSION__LIVE_STREAM_BROADCASTER_HPP_
#define IMAGE_TRANSMISSION__LIVE_STREAM_BROADCASTER_HPP_

#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"

namespace cyberdog
{
namespace interaction
{
class WebRTCManagerBase
{
public:
  virtual ~WebRTCManagerBase() {}
  virtual void OnFrame(
    uint8_t * i420_ptr, int64_t time_stamp, uint16_t frame_id,
    int height, int width) = 0;
};  // WebRTCManagerBase

class LiveStreamBroadcaster
{
public:
  explicit LiveStreamBroadcaster(rclcpp::Node * ros_node);
  std::function<void(uint8_t *, int64_t, uint16_t, int, int)> Init();

private:
  std::unique_ptr<WebRTCManagerBase> manager_ {nullptr};
};  // LiveStreamBroadcaster
}  // namespace interaction
}  // namespace cyberdog
#endif  // IMAGE_TRANSMISSION__LIVE_STREAM_BROADCASTER_HPP_
