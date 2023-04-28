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

#ifndef WEBRTC_CONDUCTOR__VIDEO_TRACK_INPUT_HPP_
#define WEBRTC_CONDUCTOR__VIDEO_TRACK_INPUT_HPP_

#include <memory>
#include <functional>
#include <chrono>
#include <iostream>
#include <vector>

#include "api/media_stream_interface.h"
#include "api/video/i420_buffer.h"
#include "api/scoped_refptr.h"
#include "api/video/video_source_interface.h"
#include "pc/video_track_source.h"
#include "media/base/video_broadcaster.h"
#include "media/base/video_adapter.h"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace interaction
{
using I420Frame = std::shared_ptr<std::vector<uint8_t>>;
using I420FrameObserver = std::function<void (I420Frame)>;

class I420DataInterface : public rtc::VideoSourceInterface<webrtc::VideoFrame>
{
public:
  I420DataInterface()
  {}

  void on_frame(
    uint8_t * i420_ptr, int64_t time_stamp = 0, uint16_t frame_id = 0,
    int height = 0, int width = 0)
  {
    if (height == 0 || width == 0) {
      ERROR("Stream data with 0 height or 0 width");
      return;
    }
    static int i = 0;
    i++;
    rtc::scoped_refptr<webrtc::I420Buffer> buffer =
      webrtc::I420Buffer::Copy(
      width, height,
      i420_ptr, width,
      i420_ptr + width * height, (width + 1) / 2,
      i420_ptr + width * height + width * height / 4, (width + 1) / 2);
    webrtc::VideoFrame captureFrame = webrtc::VideoFrame::Builder()
      .set_video_frame_buffer(buffer)
      .set_timestamp_us(time_stamp / 1000)  // nm to um
      .set_rotation(webrtc::kVideoRotation_0)
      .set_id(frame_id)
      .build();
    broadcaster_.OnFrame(captureFrame);
  }
  void AddOrUpdateSink(
    rtc::VideoSinkInterface<webrtc::VideoFrame> * sink,
    const rtc::VideoSinkWants & wants) override
  {
    broadcaster_.AddOrUpdateSink(sink, wants);
    UpdateVideoAdapter();
    INFO_STREAM(__FUNCTION__);
  }
  void RemoveSink(rtc::VideoSinkInterface<webrtc::VideoFrame> * sink) override
  {
    broadcaster_.RemoveSink(sink);
    UpdateVideoAdapter();
    INFO_STREAM(__FUNCTION__);
  }

  void UpdateVideoAdapter()
  {
    rtc::VideoSinkWants wants = broadcaster_.wants();
    video_adapter_.OnResolutionFramerateRequest(
      wants.target_pixel_count, wants.max_pixel_count, wants.max_framerate_fps);
  }

private:
  rtc::VideoBroadcaster broadcaster_;
  cricket::VideoAdapter video_adapter_;

  LOGGER_MINOR_INSTANCE("I420DataInterface");
};  // class I420DataInterface

class VideoTrack : public webrtc::VideoTrackSource
{
public:
  explicit VideoTrack(I420DataInterface * source)
  : webrtc::VideoTrackSource(false)
  {
    my_source_.reset(source);
  }
  ~VideoTrack()
  {
    INFO_STREAM(__FUNCTION__);
  }
  bool is_screencast() const override
  {
    return true;
  }
  absl::optional<bool> needs_denoising() const override
  {
    return absl::optional<bool>(true);
  }
  bool GetStats(Stats * stats) override
  {
    return true;
  }

protected:
  rtc::VideoSourceInterface<webrtc::VideoFrame> * source() override
  {
    return my_source_.get();
  }

private:
  std::unique_ptr<I420DataInterface> my_source_;

  LOGGER_MINOR_INSTANCE("VideoTrack");
};  // class VideoTrack
}  // namespace interaction
}  // namespace cyberdog
#endif  // WEBRTC_CONDUCTOR__VIDEO_TRACK_INPUT_HPP_
