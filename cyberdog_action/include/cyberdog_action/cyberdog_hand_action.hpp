// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef CYBERDOG_ACTION__CYBERDOG_HAND_ACTION_HPP_
#define CYBERDOG_ACTION__CYBERDOG_HAND_ACTION_HPP_
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <thread>
#include <vector>
#include <condition_variable>
#include <string>
#include <memory>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <chrono>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <deque>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nvinference/logging.hpp"
#include "protocol/srv/gesture_action_control.hpp"
#include "protocol/msg/gesture_action_result.hpp"
#include "protocol/msg/audio_play_extend.hpp"
#include "protocol/srv/camera_service.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cyberdog_common/cyberdog_model.hpp"
#include "protocol/msg/connector_status.hpp"
#include "protocol/msg/audio_play.hpp"


#ifndef CHECK
#define CHECK(callstr) \
  { \
    cudaError_t error_code = callstr; \
    if (error_code != cudaSuccess) { \
      std::cerr << "CUDA error " << error_code << " at " << __FILE__ << ":" << __LINE__; \
      assert(0); \
    } \
  }
#endif
namespace cyberdog
{
namespace interaction
{
struct CameraControl
{
  uint8_t command;
  int32_t timeout;
  bool thread_work;
  std::mutex mtx;
  std::condition_variable cond;
  CameraControl()
  {
    command = 1;
    timeout = 0;
    thread_work = false;
  }
};

struct ConfigParams
{
  uint32_t queue_size;
  int frames_number;
  bool refine_output;
  float_t softmax_thres;
  std::string engine_path;
  int device;
  int fps;
  bool is_specified;
  std::string version;
};

struct InferControl
{
  bool thread_work;
  std::mutex mtx;
  std::condition_variable cond;
  InferControl()
  {
    thread_work = false;
  }
};

enum class Gesture_ori
{
  Doing_other_things,
  Drumming_Fingers,
  No_gesture,
  Pulling_Hand_In,
  Pulling_Two_Fingers_In,
  Pushing_Hand_Away,
  Pushing_Two_Fingers_Away,
  Rolling_Hand_Backward,
  Rolling_Hand_Forward,
  Shaking_Hand,
  Sliding_Two_Fingers_Down,
  Sliding_Two_Fingers_Left,
  Sliding_Two_Fingers_Right,
  Sliding_Two_Fingers_Up,
  Stop_Sign,
  Swiping_Down,
  Swiping_Left,
  Swiping_Right,
  Swiping_Up,
  Thumb_Down,
  Thumb_Up,
  Turning_Hand_Clockwise,
  Turning_Hand_Counterclockwise,
  Zooming_In_With_Full_Hand,
  Zooming_In_With_Two_Fingers,
  Zooming_Out_With_Full_Hand,
  Zooming_Out_With_Two_Fingers
};

enum class Gesture_cut
{
  No_gesture,
  Pulling_Hand_Or_Two_Fingers_In,
  Pushing_Hand_Or_Two_Fingers_Away,
  Sliding_Hand_Two_Fingers_Up,
  Sliding_Hand_Two_Fingers_Down,
  Sliding_Hand_Two_Fingers_Left,
  Sliding_Hand_Two_Fingers_Right,
  Stop_Sign,
  Thumb_Up,
  Zooming_In_With_Hand_Or_Two_Fingers,
  Zooming_Out_With_Hand_Or_Two_Fingers,
  Thumb_Down
};

class gesture
{
  using GestureActionSrv = protocol::srv::GestureActionControl;
  using GestureActionMsg = protocol::msg::GestureActionResult;
  using CameraSrv = protocol::srv::CameraService;
  using CameraMsg = sensor_msgs::msg::Image;
  using WifiMsg = protocol::msg::ConnectorStatus;
  using AudioMsg = protocol::msg::AudioPlay;                     /*!< 语音消息:离线 */
  const int INPUT_H = 224;
  const int INPUT_W = 224;
  const int OUTPUT_SIZE = 27;

public:
  explicit gesture(const std::string & name);
  bool Init();
  void Run();

  ~gesture();

private:
  rclcpp::Node::SharedPtr node_ptr_{nullptr};
  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::Service<GestureActionSrv>::SharedPtr gesture_action_service_{nullptr};
  rclcpp::Subscription<CameraMsg>::SharedPtr camera_sub_{nullptr};
  rclcpp::Client<CameraSrv>::SharedPtr camera_client_{nullptr};
  rclcpp::Publisher<GestureActionMsg>::SharedPtr gesture_action_pub_{nullptr};
  rclcpp::Subscription<protocol::msg::ConnectorStatus>
  ::SharedPtr connector_sub_{nullptr};
  rclcpp::Publisher<AudioMsg>::SharedPtr audio_pub_ {nullptr};
  std::shared_ptr<AudioMsg> audio_msg_ptr_;
  Logger gLogger;
  nvinfer1::IExecutionContext * context{nullptr};
  nvinfer1::IRuntime * runtime{nullptr};
  nvinfer1::ICudaEngine * engine{nullptr};
  CameraControl ai_camera;
  InferControl img_inference;
  std::shared_ptr<std::thread> camera_thread_;
  std::shared_ptr<std::thread> inference_thread_;
  cv::Mat img;
  cv_bridge::CvImageConstPtr cv_ptr;
  void * buffers[22];
  cudaStream_t stream;
  int inputIndex[11];
  int outputIndex[11];
  std::deque<int> history;
  float max_probability = 0.0;
  bool camera_is_open = false;
  int begin_time = 0;
  int algo_duration = 0;
  int stop_time = 0;
  bool stop_algo_now = false;
  bool load_engine = false;
  ConfigParams action_toml;
  bool initcuda = false;
  std::shared_ptr<cyberdog::common::cyberdog_model> fds_;
  bool ReadTomlConfig();
  template<typename _Tp>
  int activation_function_softmax(const _Tp * src, _Tp * dst, int length, int & max_index);
  void Gesture_Action_Rec_Fun(
    const std::shared_ptr<GestureActionSrv::Request> request,
    std::shared_ptr<GestureActionSrv::Response> response);
  void CameraSignalCallback(const CameraMsg::SharedPtr msg);
  void Camera_Operate();
  void Inference_Operate();
  uint8_t ControlCamera(uint8_t _action);
  int doInference();
  int process_history(int max_index);
  int proprecess(Gesture_ori max_index);
  bool LoadEngineIntoCuda();
  bool DestroyCuda();
  bool cudaMemoryPro();
  void WifiSignalCallback(const WifiMsg::SharedPtr msg);
};  // class gesture
}    // namespace interaction
}  // namespace cyberdog
#endif  // CYBERDOG_ACTION__CYBERDOG_HAND_ACTION_HPP_
