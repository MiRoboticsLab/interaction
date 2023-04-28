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
#include <malloc.h>
#include <memory>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_action/cyberdog_hand_action.hpp"


namespace cyberdog
{
namespace interaction
{

gesture::gesture(const std::string & name)
{
  this->node_ptr_ = rclcpp::Node::make_shared(name);
  INFO("Creating cyberdog_action object(node)");
}
gesture::~gesture()
{
  INFO("Destroy [action] object(node) begin");
}

bool gesture::ReadTomlConfig()
{
  INFO("read gesture action toml file");
  toml::value value;
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto local_config_dir = local_share_dir + std::string(
    "/toml_config/interaction/gesture_action.toml");
  if (access(local_config_dir.c_str(), F_OK) != 0) {
    ERROR("%s do not exist!", local_config_dir.c_str());
    ERROR("init failed");
    return false;
  }
  if (!cyberdog::common::CyberdogToml::ParseFile(
      std::string(local_config_dir), value))
  {
    ERROR("fail to read data from toml");
  }
  if (!cyberdog::common::CyberdogToml::Get(value, "queue_size", this->action_toml.queue_size)) {
    ERROR("fail to read key queue_size from toml");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      value, "frames_number",
      this->action_toml.frames_number))
  {
    ERROR("fail to read key frames_number from toml");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      value, "refine_output",
      this->action_toml.refine_output))
  {
    ERROR("fail to read key refine_output from toml");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      value, "softmax_thres",
      this->action_toml.softmax_thres))
  {
    ERROR("fail to read key softmax_thres from toml");
  }

  if (!cyberdog::common::CyberdogToml::Get(
      value, "engine_path",
      this->action_toml.engine_path))
  {
    ERROR("fail to read key engine_path from toml");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      value, "device",
      this->action_toml.device))
  {
    ERROR("fail to read key device from toml");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      value, "fps",
      this->action_toml.fps))
  {
    ERROR("fail to read key fps from toml");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      value, "is_specified",
      this->action_toml.is_specified))
  {
    ERROR("fail to read key is_specified from toml");
  }
  if (!cyberdog::common::CyberdogToml::Get(
      value, "version",
      this->action_toml.version))
  {
    ERROR("fail to read key version from toml");
  }
  return true;
}

bool gesture::cudaMemoryPro()
{
  INFO("cudaMemoryPro begin");
  INFO("context begin");
  this->context = this->engine->createExecutionContext();
  INFO("context over");
  assert(this->context != nullptr);
  INFO("engine_tmp begin");
  const nvinfer1::ICudaEngine & engine_tmp = this->context->getEngine();
  INFO("engine_tmp over");
  // In order to bind the buffers, we need to know the names of the input and output tensors.
  // Note that indices are guaranteed to be less than IEngine::getNbBindings()
  this->inputIndex[0] = engine_tmp.getBindingIndex("i0");
  this->inputIndex[1] = engine_tmp.getBindingIndex("i1");
  this->inputIndex[2] = engine_tmp.getBindingIndex("i2");
  this->inputIndex[3] = engine_tmp.getBindingIndex("i3");
  this->inputIndex[4] = engine_tmp.getBindingIndex("i4");
  this->inputIndex[5] = engine_tmp.getBindingIndex("i5");
  this->inputIndex[6] = engine_tmp.getBindingIndex("i6");
  this->inputIndex[7] = engine_tmp.getBindingIndex("i7");
  this->inputIndex[8] = engine_tmp.getBindingIndex("i8");
  this->inputIndex[9] = engine_tmp.getBindingIndex("i9");
  this->inputIndex[10] = engine_tmp.getBindingIndex("i10");

  this->outputIndex[0] = engine_tmp.getBindingIndex("o0");
  this->outputIndex[1] = engine_tmp.getBindingIndex("o1");
  this->outputIndex[2] = engine_tmp.getBindingIndex("o2");
  this->outputIndex[3] = engine_tmp.getBindingIndex("o3");
  this->outputIndex[4] = engine_tmp.getBindingIndex("o4");
  this->outputIndex[5] = engine_tmp.getBindingIndex("o5");
  this->outputIndex[6] = engine_tmp.getBindingIndex("o6");
  this->outputIndex[7] = engine_tmp.getBindingIndex("o7");
  this->outputIndex[8] = engine_tmp.getBindingIndex("o8");
  this->outputIndex[9] = engine_tmp.getBindingIndex("o9");
  this->outputIndex[10] = engine_tmp.getBindingIndex("o10");
  INFO("cudaMalloc begin");
  CHECK(
    cudaMalloc(
      &this->buffers[this->inputIndex[0]], 3 *
      INPUT_H * INPUT_W * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->inputIndex[1]], 3 * 56 * 56 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->inputIndex[2]], 4 * 28 * 28 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->inputIndex[3]], 4 * 28 * 28 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->inputIndex[4]], 8 * 14 * 14 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->inputIndex[5]], 8 * 14 * 14 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->inputIndex[6]], 8 * 14 * 14 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->inputIndex[7]], 12 * 14 * 14 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->inputIndex[8]], 12 * 14 * 14 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->inputIndex[9]], 20 * 7 * 7 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->inputIndex[10]], 20 * 7 * 7 * sizeof(float)));
  CHECK(
    cudaMalloc(
      &this->buffers[this->outputIndex[0]],
      OUTPUT_SIZE * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->outputIndex[1]], 3 * 56 * 56 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->outputIndex[2]], 4 * 28 * 28 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->outputIndex[3]], 4 * 28 * 28 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->outputIndex[4]], 8 * 14 * 14 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->outputIndex[5]], 8 * 14 * 14 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->outputIndex[6]], 8 * 14 * 14 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->outputIndex[7]], 12 * 14 * 14 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->outputIndex[8]], 12 * 14 * 14 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->outputIndex[9]], 20 * 7 * 7 * sizeof(float)));
  CHECK(cudaMalloc(&this->buffers[this->outputIndex[10]], 20 * 7 * 7 * sizeof(float)));
  INFO("cudaMalloc over");
  INFO("cudaStreamCreate begin");
  CHECK(cudaStreamCreate(&this->stream));
  INFO("cudaStreamCreate over");
  float inputdata_1[3 * 56 * 56] = {0.0};
  float inputdata_2[4 * 28 * 28] = {0.0};
  float inputdata_3[4 * 28 * 28] = {0.0};
  float inputdata_4[8 * 14 * 14] = {0.0};
  float inputdata_5[8 * 14 * 14] = {0.0};
  float inputdata_6[8 * 14 * 14] = {0.0};
  float inputdata_7[12 * 14 * 14] = {0.0};
  float inputdata_8[12 * 14 * 14] = {0.0};
  float inputdata_9[20 * 7 * 7] = {0.0};
  float inputdata_10[20 * 7 * 7] = {0.0};
  INFO("cudaMemcpyAsync begin");
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[1]], inputdata_1,
      3 * 56 * 56 * sizeof(float), cudaMemcpyHostToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[2]], inputdata_2,
      4 * 28 * 28 * sizeof(float), cudaMemcpyHostToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[3]], inputdata_3,
      4 * 28 * 28 * sizeof(float), cudaMemcpyHostToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[4]], inputdata_4,
      8 * 14 * 14 * sizeof(float), cudaMemcpyHostToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[5]], inputdata_5,
      8 * 14 * 14 * sizeof(float), cudaMemcpyHostToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[6]], inputdata_6,
      8 * 14 * 14 * sizeof(float), cudaMemcpyHostToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[7]], inputdata_7,
      12 * 14 * 14 * sizeof(float), cudaMemcpyHostToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[8]], inputdata_8,
      12 * 14 * 14 * sizeof(float), cudaMemcpyHostToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[9]], inputdata_9,
      20 * 7 * 7 * sizeof(float), cudaMemcpyHostToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[10]], inputdata_10,
      20 * 7 * 7 * sizeof(float), cudaMemcpyHostToDevice, this->stream));
  INFO("cudaMemoryPro over");
}
bool gesture::LoadEngineIntoCuda()
{
  INFO("LoadEngineIntoCuda begin");
  // model file exist or not
  if (access(this->action_toml.engine_path.c_str(), F_OK) != 0) {
    INFO("model file %s do not exist!", this->action_toml.engine_path.c_str());
    return false;
  }
  CHECK(cudaSetDevice(this->action_toml.device));
  char * trtModelStream{nullptr};
  size_t size{0};
  std::ifstream file(this->action_toml.engine_path.c_str(), std::ios::binary);
  if (file.good()) {
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();
  }
  this->runtime = nvinfer1::createInferRuntime(gLogger);
  assert(runtime != nullptr);
  this->engine = runtime->deserializeCudaEngine(trtModelStream, size, nullptr);
  assert(engine != nullptr);
  INFO("LoadEngineIntoCuda over");
  delete[] trtModelStream;
  return true;
}

bool gesture::DestroyCuda()
{
  INFO("DestroyCuda,begin");
  cudaStreamDestroy(this->stream);
  this->context->destroy();
  this->context = nullptr;
  this->engine->destroy();
  this->engine = nullptr;
  this->runtime->destroy();
  this->runtime = nullptr;
  for (int i = 0; i < 22; i++) {
    CHECK(cudaFree(this->buffers[i]));
  }
  cudaDeviceReset();
  INFO("DestroyCuda,over");
  malloc_trim(0);
  INFO("Malloc trim complated. ");
}

bool gesture::Init()
{
  INFO("action node init");
  // read config params
  bool toml_exist = ReadTomlConfig();
  if (toml_exist == false) {
    INFO("read toml file failed,exit init");
    return toml_exist;
  } else {
    INFO("read toml file successfully");
  }
  this->gesture_action_service_ = this->node_ptr_->create_service<GestureActionSrv>(
    "gesture_action_control",
    std::bind(
      &gesture::Gesture_Action_Rec_Fun, this, std::placeholders::_1,
      std::placeholders::_2));
  // camera init
  this->camera_client_ = this->node_ptr_->create_client<CameraSrv>("camera_service");
  this->camera_sub_ = this->node_ptr_->create_subscription<CameraMsg>(
    "image", rclcpp::SystemDefaultsQoS(),
    std::bind(&gesture::CameraSignalCallback, this, std::placeholders::_1));
  this->gesture_action_pub_ = this->node_ptr_->create_publisher<GestureActionMsg>(
    "gesture_action_msg", rclcpp::SystemDefaultsQoS());
  this->audio_pub_ = this->node_ptr_->create_publisher<AudioMsg>(
    "speech_play",
    10);
  this->audio_msg_ptr_ = std::make_shared<AudioMsg>();
  this->audio_msg_ptr_->module_name = "action_gesture";
  // model download
  std::string username = "gesture_action";
  this->fds_ = std::make_shared<cyberdog::common::cyberdog_model>(
    username, this->action_toml.is_specified, this->action_toml.version);
  auto callback_group_ = this->node_ptr_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  this->connector_sub_ = this->node_ptr_->create_subscription<protocol::msg::ConnectorStatus>(
    "connector_state", rclcpp::SystemDefaultsQoS(),
    std::bind(&gesture::WifiSignalCallback, this, std::placeholders::_1), sub_options);

  this->camera_thread_ = std::make_shared<std::thread>(&gesture::Camera_Operate, this);
  this->inference_thread_ = std::make_shared<std::thread>(&gesture::Inference_Operate, this);

  return true;
}


void gesture::WifiSignalCallback(const WifiMsg::SharedPtr msg)
{
  if (msg->is_internet && this->connector_sub_ != nullptr) {
    INFO("[WifiSignalCallback] internet is ok!!!");
    this->fds_->SetTimeout(600);
    int32_t code = this->fds_->UpdateModels();
    if (code == 0) {
      INFO("download and update gesture_action model from Fds successfully");
    } else {
      INFO("download and update gesture_action model from Fds failed or timeout");
    }
    this->connector_sub_ = nullptr;
  }
}


void gesture::Gesture_Action_Rec_Fun(
  const std::shared_ptr<GestureActionSrv::Request> request,
  std::shared_ptr<GestureActionSrv::Response> response)
{
  INFO("new gesture action request come");
  this->ai_camera.command = request->command;
  this->ai_camera.timeout = request->timeout;
  if (this->camera_is_open == false) {
    if (this->ai_camera.command ==
      GestureActionSrv::Request::STOP_ALGO)
    {
      INFO("algo already closed");
    } else {
      INFO("new open algo request come in ");
      this->begin_time = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      this->algo_duration = request->timeout;
      this->stop_time = this->begin_time + this->algo_duration;
      INFO("ori algo duration %d seconds", this->algo_duration);
      std::unique_lock<std::mutex> run_camera_lock(this->ai_camera.mtx);
      this->ai_camera.thread_work = true;
      this->ai_camera.cond.notify_all();
    }
  } else {
    if (this->ai_camera.command ==
      GestureActionSrv::Request::STOP_ALGO)
    {
      INFO("new close algo request come in ,close algo now");
      // 立刻关闭算法
      this->stop_algo_now = true;
    } else {
      INFO("other open algo request come in");
      int now_time = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      if (this->stop_time - now_time > request->timeout) {
        INFO("no need to change stop_time ");
      } else {
        this->stop_time = now_time + request->timeout;
        INFO("change stop_time ");
      }
    }
  }
  response->code = GestureActionSrv::Response::RESULT_SUCCESS;
}


void gesture::Camera_Operate()
{
  INFO("Camera Operate thread begin");
  while (rclcpp::ok) {
    std::unique_lock<std::mutex> run_camera_lock(this->ai_camera.mtx);
    this->ai_camera.cond.wait(
      run_camera_lock, [&] {
        return this->ai_camera.thread_work;
      });
    INFO("new camera control cmd come");
    INFO("open ai camera");
    this->audio_msg_ptr_->play_id = AudioMsg::PID_MODEL_LOADING_START;
    this->audio_pub_->publish(*(this->audio_msg_ptr_));
    // 添加模型替换判断
    if (this->fds_->Load_Model_Check()) {
      this->fds_->Post_Process();
      INFO("replace and remove temp model prepare load new model");
    } else {
      INFO("load model without replace new model");
    }
    bool load_result = LoadEngineIntoCuda();
    if (!load_result) {
      this->audio_msg_ptr_->play_id = AudioMsg::PID_MODEL_VERSION_OLD;
      this->audio_pub_->publish(*(this->audio_msg_ptr_));
      this->ai_camera.thread_work = false;
      continue;
    }
    bool memery_result = cudaMemoryPro();
    uint8_t result_open = ControlCamera(CameraSrv::Request::START_IMAGE_PUBLISH);
    if (result_open == 0) {
      this->camera_is_open = true;
      this->load_engine = true;
      INFO("open ai camera successfully");
      this->audio_msg_ptr_->play_id = AudioMsg::PID_MODEL_LOADING_COMPLETE;
      this->audio_pub_->publish(*(this->audio_msg_ptr_));
    } else {
      INFO("open ai camera failed");
    }
    this->ai_camera.thread_work = false;
    INFO("wait time out or user shut down camera");
    int now_time;
    while (rclcpp::ok) {
      // INFO("wait for timeout close ai camera");
      now_time = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      if (now_time > this->stop_time ||
        this->stop_algo_now)
      {
        if (this->stop_algo_now == false) {
          INFO("algo timeout,close ai camera");
          INFO("all algo operate duration is %d", this->stop_time - this->begin_time);
        } else {
          INFO("all algo operate duration is %d", this->stop_time - this->begin_time);
          INFO("new algo request ask to close ai camera now");
        }
        uint8_t result_close = ControlCamera(CameraSrv::Request::STOP_IMAGE_PUBLISH);
        if (result_close == 0) {
          INFO("close ai camera successfully");
          this->camera_is_open = false;
          this->stop_algo_now = false;
          this->algo_duration = 0;
          this->begin_time = 0;
        } else {
          INFO("close ai camera failed");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        bool destroy_result = DestroyCuda();
        if (destroy_result == true) {
          this->load_engine = false;
        }
        break;
      }
    }
  }
}

void gesture::CameraSignalCallback(const CameraMsg::SharedPtr msg)
{
  if (this->load_engine == false) {
    return;
  }
  std::unique_lock<std::mutex> inference_lock(this->img_inference.mtx);
  this->img_inference.thread_work = true;
  // cvbridge
  this->cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  this->cv_ptr->image.copyTo(this->img);
  // debug show image
  // cv::imshow("vision", this->img);
  // cv::waitKey(10);
  this->img_inference.cond.notify_one();
}

void gesture::Inference_Operate()
{
  INFO("Inference thread begin");
  while (rclcpp::ok) {
    std::unique_lock<std::mutex> inference_lock(this->img_inference.mtx);
    this->img_inference.cond.wait(
      inference_lock, [&] {
        return this->img_inference.thread_work;
      });
    // inference
    auto start = std::chrono::system_clock::now();
    int max_index = doInference();
    int new_index = proprecess(Gesture_ori(max_index));

    if (this->history.size() < this->action_toml.queue_size) {
      this->history.push_back(new_index);
    } else {
      int history_index = process_history(new_index);
      if (history_index != 0) {
        INFO("history index is %d", history_index);
        INFO("max_probability is %f", this->max_probability);
      }
      std::shared_ptr<GestureActionMsg> gesture_action_msg_ptr_ =
        std::make_shared<GestureActionMsg>();
      gesture_action_msg_ptr_->id = history_index;
      this->gesture_action_pub_->publish(*gesture_action_msg_ptr_);
    }
    auto end = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    this->img_inference.thread_work = false;
  }
}


int gesture::proprecess(Gesture_ori max_index)
{
  int new_index;
  switch (max_index) {
    case Gesture_ori::Doing_other_things:
      new_index = static_cast<int>(Gesture_cut::No_gesture);
      break;
    case Gesture_ori::Drumming_Fingers:
      new_index = static_cast<int>(Gesture_cut::No_gesture);
      break;
    case Gesture_ori::No_gesture:
      new_index = static_cast<int>(Gesture_cut::No_gesture);
      break;
    case Gesture_ori::Pulling_Hand_In:
      new_index = static_cast<int>(Gesture_cut::Pulling_Hand_Or_Two_Fingers_In);
      break;
    case Gesture_ori::Pulling_Two_Fingers_In:
      new_index = static_cast<int>(Gesture_cut::Pulling_Hand_Or_Two_Fingers_In);
      break;
    case Gesture_ori::Pushing_Hand_Away:
      new_index = static_cast<int>(Gesture_cut::Pushing_Hand_Or_Two_Fingers_Away);
      break;
    case Gesture_ori::Pushing_Two_Fingers_Away:
      new_index = static_cast<int>(Gesture_cut::Pushing_Hand_Or_Two_Fingers_Away);
      break;
    case Gesture_ori::Rolling_Hand_Backward:
      new_index = static_cast<int>(Gesture_cut::No_gesture);
      break;
    case Gesture_ori::Rolling_Hand_Forward:
      new_index = static_cast<int>(Gesture_cut::No_gesture);
      break;
    case Gesture_ori::Shaking_Hand:
      new_index = static_cast<int>(Gesture_cut::No_gesture);
      break;
    case Gesture_ori::Sliding_Two_Fingers_Down:
      new_index = static_cast<int>(Gesture_cut::Sliding_Hand_Two_Fingers_Down);
      break;
    case Gesture_ori::Sliding_Two_Fingers_Left:
      new_index = static_cast<int>(Gesture_cut::Sliding_Hand_Two_Fingers_Left);
      break;
    case Gesture_ori::Sliding_Two_Fingers_Right:
      new_index = static_cast<int>(Gesture_cut::Sliding_Hand_Two_Fingers_Right);
      break;
    case Gesture_ori::Sliding_Two_Fingers_Up:
      new_index = static_cast<int>(Gesture_cut::Sliding_Hand_Two_Fingers_Up);
      break;
    case Gesture_ori::Stop_Sign:
      new_index = static_cast<int>(Gesture_cut::Stop_Sign);
      break;
    case Gesture_ori::Swiping_Down:
      new_index = static_cast<int>(Gesture_cut::Sliding_Hand_Two_Fingers_Down);
      break;
    case Gesture_ori::Swiping_Left:
      new_index = static_cast<int>(Gesture_cut::Sliding_Hand_Two_Fingers_Left);
      break;
    case Gesture_ori::Swiping_Right:
      new_index = static_cast<int>(Gesture_cut::Sliding_Hand_Two_Fingers_Right);
      break;
    case Gesture_ori::Swiping_Up:
      new_index = static_cast<int>(Gesture_cut::Sliding_Hand_Two_Fingers_Up);
      break;
    case Gesture_ori::Thumb_Down:
      new_index = static_cast<int>(Gesture_cut::No_gesture);
      break;
    case Gesture_ori::Thumb_Up:
      new_index = static_cast<int>(Gesture_cut::Thumb_Up);
      break;
    case Gesture_ori::Turning_Hand_Clockwise:
      new_index = static_cast<int>(Gesture_cut::No_gesture);
      break;
    case Gesture_ori::Turning_Hand_Counterclockwise:
      new_index = static_cast<int>(Gesture_cut::No_gesture);
      break;
    case Gesture_ori::Zooming_In_With_Full_Hand:
      new_index = static_cast<int>(Gesture_cut::Zooming_In_With_Hand_Or_Two_Fingers);
      break;
    case Gesture_ori::Zooming_In_With_Two_Fingers:
      new_index = static_cast<int>(Gesture_cut::Zooming_In_With_Hand_Or_Two_Fingers);
      break;
    case Gesture_ori::Zooming_Out_With_Full_Hand:
      new_index = static_cast<int>(Gesture_cut::Zooming_Out_With_Hand_Or_Two_Fingers);
      break;
    case Gesture_ori::Zooming_Out_With_Two_Fingers:
      new_index = static_cast<int>(Gesture_cut::Zooming_Out_With_Hand_Or_Two_Fingers);
      break;
    default:
      break;
  }
  return new_index;
}

int gesture::process_history(int index)
{
  int history_filter_result;
  if (this->action_toml.refine_output == false) {
    history_filter_result = index;
    return history_filter_result;
    INFO("refine model ");
  }
  int same_nums = 0;
  for (auto frame : this->history) {
    if (index == frame) {
      same_nums++;
    }
  }
  // INFO("the same index is same_nums %d",same_nums);
  if (same_nums >= this->action_toml.frames_number) {
    history_filter_result = index;
  } else {
    history_filter_result = 0;
  }
  // INFO("the history filter result index %d",history_filter_result);
  this->history.push_back(index);
  this->history.pop_front();
  return history_filter_result;
}

int gesture::doInference()
{
  float inputdata_0[3 * INPUT_H * INPUT_W];
  cv::cvtColor(this->img, this->img, cv::COLOR_BGR2RGB);
  cv::Mat re(INPUT_H, INPUT_W, CV_8UC3);
  cv::resize(this->img, re, re.size(), 0, 0, cv::INTER_LINEAR);
  // tansform 0~1 ,normalize
  for (int i = 0; i < INPUT_H * INPUT_W; i++) {
    inputdata_0[i] = (re.at<cv::Vec3b>(i)[0] / 255.0 - 0.485) / 0.229;
    inputdata_0[i + INPUT_H *
      INPUT_W] = (re.at<cv::Vec3b>(i)[1] / 255.0 - 0.456) / 0.224;
    inputdata_0[i + 2 * INPUT_H * INPUT_W] = (re.at<cv::Vec3b>(
        i)[2] / 255.0 - 0.406) / 0.225;
  }
  float prob[OUTPUT_SIZE];
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[0]], inputdata_0,
      3 * INPUT_H * INPUT_W * sizeof(float),
      cudaMemcpyHostToDevice, this->stream));
  this->context->enqueue(1, this->buffers, this->stream, nullptr);
  CHECK(
    cudaMemcpyAsync(
      prob, this->buffers[this->outputIndex[0]],
      OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, this->stream));
  cudaStreamSynchronize(this->stream);
  float_t softmax[OUTPUT_SIZE] = {0};
  int max_index = 0;
  activation_function_softmax(prob, softmax, OUTPUT_SIZE, max_index);
  this->max_probability = softmax[max_index];
  if (softmax[max_index] < this->action_toml.softmax_thres) {
    max_index = 0;
  }
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[1]], this->buffers[this->outputIndex[1]],
      3 * 56 * 56 * sizeof(float), cudaMemcpyDeviceToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[2]], this->buffers[this->outputIndex[2]],
      4 * 28 * 28 * sizeof(float), cudaMemcpyDeviceToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[3]], this->buffers[this->outputIndex[3]],
      4 * 28 * 28 * sizeof(float), cudaMemcpyDeviceToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[4]], this->buffers[this->outputIndex[4]],
      8 * 14 * 14 * sizeof(float), cudaMemcpyDeviceToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[5]], this->buffers[this->outputIndex[5]],
      8 * 14 * 14 * sizeof(float), cudaMemcpyDeviceToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[6]], this->buffers[this->outputIndex[6]],
      8 * 14 * 14 * sizeof(float), cudaMemcpyDeviceToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[7]], this->buffers[this->outputIndex[7]],
      12 * 14 * 14 * sizeof(float), cudaMemcpyDeviceToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[8]], this->buffers[this->outputIndex[8]],
      12 * 14 * 14 * sizeof(float), cudaMemcpyDeviceToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[9]], this->buffers[this->outputIndex[9]],
      20 * 7 * 7 * sizeof(float), cudaMemcpyDeviceToDevice, this->stream));
  CHECK(
    cudaMemcpyAsync(
      this->buffers[this->inputIndex[10]], this->buffers[this->outputIndex[10]],
      20 * 7 * 7 * sizeof(float), cudaMemcpyDeviceToDevice, this->stream));
  return max_index;
}


uint8_t gesture::ControlCamera(uint8_t _action)
{
  /*
   * return:
   * 0: 控制相机成功
   * 1: 客户端在请求服务出现时被打断
   * 2: 等待服务出现（启动）超时
   * 3: 请求相机模块超时或延迟
   * 4: 控制相机失败
   */
  auto single_control = [&](uint8_t _command) -> uint {
      auto request = std::make_shared<CameraSrv::Request>();
      std::string operate;
      switch (_command) {
        case CameraSrv::Request::SET_PARAMETERS:
          operate = "set parameters";
          break;
        case CameraSrv::Request::TAKE_PICTURE:
          operate = "task picture";
          break;
        case CameraSrv::Request::START_RECORDING:
          operate = "start recoreing";
          break;
        case CameraSrv::Request::STOP_RECORDING:
          operate = "stop recoreing";
          break;
        case CameraSrv::Request::GET_STATE:
          operate = "get state";
          break;
        case CameraSrv::Request::DELETE_FILE:
          operate = "delete file";
          break;
        case CameraSrv::Request::GET_ALL_FILES:
          operate = "get all files";
          break;
        case CameraSrv::Request::START_LIVE_STREAM:
          operate = "start live stream";
          break;
        case CameraSrv::Request::STOP_LIVE_STREAM:
          operate = "stop live stream";
          break;
        case CameraSrv::Request::START_IMAGE_PUBLISH:
          operate = "start publist image";
          request->width = INPUT_H;
          request->height = INPUT_W;
          request->fps = this->action_toml.fps;
          break;
        case CameraSrv::Request::STOP_IMAGE_PUBLISH:
          operate = "stop publist image";
          break;
        default:
          operate = "undefined";
          break;
      }
      INFO("Control camera <%s>", operate.c_str());
      request->command = _command;
      request->args = "";
      if (!rclcpp::ok()) {
        WARN("Client interrupted while requesting for service to appear.");
        return 1;
      }
      if (!this->camera_client_->wait_for_service(std::chrono::seconds(3))) {
        WARN("Waiting for service to appear(start) timeout.");
        return 2;
      }
      auto result = this->camera_client_->async_send_request(request);
      std::future_status status =
        result.wait_for(std::chrono::seconds(5));
      if (status != std::future_status::ready) {
        WARN("Request camera module timedout or deferred.");
        return 3;
      }
      auto response_ptr = result.get();
      if (response_ptr->result == CameraSrv::Response::RESULT_SUCCESS) {
        INFO(
          "Control camera module succeeded, %d, <%s>", static_cast<int>(response_ptr->result),
          response_ptr->msg.c_str());
        return 0;
      } else {
        WARN(
          "Control camera module failed, %d, <%s>", static_cast<int>(response_ptr->result),
          response_ptr->msg.c_str());
        return 4;
      }
    };
  uint ret = 0;
  if (_action == CameraSrv::Request::START_IMAGE_PUBLISH) {
    ret = single_control(CameraSrv::Request::STOP_IMAGE_PUBLISH);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return (ret == 0) ? single_control(CameraSrv::Request::START_IMAGE_PUBLISH) : ret;
  } else {
    return single_control(_action);
  }
}


template<typename _Tp>
int gesture::activation_function_softmax(
  const _Tp * src, _Tp * dst, int length, int &
  max_index)
{
  const _Tp alpha = *std::max_element(src, src + length);
  _Tp denominator{0};

  for (int i = 0; i < length; ++i) {
    if (src[i] == alpha) {
      max_index = i;
    }
    dst[i] = std::exp(src[i] - alpha);
    denominator += dst[i];
  }

  for (int i = 0; i < length; ++i) {
    dst[i] /= denominator;
  }

  return 0;
}


void gesture::Run()
{
  INFO("cyberdog_action node spin,wait for request");
  this->executor_.add_node(node_ptr_);
  this->executor_.spin();
  INFO("cyberdog_action node rclcpp close");
  rclcpp::shutdown();
}
}  // namespace interaction
}  // namespace cyberdog
