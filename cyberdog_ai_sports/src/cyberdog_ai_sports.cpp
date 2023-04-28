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
#include <string>
#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "cyberdog_ai_sports/cyberdog_ai_sports.hpp"

namespace cyberdog
{
namespace interaction
{

sport::sport(const std::string & name)
: Node(name),
  name_(name),
  node_config_dir_("/toml_config/interaction/sports.toml")
{
  INFO("Creating [sport] object(node)");
}

sport::~sport()
{
  INFO("Destroy [sport] object(node)");
}

bool sport::Init()
{
  // sport counts init
  this->ReadTomlConfig();
  // 创建回调函数组
  this->ctl_life_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  this->ai_sport_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  this->sport_counts_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  this->time_cb_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  this->ai_change_state_ = this->create_client<CStateSrv>(
    "vision_manager/change_state", rmw_qos_profile_services_default,
    ctl_life_cb_group_);
  this->ai_get_state_ = this->create_client<GStateSrv>(
    "vision_manager/get_state", rmw_qos_profile_services_default,
    ctl_life_cb_group_);
  // configure vision_manager
  Is_Configure();
  INFO("sport node init");
  rclcpp::PublisherOptions pub_options;
  pub_options.callback_group = sport_counts_cb_group_;
  this->sport_counts_result_pub_ = this->create_publisher<SportMsg>(
    "sport_counts_msg", rclcpp::SystemDefaultsQoS(), pub_options);
  this->ai_sports_client_ = this->create_client<VisionSrv>(
    "algo_manager", rmw_qos_profile_services_default, ai_sport_cb_group_);
  this->sport_counts_service_ = this->create_service<SportSrv>(
    "sport_manager", std::bind(
      &sport::Ai_Sport_Process_Fun, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, sport_counts_cb_group_);
  this->ai_sports_request_ = std::make_shared<VisionSrv::Request>();
  // audio
  this->audio_pub_ = this->create_publisher<AudioMsg>("speech_play", 10, pub_options);
  this->sport_audiomsg_ptr_ = std::make_shared<AudioMsg>();
  this->sport_audiomsg_ptr_->module_name = "bone";
  // create timer
  this->update_timer_ = this->create_wall_timer(
    std::chrono::seconds(static_cast<int64_t>(1)),
    std::bind(&sport::UpdateStatus, this), time_cb_group_);

  return true;
}

bool sport::ReadTomlConfig()
{
  INFO("Load configuration file.");
  this->params_pkg_dir_ = ament_index_cpp::get_package_share_directory("params");
  this->node_config_dir_ = this->params_pkg_dir_ + this->node_config_dir_;
  INFO("Params config file dir:<%s>", this->node_config_dir_.c_str());

  if (access(this->node_config_dir_.c_str(), F_OK)) {
    ERROR("Params config file does not exist");
    return false;
  }

  if (access(this->node_config_dir_.c_str(), R_OK)) {
    ERROR("Params config file does not have read permissions");
    return false;
  }

  if (access(this->node_config_dir_.c_str(), W_OK)) {
    ERROR("Params config file does not have write permissions");
    return false;
  }

  if (!cyberdog::common::CyberdogToml::ParseFile(
      this->node_config_dir_.c_str(), this->params_toml_))
  {
    ERROR("Params config file is not in toml format");
    return false;
  }
  // 从toml中读取参数
  this->squat_dataFusion_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Squat", "DataFusionNumber"));
  this->squat_dataFiler_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Squat", "DataFilterNumber"));
  this->squat_threMinAngle_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Squat", "ThreMinAngle"));
  this->squat_threMaxAngle_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Squat", "ThreMaxAngle"));


  this->highKnee_dataFusion_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "HighKnees", "DataFusionNumber"));
  this->highKnee_dataFiler_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "HighKnees", "DataFilterNumber"));
  this->highKnee_threMinAngle_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "HighKnees", "ThreMinAngle"));
  this->highKnee_threMaxAngle_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "HighKnees", "ThreMaxAngle"));

  this->sitUp_dataFusion_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Situp", "DataFusionNumber"));
  this->sitUp_dataFiler_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Situp", "DataFilterNumber"));
  this->sitUp_threMinAngle_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Situp", "ThreMinAngle"));
  this->sitUp_threMidAngle1_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Situp", "ThreMidAngle1"));
  this->sitUp_threMidAngle2_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Situp", "ThreMidAngle2"));
  this->sitUp_threMaxAngle_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Situp", "ThreMaxAngle"));

  this->pressUp_dataFusion_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "PressUp", "DataFusionNumber"));
  this->pressUp_dataFiler_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "PressUp", "DataFilterNumber"));
  this->pressUp_threMaxAngle_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "PressUp", "ThreMaxAngle"));
  this->pressUp_difMinAngle_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "PressUp", "DifMaxAngle"));
  this->pressUp_difMaxAngle_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "PressUp", "DifminAngle"));

  this->plank_dataFusion_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Plank", "DataFusionNumber"));
  this->plank_dataFiler_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Plank", "DataFilterNumber"));
  this->plank_threMinAngle_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Plank", "ThreMinAngle"));
  this->plank_threMaxAngle_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "Plank", "ThreMaxAngle"));

  this->jump_dataFusion_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "JumpJack", "DataFusionNumber"));
  this->jump_dataFiler_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "JumpJack", "DataFilterNumber"));
  this->jump_threMidAngle1_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "JumpJack", "ThreMidAngle1"));
  this->jump_threMidAngle2_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "JumpJack", "ThreMidAngle2"));
  this->jump_threMidAngle3_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "JumpJack", "ThreMidAngle3"));
  this->jump_threMidAngle4_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "JumpJack", "ThreMidAngle4"));
  this->jump_threMidAngle5_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "JumpJack", "ThreMidAngle5"));
  this->jump_threMidAngle6_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "JumpJack", "ThreMidAngle6"));
  this->jump_threMidAngle7_ = static_cast<int>(toml::find<int>(
      this->params_toml_, "Sport", "JumpJack", "ThreMidAngle7"));
}
void sport::UpdateStatus()
{
  if (this->wake_sport_counts_call_ == false) {
    return;
  }
  this->end_time_ = std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  INFO("end_time: %d.", static_cast<int>(this->end_time_));
  if (this->end_time_ - this->start_time_ >= this->sport_counts_inter_time_) {
    INFO("end_time: %d.", static_cast<int>(this->end_time_));
    bool status = this->Is_Deactive();
    INFO("Timeout closing algorithm.");
    if (status) {
      std::unique_lock<std::mutex> lock(sport_counts_mtx_);
      std::shared_ptr<SportMsg> sport_counts_result_msg_ =
        std::make_shared<SportMsg>();
      sport_counts_result_msg_->algo_switch = SportMsg::TIMEOUT_CLOSE;
      sport_counts_result_msg_->sport_type = static_cast<int32_t>(this->sport_type_);
      this->sport_counts_result_pub_->publish(*sport_counts_result_msg_);
      this->ResetAlgo();
    }
  }
}

void sport::Ai_Sport_Request_Load(AlgoManagerRequest sport_counts_request)
{
  AlgoMsg algo_enable_list;
  for (auto algo_enable_single : sport_counts_request.algo_enable_) {
    algo_enable_list.algo_module = algo_enable_single;
    this->ai_sports_request_->algo_enable.push_back(algo_enable_list);
    INFO(
      "need enable algo %d",
      static_cast<int>(algo_enable_list.algo_module));
  }
  AlgoMsg algo_disable_list;
  for (auto algo_disable_single : sport_counts_request.algo_disable_) {
    algo_disable_list.algo_module = algo_disable_single;
    INFO(
      "need disable algo %d",
      static_cast<int>(algo_enable_list.algo_module));
    this->ai_sports_request_->algo_disable.push_back(algo_disable_list);
  }
}

void sport::Ai_Sport_Process_Fun(
  const std::shared_ptr<SportSrv::Request> request,
  std::shared_ptr<SportSrv::Response> response)
{
  // stop algorithm
  if (request->command == false) {
    if (Is_Deactive()) {
      INFO(
        "Turn off sport counts algorithm, %d.",
        static_cast<int>(request->command));
      std::shared_ptr<SportMsg> sport_counts_result_msg_ =
        std::make_shared<SportMsg>();
      sport_counts_result_msg_->algo_switch = SportMsg::REQUEST_CLOSE;
      sport_counts_result_msg_->sport_type = static_cast<int32_t>(this->sport_type_);
      this->sport_counts_result_pub_->publish(*sport_counts_result_msg_);
      this->ResetAlgo();
    }
    response->result = SportSrv::Response::ENABLE_SUCCESS;
    response->code = SportSrv::Response::ENABLE_SUCCESS;
    return;
  }
  if (wake_sport_counts_call_ == true) {
    INFO("Algorithm is processing.");
    return;
  }

  INFO(
    "Sport_counts service already started,current sport is: %d.",
    static_cast<int>(request->sport_type));
  this->sport_type_ = static_cast<int>(request->sport_type);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = this->ai_sport_cb_group_;
  rclcpp::SensorDataQoS pub_qos;
  pub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  this->ai_sports_sub_ = this->create_subscription<VisionMsg>(
    "person", pub_qos,
    std::bind(
      &sport::Ai_Sport_Result_Callback,
      this, std::placeholders::_1), sub_options);
  if (request->timeout > 5 || request->timeout < 300) {
    if (request->sport_type == SportSrv::Request::SPORT_PLANK) {
      this->sport_counts_inter_time_ = request->timeout + 60;
      this->sport_counts_ = request->timeout;
      INFO("timeout: %d.", this->sport_counts_inter_time_);

    } else {
      this->sport_counts_inter_time_ = request->timeout;
      INFO("timeout: %d.", this->sport_counts_inter_time_);
      this->sport_counts_ = request->counts;
      INFO("request counts: %d.", this->sport_counts_);
    }
  }
  AlgoManagerRequest sport_counts_request;
  sport_counts_request.algo_enable_.push_back(AlgoMsg::ALGO_BODY);
  sport_counts_request.algo_enable_.push_back(AlgoMsg::ALGO_KEYPOINTS);
  this->Ai_Sport_Request_Load(sport_counts_request);
  auto ai_sport_response_ = this->ai_sports_client_->async_send_request(
    this->ai_sports_request_);
  auto status = ai_sport_response_.wait_for(std::chrono::seconds(2));
  if (status == std::future_status::ready) {
    // start process algorithm
    this->Is_Active();
    INFO(
      "Receive the return response of the request to sports count %d.",
      static_cast<int>(ai_sport_response_.get()->result_enable));
    response->result = static_cast<int>(ai_sport_response_.get()->result_enable);
    response->code = static_cast<int>(ai_sport_response_.get()->result_enable);
    if (!static_cast<int>(ai_sport_response_.get()->result_enable)) {
      this->wake_sport_counts_call_ = true;
    }
    this->ai_sports_request_->algo_enable.clear();
    this->sport_audiomsg_ptr_->play_id = AudioMsg::PID_BONE_POINT_START;
    this->audio_pub_->publish(*(this->sport_audiomsg_ptr_));
    this->start_time_ = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    INFO("start_time_: %d.", static_cast<int>(this->start_time_));
  } else {
    ERROR("Failed to call service /algo_manager");
  }
}

void sport::Ai_Sport_Result_Callback(VisionMsg msg)
{
  std::unique_lock<std::mutex> lock(sport_counts_mtx_);
  if (this->wake_sport_counts_call_ == false) {
    INFO("sport counts algorithm failed.");
    return;
  }
  INFO("The topic callback of sport_counts already started.");
  if (msg.body_info.count > 0) {
    INFO(
      "Number of human bodies recognized, %d.",
      static_cast<int>(msg.body_info.count));
    if (msg.body_info.count != 1) {
      INFO("Recognition to multiple people.");
      this->sport_audiomsg_ptr_->play_id = AudioMsg::PID_BONE_POINT_DETECTING;
      this->audio_pub_->publish(*(this->sport_audiomsg_ptr_));
      return;
    }
    std::vector<std::vector<cyberdog::interaction::XMPoint>> mult_human;
    uint32_t height;
    uint32_t width;
    for (auto num = 0; num < msg.body_info.count; num++) {
      std::vector<cyberdog::interaction::XMPoint> single_human;
      auto body = msg.body_info.infos[num];
      XMPoint point;
      point.x = body.roi.x_offset;
      point.y = body.roi.y_offset;
      single_human.push_back(point);
      // height and width of boundingbox
      height = body.roi.height;
      width = body.roi.width;
      INFO("Height and width of boundingbox: (%d, %d).", height, width);
      point.x = body.roi.x_offset + width;
      point.y = body.roi.y_offset + height;
      single_human.push_back(point);
      for (auto i : msg.body_info.infos[num].keypoints) {
        XMPoint point;
        point.x = i.x;
        point.y = i.y;
        single_human.push_back(point);
      }
      INFO("Keypoint size: %d.", single_human.size());
      // audio prompt
      this->flag_++;
      if (flag_ % 20 == 0 && ((single_human[2].x == 0 && single_human[2].y == 0) &&
        (single_human[3].x == 0 && single_human[3].y == 0) &&
        (single_human[4].x == 0 && single_human[4].y == 0)) ||
        ((single_human[17].x == 0 && single_human[17].y == 0) &&
        (single_human[18].x == 0 && single_human[18].y == 0)))
      {
        this->sport_audiomsg_ptr_->play_id = AudioMsg::PID_BONE_POINT_BACK;
        this->audio_pub_->publish(*(this->sport_audiomsg_ptr_));
      }
      mult_human.push_back(single_human);
    }
    // keypoints processing
    int32_t counts = this->ProcessKeypoint(mult_human, width, height);
    if (this->sport_type_ != 5 && update_counts_ < counts) {
      std::shared_ptr<SportMsg> sport_counts_result_msg_ =
        std::make_shared<SportMsg>();
      sport_counts_result_msg_->sport_type = this->sport_type_;
      sport_counts_result_msg_->counts = counts;
      this->sport_counts_result_pub_->publish(*sport_counts_result_msg_);
      update_counts_ = counts;
    }
    if (this->sport_type_ == 5 && update_counts_ < counts) {
      std::shared_ptr<SportMsg> sport_counts_result_msg_ =
        std::make_shared<SportMsg>();
      counts = counts / 10;
      sport_counts_result_msg_->duration = counts;
      sport_counts_result_msg_->sport_type = this->sport_type_;
      this->sport_counts_result_pub_->publish(*sport_counts_result_msg_);
      update_counts_ = counts;
    }
    INFO("Current number of sport/time: %d", counts);
    INFO("sport_counts_: %d", sport_counts_);
    if (this->sport_counts_ == counts) {
      if (Is_Deactive()) {
        INFO(
          "Turn off sport counts algorithm, %d.",
          static_cast<int>(this->sport_counts_));
        std::shared_ptr<SportMsg> sport_counts_result_msg_ =
          std::make_shared<SportMsg>();
        sport_counts_result_msg_->algo_switch = SportMsg::COUNT_COMPLETE_CLOSE;
        sport_counts_result_msg_->sport_type = static_cast<int32_t>(this->sport_type_);
        this->sport_counts_result_pub_->publish(*sport_counts_result_msg_);
        this->ResetAlgo();
      }
    }
  } else {
    INFO("No human body detected.");
  }
}

int32_t sport::ProcessKeypoint(
  std::vector<std::vector<XMPoint>> & multHuman,
  uint32_t & height, uint32_t & width)
{
  int32_t counts = 0;
  for (int i = 0; i < 1; i++) {
    auto human = multHuman[i];
    cyberdog::interaction::Angle angle;
    std::vector<std::vector<int>> nums;
    switch (this->sport_type_) {
      case SportSrv::Request::SPORT_SQUAT:  // 深蹲
        {
          Squart squart(this->squat_dataFusion_, this->squat_dataFiler_,
            this->squat_threMinAngle_, this->squat_threMaxAngle_);
          float value1 = angle.Angle_Left_Knee(human);
          float value2 = angle.Angle_Right_Knee(human);
          this->calculate_Angle1_.push_back(value1);
          this->calculate_Angle2_.push_back(value2);
          if (this->flag_ > 2) {      // 从检测到第二帧骨骼点开始处理
            std::vector<float> filtAngle1, filtAngle2;
            filtAngle1 = MedianFilter(
              this->calculate_Angle1_,
              squart.filterNumber);
            filtAngle2 = MedianFilter(
              this->calculate_Angle2_,
              squart.filterNumber);
            nums.push_back(DataFusion(filtAngle1, squart.fusionNumber));
            nums.push_back(DataFusion(filtAngle2, squart.fusionNumber));
            SquatCounts(nums, squart);
            counts = squart.count;
          }
          break;
        }
      case SportSrv::Request::SPORT_HIGHKNEES:  // 高抬腿
        {
          HighKnees highKnee(this->highKnee_dataFusion_, this->highKnee_dataFiler_,
            this->highKnee_threMinAngle_, this->highKnee_threMaxAngle_);
          float value1 = angle.Angle_Left_Knee(human);
          float value2 = angle.Angle_Right_Knee(human);
          this->calculate_Angle1_.push_back(value1);
          this->calculate_Angle2_.push_back(value2);
          if (this->flag_ > 2) {
            std::vector<float> filtAngle1, filtAngle2;
            filtAngle1 = MedianFilter(
              this->calculate_Angle1_,
              highKnee.filterNumber);
            filtAngle2 = MedianFilter(
              this->calculate_Angle2_,
              highKnee.filterNumber);
            nums.push_back(DataFusion(filtAngle1, highKnee.fusionNumber));
            nums.push_back(DataFusion(filtAngle2, highKnee.fusionNumber));
            HighKneesCounts(nums, highKnee);
            counts = highKnee.count;
          }
          break;
        }
      case SportSrv::Request::SPORT_SITUP:  // 仰卧起坐
        {
          if (height > width) {
            break;
          }
          SitUp sitUp(this->sitUp_dataFusion_, this->sitUp_dataFiler_,
            this->sitUp_threMinAngle_, this->sitUp_threMidAngle1_,
            this->sitUp_threMidAngle2_, this->sitUp_threMaxAngle_);
          float value1 = angle.Angle_Left_Hip(human);
          float value2 = angle.Angle_Right_Hip(human);
          this->calculate_Angle1_.push_back(value1);
          this->calculate_Angle2_.push_back(value2);
          if (this->flag_ > 2) {
            std::vector<float> filtAngle1, filtAngle2;
            filtAngle1 = MedianFilter(this->calculate_Angle1_, sitUp.filterNumber);
            filtAngle2 = MedianFilter(this->calculate_Angle2_, sitUp.filterNumber);
            nums.push_back(DataFusion(filtAngle1, sitUp.fusionNumber));
            nums.push_back(DataFusion(filtAngle2, sitUp.fusionNumber));
            SitUpCounts(nums, sitUp);
            counts = sitUp.count;
          }
          break;
        }
      case SportSrv::Request::SPORT_PRESSUP:  // 俯卧撑
        {
          if (height > width) {
            break;
          }
          PushUp pushUp(this->pressUp_dataFusion_, this->pressUp_dataFiler_,
            this->pressUp_difMinAngle_, this->pressUp_difMaxAngle_,
            this->pressUp_threMaxAngle_);
          float value1 = angle.Angle_Left_PushUps(human);
          float value2 = angle.Angle_Right_PushUps(human);
          this->calculate_Angle1_.push_back(value1);
          this->calculate_Angle2_.push_back(value2);
          if (this->flag_ > 2) {
            std::vector<float> filtAngle1, filtAngle2;
            filtAngle1 = MedianFilter(this->calculate_Angle1_, pushUp.filterNumber);
            filtAngle2 = MedianFilter(this->calculate_Angle2_, pushUp.filterNumber);
            nums.push_back(DataFusion(filtAngle1, pushUp.fusionNumber));
            nums.push_back(DataFusion(filtAngle2, pushUp.fusionNumber));
            PushUpCounts(nums, pushUp);
            counts = pushUp.count;
          }
          break;
        }
      case SportSrv::Request::SPORT_PLANK:  // 平板支撑
        {
          if (height > width) {
            break;
          }
          Plank plank(this->plank_dataFusion_, this->plank_dataFiler_,
            this->plank_threMinAngle_, this->plank_threMaxAngle_);
          float value1 = angle.Angle_Left_Elbow(human);
          float value2 = angle.Angle_Right_Elbow(human);
          this->calculate_Angle1_.push_back(value1);
          this->calculate_Angle2_.push_back(value2);
          if (this->flag_ > 2) {
            std::vector<float> filtAngle1, filtAngle2;
            filtAngle1 = MedianFilter(this->calculate_Angle1_, plank.filterNumber);
            filtAngle2 = MedianFilter(this->calculate_Angle2_, plank.filterNumber);
            nums.push_back(DataFusion(filtAngle1, plank.fusionNumber));
            nums.push_back(DataFusion(filtAngle2, plank.fusionNumber));
            PlankTime(nums, plank);
            counts = plank.count;
          }
          break;
        }
      case SportSrv::Request::SPORT_JUMPJACK:  // 开合跳
        {
          JumpJack jumpJack(this->jump_dataFusion_, this->jump_dataFiler_,
            this->jump_threMidAngle1_,
            this->jump_threMidAngle2_, this->jump_threMidAngle3_, this->jump_threMidAngle4_,
            this->jump_threMidAngle5_, this->jump_threMidAngle6_, this->jump_threMidAngle7_);
          float value1 = angle.Angle_Left_Shoulder(human);
          float value2 = angle.Angle_Right_Shoulder(human);
          float value3 = angle.Angle_Nose(human);
          this->calculate_Angle1_.push_back(value1);
          this->calculate_Angle2_.push_back(value2);
          this->calculate_Angle3_.push_back(value3);
          if (this->flag_ > 2) {
            std::vector<float> filtAngle1, filtAngle2, filtAngle3;
            filtAngle1 = MedianFilter(this->calculate_Angle1_, jumpJack.filterNumber);
            filtAngle2 = MedianFilter(this->calculate_Angle2_, jumpJack.filterNumber);
            filtAngle3 = MedianFilter(this->calculate_Angle3_, jumpJack.filterNumber);
            nums.push_back(DataFusion(filtAngle1, jumpJack.fusionNumber));
            nums.push_back(DataFusion(filtAngle2, jumpJack.fusionNumber));
            nums.push_back(DataFusion(filtAngle3, jumpJack.fusionNumber));
            JumpJackCounts(nums, jumpJack);
            counts = jumpJack.count;
          }
          break;
        }
      default: {
          break;
        }
    }
  }
  INFO("Keypoint processing has already finished.");
  return counts;
}

void sport::ResetAlgo()
{
  this->sport_counts_ = 0;
  this->sport_type_ = 0;
  this->flag_ = 0;
  this->sport_counts_inter_time_ = 300;
  this->ai_sports_request_->algo_enable.clear();
  this->calculate_Angle1_.clear();
  this->calculate_Angle2_.clear();
  this->calculate_Angle3_.clear();
  this->wake_sport_counts_call_ = false;
  this->update_counts_ = 0;
}

bool sport::Is_Configure()
{
  // 延时等待vision启动
  while (!this->ai_get_state_->wait_for_service(std::chrono::seconds(1))) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  INFO("vision_manager already start.");
  std::this_thread::sleep_for(std::chrono::milliseconds(30000));
  auto get_state_request = std::make_shared<GStateSrv::Request>();
  auto get_respone = this->ai_get_state_->async_send_request(get_state_request);
  if (get_respone.wait_for(std::chrono::seconds(100)) != std::future_status::ready) {
    ERROR("Failed to call service /get_state");
    return false;
  }
  if (!this->ai_change_state_->wait_for_service(std::chrono::seconds(10))) {
    ERROR("ai_change_state service not ready.");
    return false;
  }
  if (get_respone.get()->current_state.id == StateMsg::PRIMARY_STATE_UNCONFIGURED) {
    auto change_state_request = std::make_shared<CStateSrv::Request>();
    change_state_request->transition.id = TransMsg::TRANSITION_CONFIGURE;
    auto change_respone = this->ai_change_state_->async_send_request(change_state_request);
    if (change_respone.wait_for(std::chrono::seconds(150)) != std::future_status::ready) {
      ERROR("Failed to call service /change_state");
    } else {
      if (change_respone.get()->success) {
        INFO("Transition active successfully triggered.");
      } else {
        WARN("Failed to transform, currently in inactive");
      }
    }
  }
  return true;
}

bool sport::Is_Active()
{
  // ai模式置为active
  if (!this->ai_get_state_->wait_for_service(std::chrono::seconds(10))) {
    ERROR("ai_get_state service not ready.");
  }
  INFO("Request to set AI Mode to active");
  auto get_state_request = std::make_shared<GStateSrv::Request>();
  auto get_respone = this->ai_get_state_->async_send_request(get_state_request);
  auto cur_status = get_respone.wait_for(std::chrono::seconds(10));
  if (cur_status != std::future_status::ready) {
    ERROR("Failed to call service /get_state");
    return false;
  }
  if (!this->ai_change_state_->wait_for_service(std::chrono::seconds(10))) {
    ERROR("ai_change_state service not ready.");
  }
  if (get_respone.get()->current_state.id == StateMsg::PRIMARY_STATE_INACTIVE) {
    auto change_state_request = std::make_shared<CStateSrv::Request>();
    change_state_request->transition.id = TransMsg::TRANSITION_ACTIVATE;
    auto change_respone = this->ai_change_state_->async_send_request(change_state_request);
    if (change_respone.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
      if (change_respone.get()->success) {
        INFO("Transition active successfully triggered.");
      } else {
        WARN("Failed to transform.");
      }
    } else {
      WARN("Failed to call service /change_state.");
    }
  }
  get_respone = this->ai_get_state_->async_send_request(get_state_request);
  if (get_respone.get()->current_state.id != StateMsg::PRIMARY_STATE_ACTIVE) {
    ERROR("The AI algorithm is not in active state");
    return false;
  }
  return true;
}

bool sport::Is_Deactive()
{
  // ai模式置为inactive
  if (!this->ai_get_state_->wait_for_service(std::chrono::seconds(10))) {
    ERROR("ai_get_state service not ready.");
  }
  INFO("Request to set AI Mode to Inactive");
  auto get_state_request = std::make_shared<GStateSrv::Request>();
  auto get_respone = this->ai_get_state_->async_send_request(get_state_request);
  auto cur_status = get_respone.wait_for(std::chrono::seconds(10));
  if (cur_status != std::future_status::ready) {
    ERROR("Failed to call service /get_state");
    return false;
  }
  if (!this->ai_change_state_->wait_for_service(std::chrono::seconds(10))) {
    ERROR("ai_change_state service not ready.");
  }
  if (get_respone.get()->current_state.id == StateMsg::PRIMARY_STATE_ACTIVE) {
    auto change_state_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_state_request->transition.id = TransMsg::TRANSITION_DEACTIVATE;
    auto change_respone = this->ai_change_state_->async_send_request(change_state_request);
    if (change_respone.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
      if (change_respone.get()->success) {
        INFO("Transition inactive successfully triggered.");
      } else {
        WARN("Failed to transform");
      }
    } else {
      WARN("Failed to call service /change_state");
    }
  }
  get_respone = this->ai_get_state_->async_send_request(get_state_request);
  if (get_respone.get()->current_state.id != StateMsg::PRIMARY_STATE_INACTIVE) {
    ERROR("The AI algorithm is not in active state");
    return false;
  }
  return true;
}

void sport::Run()
{
  INFO("cyberdog_ai_sports node spin");
  this->executor_.add_node(this->get_node_base_interface());
  this->executor_.spin();
  rclcpp::shutdown();
}
}  // namespace interaction
}  // namespace cyberdog
