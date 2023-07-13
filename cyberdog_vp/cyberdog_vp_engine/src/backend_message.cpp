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

#include "cyberdog_vp_engine/backend_message.hpp"

namespace cyberdog_visual_programming_engine
{
BackendMessage::BackendMessage(const std::string & msg)
: state_(CommonEnum::efficient)
{
  this->describe_ = "Received backend message";
  INFO("%s: %s", this->describe_.c_str(), msg.c_str());
  rapidjson::Document json_doc;
  if (cyberdog::common::CyberdogJson::String2Document(msg, json_doc)) {
    // auto get_value = [&](CommonEnum key, rapidjson::Value & value) -> bool {
    //     if (!cyberdog::common::CyberdogJson::Get(
    //         json_doc, this->keys.at(key).c_str(), value))
    //     {
    //       this->state_ = key;
    //       this->describe_ += "but get '" +
    //         this->keys.at(key) + "' is fail, message:\n" + msg;
    //       return false;
    //     }
    //     return true;
    //   };
    // // 可按照如下方式解析嵌套结构
    // // rapidjson::Value data_val;
    // // rapidjson::Document data_doc;
    // // if (get_value(CommonEnum::data, data_val) &&
    // //   cyberdog::common::CyberdogJson::Value2Document(data_val, data_doc) &&
    // //   get_str(data_doc, CommonEnum::type, this->backend_.data.type));
    auto get_str =
      [&](const rapidjson::Document & doc, CommonEnum key, std::string & now_data,
        bool must = true) -> bool {
        now_data = "";
        if (doc.IsObject()) {
          if (doc.HasMember(this->keys.at(key).c_str())) {
            if (doc[this->keys.at(key).c_str()].IsString()) {
              now_data = doc[this->keys.at(key).c_str()].GetString();
              if (must && now_data.empty()) {
                this->state_ = key;
                this->describe_ = "Received message, but [" + this->keys.at(key) +
                  "] is empty! message:\n" + msg;
              }
            } else {
              this->state_ = key;
              this->describe_ = "Received message, but input doc [" + this->keys.at(key) +
                "] primary key is not string! message:\n" + msg;
            }
          } else {
            this->state_ = key;
            this->describe_ = "Received message, but input doc no [" + this->keys.at(key) +
              "] primary key! message:\n" + msg;
          }
        } else {
          this->state_ = key;
          this->describe_ = "Received message, but input doc should be kObejectType! message:\n" +
            msg;
        }
        if (this->state_ != CommonEnum::efficient) {
          WARN("%s", this->describe_.c_str());
          return false;
        }
        return true;
      };

    auto strs_to_str = [&]() -> std::string {
        std::ostringstream ret;
        size_t vct_size = this->backend_.data.size();
        for (size_t i = 0; i < vct_size; i++) {
          ret << "[" << i << "] : \n" << this->backend_.data.at(i) << std::endl;
        }
        return ret.str();
      };

    if (get_str(json_doc, CommonEnum::code, this->backend_.code) &&
      get_str(json_doc, CommonEnum::message, this->backend_.message) &&
      get_str(json_doc, CommonEnum::request_id, this->backend_.request_id))
    {
      if (json_doc.HasMember(this->keys.at(CommonEnum::data).c_str())) {
        if (json_doc[this->keys.at(CommonEnum::data).c_str()].IsArray()) {
          rapidjson::Value & data_list = json_doc[this->keys.at(CommonEnum::data).c_str()];
          for (size_t i = 0; i < data_list.Size(); ++i) {
            if (data_list[i].IsObject()) {
              rapidjson::Value & objectValue = data_list[i];
              rapidjson::StringBuffer buffer;
              rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
              writer.SetMaxDecimalPlaces(4);
              objectValue.Accept(writer);
              this->backend_.data.push_back(std::string(buffer.GetString()));
            }
          }
        } else {
          this->state_ = CommonEnum::invalid;
          this->describe_ +=
            "but not in compliance with the agreement(list), message:\n"
            +
            msg;
        }
      } else {
        this->state_ = CommonEnum::invalid;
        this->describe_ +=
          "but not in compliance with the agreement(data/list), message:\n"
          +
          msg;
      }
    } else {
      this->state_ = CommonEnum::invalid;
      this->describe_ +=
        "but not in compliance with the agreement(code/message/request_id), message:\n"
        +
        msg;
    }
    // {
    //   "code": "code",
    //   "message": "message",
    //   "request_id": "request_id",
    //   "data": [
    //     {
    //       "type": "type",
    //       "id": "id",
    //       "describe": "describe",
    //       "style": "style",
    //       "operate": "operate",
    //       "mode": "mode",
    //       "condition": "condition",
    //       "body": "body"
    //       "target_id": ["target_id", ...]
    //     },
    //     ...
    //   ]
    // }
    INFO(
      "BackendMessage is:"
      "\ncode=<%s>"
      "\nmessage=<%s>"
      "\nrequest_id=<%s>"
      "\ndata=<%s>",
      this->backend_.code.c_str(),
      this->backend_.message.c_str(),
      this->backend_.request_id.c_str(),
      strs_to_str().c_str()
    );
  } else {
    this->state_ = CommonEnum::invalid;
    this->describe_ += "but does not conform to json rules, message:\n" +
      msg;
  }
  if (this->state_ != CommonEnum::efficient) {
    WARN("%s", this->describe_.c_str());
  } else {
    INFO("The currently received message is valid.");
  }
}

void BackendMessage::getRobotendMsg(GRPCMsg & msg_)
{
  // {
  //   "feedback": {
  //     "type": "type",
  //     "id": "id",
  //     "operate": "operate",
  //     "state": 0,
  //     "describe": "describe"
  //   }
  // }
  rapidjson::Document task_json(rapidjson::kObjectType);
  CyberdogJson::Add(task_json, "type", OperateMsg::STEP_PARSE);
  CyberdogJson::Add(task_json, "id", this->backend_.request_id);
  CyberdogJson::Add(task_json, "operate", "");
  CyberdogJson::Add(task_json, "state", static_cast<int>(this->state_));
  CyberdogJson::Add(task_json, "describe", this->describe_);
  rapidjson::Document robotend_json(rapidjson::kObjectType);
  CyberdogJson::Add(robotend_json, "feedback", task_json);
  CyberdogJson::Document2String(robotend_json, msg_.data);
}

}   // namespace cyberdog_visual_programming_engine
