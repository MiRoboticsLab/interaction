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
#include <vector>

#include "cyberdog_vp_engine/frontend_message.hpp"

namespace cyberdog_visual_programming_engine
{
FrontendMessage::FrontendMessage(const std::string & msg)
: state_(CommonEnum::efficient)
{
  INFO("Received frontend message: %s", msg.c_str());
  rapidjson::Document json_doc;
  if (cyberdog::common::CyberdogJson::String2Document(msg, json_doc)) {
    auto get_str = [&](CommonEnum key, std::string & now_data, bool must = true) -> bool {
        now_data = "";
        if (json_doc.IsObject()) {
          if (json_doc.HasMember(this->keys.at(key).c_str())) {
            if (json_doc[this->keys.at(key).c_str()].IsString()) {
              now_data = json_doc[this->keys.at(key).c_str()].GetString();
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
            if (must) {
              this->state_ = key;
              this->describe_ = "Received message, but input doc no [" + this->keys.at(key) +
                "] primary key! message:\n" + msg;
            }
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

    auto get_strs =
      [&](CommonEnum key, std::vector<std::string> & now_data, bool must = true) -> bool {
        now_data.clear();
        if (json_doc.IsObject()) {
          if (json_doc.HasMember(this->keys.at(key).c_str())) {
            if (json_doc[this->keys.at(key).c_str()].IsArray()) {
              rapidjson::Value & list = json_doc[this->keys.at(key).c_str()];
              std::string new_data = "";
              for (size_t i = 0; i < list.Size(); ++i) {
                if (list[i].IsString()) {
                  new_data = list[i].GetString();
                  if (new_data.empty()) {
                    this->state_ = key;
                    this->describe_ = "Received message, but input doc [" + this->keys.at(key) +
                      "] [" + std::to_string(i) + "] primary key is empty! message:\n" + msg;
                    break;
                  }
                  now_data.push_back(list[i].GetString());
                } else {
                  this->state_ = key;
                  this->describe_ = "Received message, but input doc [" + this->keys.at(key) +
                    "] [" + std::to_string(i) + "] primary key is not string! message:\n" + msg;
                  break;
                }
              }
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

    auto strs_to_str = [&](std::vector<std::string> & now_data) -> std::string {
        std::ostringstream ret;
        ret << "[";
        if (!now_data.empty()) {
          int max = now_data.size() - 1;
          for (int i = 0; i < max; i++) {
            ret << now_data[i] << ", ";
          }
          ret << now_data[max];
        }
        ret << "]";
        return ret.str();
      };

    auto regex_judge_condition =
      [&](const std::string & _condition, const std::string & _regex) -> bool {
        this->state_ = CommonEnum::efficient;
        std::regex now_reg(_regex);
        if (!std::regex_match(_condition, now_reg)) {
          this->state_ = CommonEnum::condition;
          this->describe_ = "Received message and resolved 'condition', but value is invalid, " +
            _condition;
        }
        return static_cast<bool>(this->state_ == CommonEnum::efficient);
      };
    auto judge_single_condition = [&]() -> bool {
        // now
        // HH:MM
        // HH:MM YYYY-MM-DD
        // now + number[minutes|hours|days|weeks|months|years]
        // HH:MM  + number[minutes|hours|days|weeks|months|years]
        // HH:MM YYYY-MM-DD + number[minutes|hours|days|weeks|months|years]
        std::ostringstream nowmsg;
        nowmsg <<
          "(" <<
          "now" << "|" <<
          "([0]{0,1}\\d|1\\d|2[0-3]):([0-5][0-9]{0,1})" <<
          "([\\s]*([\\d]+)\\-([0]{0,1}[1-9]|1[0-2])\\-([0]{0,1}[1-9]|[1-2]\\d|3[0-1]))" <<
          "{0,1}" <<
          ")" <<
          "([\\s]*\\+[\\s]*([\\d]+)(minutes|hours|days|weeks|months|years))" <<
          "{0,1}";
        std::string single_condition = nowmsg.str();
        return regex_judge_condition(this->frontend_.condition, single_condition);
      };

    auto judge_cycle_condition = [&]() -> bool {
        // * * * * *
        // - - - - -
        // | | | | |
        // | | | | +----- day of week (0 - 7) (Sunday=0 or 7) OR sun,mon,tue,wed,thu,fri,sat
        // | | | +---------- month (1 - 12) OR jan,feb,mar,apr ...
        // | | +--------------- day of month (1 - 31)
        // | +-------------------- hour (0 - 23)
        // +------------------------- minute (0 - 59)
        // ,-/
        std::ostringstream nowmsg;
        nowmsg <<
          "(" <<
          "\\*|" <<
          "(\\*|([0]{0,1}[0-5][0-9]{0,1}))/([1-5][0-9]{0,1})|" <<
          "([0]{0,1}[0-5][0-9]{0,1})(,([0]{0,1}[0-5][0-9]{0,1})){0,58}|" <<
          "([0]{0,1}[0-5][0-9]{0,1})(-([0]{0,1}[0-5][0-9]{0,1})){0,1}" <<
          ")" << "[\\s]" <<
          "(" <<
          "\\*|" <<
          "(\\*|([0]{0,1}\\d|1\\d|2[0-3]))/([1-9]|1\\d|2[0-3])|" <<
          "([0]{0,1}\\d|1\\d|2[0-3])(,([0]{0,1}\\d|1\\d|2[0-3])){0,22}|" <<
          "([0]{0,1}\\d|1\\d|2[0-3])(-([1-9]|1\\d|2[0-3])){0,1}" <<
          ")" << "[\\s]" <<
          "(" <<
          "\\*|" <<
          "(\\*|([0]{0,1}[1-9]|[1-2]\\d|3[0-1]))/([0]{0,1}[1-9]|[1-2]\\d|3[0-1])|" <<
          "([0]{0,1}[1-9]|[1-2]\\d|3[0-1])(,([0]{0,1}[1-9]|[1-2]\\d|3[0-1])){0,30}|" <<
          "([0]{0,1}[1-9]|[1-2]\\d|3[0-1])(-([0]{0,1}[1-9]|[1-2]\\d|3[0-1])){0,1}" <<
          ")" << "[\\s]" <<
          "(" <<
          "\\*|" <<
          "(\\*|([0]{0,1}[1-9]|1[0-2]))/([0]{0,1}[1-9]|1[0-2])|" <<
          "([0]{0,1}[1-9]|1[0-2])(,([0]{0,1}[1-9]|1[0-2])){0,11}|" <<
          "([0]{0,1}[1-9]|1[0-2])(-([0]{0,1}[1-9]|1[0-2])){0,1}" <<
          ")" << "[\\s]" <<
          "(" <<
          "\\*|" <<
          "(\\*|([0]{0,1}[1-7]))/([0]{0,1}[1-7])|" <<
          "([0]{0,1}[1-7])(,([0]{0,1}[1-7])){0,6}|" <<
          "([0]{0,1}[1-7])(-([0]{0,1}[1-7])){0,1}" <<
          ")"
        ;
        std::string cycle_condition = nowmsg.str();
        return regex_judge_condition(this->frontend_.condition, cycle_condition);
      };

    auto judge_module_condition = [&]() -> bool {
        // function()
        // function(a)
        // function(a,b)
        // function(a,b,...)
        std::ostringstream nowmsg;
        nowmsg <<
          "[a-zA-Z_]+\\w+" <<
          "[\\s]*[\\(]" <<
          "[\\s]*([a-zA-Z_]+\\w+){0,1}([\\s]*,[\\s]*[a-zA-Z_]+\\w+){0,}" <<
          "[\\s]*[\\)]";
        std::string function_condition = nowmsg.str();
        return regex_judge_condition(this->frontend_.condition, function_condition);
      };

    auto task_other_operate = [&]() -> bool {
        return static_cast<bool>(
          (this->frontend_.operate == OperateMsg::OPERATE_INQUIRY) ||
          (this->frontend_.operate == OperateMsg::OPERATE_DELETE) ||
          (this->frontend_.operate == OperateMsg::OPERATE_SHUTDOWN) ||
          (this->frontend_.operate == OperateMsg::OPERATE_SUSPEND) ||
          (this->frontend_.operate == OperateMsg::OPERATE_RECOVER));
      };

    auto judge_describe_uniqueness = [&]() -> bool {
        std::string workspace = "";
        if (!GetWorkspace(workspace)) {
          ERROR("%s Get workspace failed.", this->frontend_.id.c_str());
          return false;
        }
        std::string registry_file = workspace + "/" + this->frontend_.type + "/" +
          this->frontend_.type + ".toml";
        if (!JudgeConfileFile(registry_file)) {
          ERROR("%s Judge task registry file failed.", this->frontend_.id.c_str());
          return false;
        }
        using TomlList = std::vector<OperateMsg>;
        toml::value registry_toml;
        if (!cyberdog::common::CyberdogToml::ParseFile(
            registry_file.c_str(), registry_toml))
        {
          ERROR("%s Parse task registry file failed.", this->frontend_.id.c_str());
          return false;
        }
        const toml::value now_lists = toml::find(registry_toml, this->frontend_.type);
        if (!now_lists.is_table()) {
          ERROR("%s Toml is not table.", this->frontend_.id.c_str());
          return false;
        }
        TomlList tag_list;
        for (const auto & meta : now_lists.as_table()) {
          const std::string & id = meta.first;
          if ((id == "id") ||
            (id == "terminal_default") ||
            (id == "visual_default") ||
            (id == OperateMsg::OPERATE_DEBUG))
          {
            continue;
          }
          // const toml::table& table = meta.second;
          // for (const auto& keyVal : table) {
          //   const std::string& key = keyVal.first;
          //   const toml::value& value = keyVal.second;
          //   std::cout << "  " << key << " = " << value << std::endl;
          // }
          std::string describe = toml::find_or(
            registry_toml, this->frontend_.type, id, "describe",
            "");
          if (describe == this->frontend_.describe) {
            ERROR(
              "%s Describe the conflict, same as task %s.", this->frontend_.id.c_str(),
              id.c_str());
            return false;
          }
        }
        return true;
      };

    auto judge_task = [&]() -> bool {
        this->state_ = CommonEnum::efficient;
        if (this->frontend_.operate == OperateMsg::OPERATE_SAVE) {
          if (this->frontend_.mode == OperateMsg::MODE_SINGLE) {
            judge_single_condition();
          } else if (this->frontend_.mode == OperateMsg::MODE_CYCLE) {
            judge_cycle_condition();
          } else {
            this->state_ = CommonEnum::mode;
            this->describe_ =
              "[Judge Task] Received message and resolved 'mode', but value is invalid, " +
              this->frontend_.mode;
          }
        } else if (this->frontend_.operate == OperateMsg::OPERATE_RUN) {
          if (!this->frontend_.mode.empty()) {
            if (this->frontend_.mode == OperateMsg::MODE_SINGLE) {
              judge_single_condition();
            } else if (this->frontend_.mode == OperateMsg::MODE_CYCLE) {
              judge_cycle_condition();
            } else {
              this->state_ = CommonEnum::mode;
              this->describe_ =
                "[Judge Task] Received message and resolved 'mode', but value is invalid, " +
                this->frontend_.mode;
            }
          }
        } else if ((this->frontend_.operate == OperateMsg::OPERATE_DEBUG)) {
          std::string error_msg_head =
            "[Judge Task] Received debug task message and resolved ";
          if ((this->frontend_.target_id.empty() ||
            (this->frontend_.target_id.front() != OperateMsg::OPERATE_DEBUG)))
          {
            this->state_ = CommonEnum::target_id;
            this->describe_ = error_msg_head +
              "'target_id', but value is invalid, " +
              this->frontend_.mode;
          }
          if ((this->frontend_.mode != OperateMsg::MODE_SINGLE)) {
            this->state_ = CommonEnum::mode;
            this->describe_ = error_msg_head +
              "'mode', but value is invalid, " +
              this->frontend_.mode;
          }
          if ((this->frontend_.condition != "now")) {
            this->state_ = CommonEnum::condition;
            this->describe_ = error_msg_head +
              "'condition', but value is invalid, " +
              this->frontend_.mode;
          }
        } else if (task_other_operate()) {
          // 仅需 id, target_id 支持，无需其他
        } else {
          this->state_ = CommonEnum::operate;
          this->describe_ =
            "[Judge Task] Received message and resolved 'operate', but value is invalid, " +
            this->frontend_.operate;
        }
        return static_cast<bool>(this->state_ == CommonEnum::efficient);
      };

    auto module_other_operate = [&]() -> bool {
        return static_cast<bool>((this->frontend_.operate == OperateMsg::OPERATE_DELETE) ||
               (this->frontend_.operate == OperateMsg::OPERATE_INQUIRY));
      };

    auto judge_module = [&]() -> bool {
        this->state_ = CommonEnum::efficient;
        if (this->frontend_.id.empty()) {
          this->state_ = CommonEnum::id;
          this->describe_ =
            "[Judge Module] Received message and resolved 'id', but value is invalid, " +
            this->frontend_.id;
        }
        if ((this->frontend_.operate == OperateMsg::OPERATE_SAVE)) {
          if ((this->frontend_.mode == OperateMsg::MODE_COMMON) ||
            (this->frontend_.mode == OperateMsg::MODE_SEQUENCE))
          {
            judge_module_condition();
          } else {
            this->state_ = CommonEnum::mode;
            this->describe_ =
              "[Judge Module] Received message and resolved 'mode', but value is invalid, " +
              this->frontend_.mode;
          }
        } else if (module_other_operate()) {
          // 仅需 id, target_id 支持，无需其他
        } else {
          this->state_ = CommonEnum::operate;
          this->describe_ =
            "[Judge Module] Received message and resolved 'operate', but value is invalid, " +
            this->frontend_.operate;
        }
        return static_cast<bool>(this->state_ == CommonEnum::efficient);
      };

    auto judge_ai = [&]() -> bool {
        if (this->frontend_.operate == OperateMsg::OPERATE_INQUIRY) {
          if ((this->frontend_.mode == OperateMsg::MODE_ALL) ||
            (this->frontend_.mode == OperateMsg::MODE_PERSONNEL) ||
            (this->frontend_.mode == OperateMsg::MODE_FACE) ||
            (this->frontend_.mode == OperateMsg::MODE_VOICEPRINT) ||
            (this->frontend_.mode == OperateMsg::MODE_TAAINING_WORDS))
          {
            // 仅需 id 支持，无需其他
          } else {
            this->state_ = CommonEnum::mode;
            this->describe_ =
              "[Judge Module] Received message and resolved 'mode', but value is invalid, " +
              this->frontend_.mode;
          }
        } else {
          this->state_ = CommonEnum::operate;
          this->describe_ =
            "[Judge Module] Received message and resolved 'operate', but value is invalid, " +
            this->frontend_.operate;
        }
        return static_cast<bool>(this->state_ == CommonEnum::efficient);
      };

    auto judge_slam = [&]() -> bool {
        if (this->frontend_.operate == OperateMsg::OPERATE_INQUIRY) {
          if ((this->frontend_.mode == OperateMsg::MODE_MAP) ||
            (this->frontend_.mode == OperateMsg::MODE_PRESET))
          {
            // 仅需 id 支持，无需其他
          } else {
            this->state_ = CommonEnum::mode;
            this->describe_ =
              "[Judge Module] Received message and resolved 'mode', but value is invalid, " +
              this->frontend_.mode;
          }
        } else {
          this->state_ = CommonEnum::operate;
          this->describe_ =
            "[Judge Module] Received message and resolved 'operate', but value is invalid, " +
            this->frontend_.operate;
        }
        return static_cast<bool>(this->state_ == CommonEnum::efficient);
      };
    // {
    //   "type": "type",
    //   "id": "id",
    //   "operate": "operate",
    //   "target_id": ["id","id"],
    //   "describe": "describe",
    //   "style": "style",
    //   "mode": "mode",
    //   "condition": "condition",
    //   "body": "body"
    // }
    if (get_str(CommonEnum::type, this->frontend_.type) &&
      get_str(CommonEnum::id, this->frontend_.id) &&
      get_str(CommonEnum::operate, this->frontend_.operate) &&
      get_strs(
        CommonEnum::target_id, this->frontend_.target_id,
        static_cast<bool>(this->frontend_.operate != OperateMsg::OPERATE_INQUIRY)) &&
      get_str(
        CommonEnum::describe, this->frontend_.describe,
        static_cast<bool>(
          (this->frontend_.operate == OperateMsg::OPERATE_SAVE) ||
          (this->frontend_.operate == OperateMsg::OPERATE_DEBUG))) &&
      get_str(
        CommonEnum::style, this->frontend_.style,
        static_cast<bool>(
          (this->frontend_.operate == OperateMsg::OPERATE_SAVE) ||
          (this->frontend_.operate == OperateMsg::OPERATE_DEBUG))) &&
      get_str(
        CommonEnum::mode, this->frontend_.mode,
        static_cast<bool>(
          (this->frontend_.operate == OperateMsg::OPERATE_SAVE) ||
          (this->frontend_.operate == OperateMsg::OPERATE_DEBUG))) &&
      get_str(
        CommonEnum::condition, this->frontend_.condition,
        static_cast<bool>(
          (this->frontend_.operate == OperateMsg::OPERATE_SAVE) ||
          (this->frontend_.operate == OperateMsg::OPERATE_DEBUG))) &&
      get_str(
        CommonEnum::body, this->frontend_.body,
        static_cast<bool>(
          (this->frontend_.operate == OperateMsg::OPERATE_SAVE) ||
          (this->frontend_.operate == OperateMsg::OPERATE_DEBUG))))
    {
      // if (!this->frontend_.style.empty()) {
      //   this->frontend_.style = Subreplace(this->frontend_.style, "\"", "\\\"");
      // }
      if (((this->frontend_.operate == OperateMsg::OPERATE_SAVE) ||
        (this->frontend_.operate == OperateMsg::OPERATE_DEBUG)) &&
        !judge_describe_uniqueness())
      {
        this->state_ = CommonEnum::describe;
        this->describe_ =
          "[Judge Task] Received message and resolved 'describe', but value is invalid, " +
          this->frontend_.describe;
      }

      if (this->frontend_.type == OperateMsg::TYPE_TASK) {
        judge_task();
      } else if (this->frontend_.type == OperateMsg::TYPE_MODULE) {
        judge_module();
      } else if (this->frontend_.type == OperateMsg::TYPE_AI) {
        judge_ai();
      } else if (this->frontend_.type == OperateMsg::TYPE_SLAM) {
        judge_slam();
      } else {
        this->state_ = CommonEnum::invalid;
        this->describe_ =
          "Received message, but type does not meet the constraint rules, message:\n" +
          msg;
      }
    }
    INFO(
      "FrontendMessage is:\ntype=%s"
      "\nid=%s"
      "\noperate=%s"
      "\ntarget_id=%s"
      "\ndescribe=%s"
      "\nstyle=%s"
      "\nmode=%s"
      "\ncondition=%s"
      "\nbody=%s",
      this->frontend_.type.c_str(),
      this->frontend_.id.c_str(),
      this->frontend_.operate.c_str(),
      strs_to_str(this->frontend_.target_id).c_str(),
      this->frontend_.describe.c_str(),
      this->frontend_.style.c_str(),
      this->frontend_.mode.c_str(),
      this->frontend_.condition.c_str(),
      this->frontend_.body.c_str()
    );
  } else {
    this->state_ = CommonEnum::invalid;
    this->describe_ = "Received robotend message, but does not conform to json rules, message:\n" +
      msg;
  }
  if (this->state_ != CommonEnum::efficient) {
    WARN("%s", this->describe_.c_str());
  } else {
    INFO("The currently received message is valid.");
  }
}

void FrontendMessage::getRobotendMsg(GRPCMsg & msg_)
{
  // {
  //   "feedback": {
  //     "type": "type",
  //     "id": "id",
  //     "operate": "operate",
  //     "state": 0,
  //     "describe": "describe"
  //   },
  // }
  rapidjson::Document task_json(rapidjson::kObjectType);
  CyberdogJson::Add(task_json, "type", OperateMsg::STEP_PARSE);
  CyberdogJson::Add(task_json, "id", this->frontend_.id);
  CyberdogJson::Add(task_json, "operate", this->frontend_.operate);
  CyberdogJson::Add(task_json, "state", static_cast<int>(this->state_));
  CyberdogJson::Add(task_json, "describe", this->describe_);
  rapidjson::Document robotend_json(rapidjson::kObjectType);
  CyberdogJson::Add(robotend_json, "feedback", task_json);
  CyberdogJson::Document2String(robotend_json, msg_.data);
}

}   // namespace cyberdog_visual_programming_engine
