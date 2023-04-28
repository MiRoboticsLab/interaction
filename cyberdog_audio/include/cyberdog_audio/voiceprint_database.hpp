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
#ifndef CYBERDOG_AUDIO__VOICEPRINT_DATABASE_HPP_
#define CYBERDOG_AUDIO__VOICEPRINT_DATABASE_HPP_

#include <string>
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

namespace cyberdog
{
namespace interaction
{
struct VoicePrint
{
  std::string name;
  std::string id;
};
class VoiceprintDatabase final
{
public:
  VoiceprintDatabase()
  {
    try {
      auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
      auto path = local_share_dir + std::string("/toml_config/audio");
      if (access(path.c_str(), F_OK) != 0) {
        std::string cmd = "mkdir -p " + path;
        std::system(cmd.c_str());
        cmd = "chmod 777 " + path;
        std::system(cmd.c_str());
      }
      database_path = path + "/" + "voiceprints.json";
    } catch (...) {
      ERROR("mkdir audio directory error.");
    }
  }

  void Insert(VoicePrint & vp)
  {
    Document json_document(kObjectType);
    CyberdogJson::ReadJsonFromFile(database_path, json_document);
    rapidjson::Value voiceprints_array(rapidjson::kArrayType);
    bool success = CyberdogJson::Get(json_document, "voiceprints", voiceprints_array);
    if (success) {
      bool find = false;
      size_t len = voiceprints_array.Size();
      for (size_t i = 0; i < len; i++) {
        rapidjson::Value & object = voiceprints_array[i];
        if (object.IsObject()) {
          if (object.HasMember("name") && object["name"].IsString()) {
            if (vp.name == object["name"].GetString()) {
              object["id"].SetString(vp.id.c_str(), vp.id.size());
              find = true;
              break;
            }
          }
        }
      }
      if (!find) {
        Document::AllocatorType & allocator = json_document.GetAllocator();
        rapidjson::Value vp_val(rapidjson::kObjectType);
        rapidjson::Value rval;
        vp_val.AddMember("name", rval.SetString(vp.name.c_str(), allocator), allocator);
        vp_val.AddMember("id", rval.SetString(vp.id.c_str(), allocator), allocator);
        voiceprints_array.PushBack(vp_val, allocator);
      }
    } else {
      Document::AllocatorType & allocator = json_document.GetAllocator();
      rapidjson::Value vp_val(rapidjson::kObjectType);
      rapidjson::Value rval;
      vp_val.AddMember("name", rval.SetString(vp.name.c_str(), allocator), allocator);
      vp_val.AddMember("id", rval.SetString(vp.id.c_str(), allocator), allocator);
      voiceprints_array.PushBack(vp_val, allocator);
    }
    CyberdogJson::Add(json_document, "voiceprints", voiceprints_array);
    std::string info("");
    if (!CyberdogJson::Document2String(json_document, info)) {
      ERROR("error while encoding to json");
      info = "{\"error\": \"unkown encoding json error!\"}";
    }
    INFO("voiceprints write json document:%s", info.c_str());
    CyberdogJson::WriteJsonToFile(database_path, json_document);
  }

private:
  std::string database_path;
};
}  // namespace interaction
}  // namespace cyberdog

#endif  // CYBERDOG_AUDIO__VOICEPRINT_DATABASE_HPP_
