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

#include "cyberdog_vp_abilityset/follow.hpp"

namespace cyberdog_visual_programming_abilityset
{
bool Follow::SetData(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("%s Set data failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

bool Follow::SetMechanism(const toml::value &)
{
  try {
    Debug("%s", std::string(__FUNCTION__).c_str());
  } catch (const std::exception & e) {
    Error("%s Set mechanism failed: %s", this->logger_.c_str(), e.what());
    return false;
  }
  return true;
}

State Follow::AddPersonnel(const std::string _preset_name)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(const std::string preset_name = '%s')",
    _preset_name.c_str());
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  // code ***
  return this->GetState(funs, StateCode::success);
}

State Follow::DeletePersonnel(const std::string _preset_name)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(const std::string preset_name = '%s')",
    _preset_name.c_str());
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  // code ***
  return this->GetState(funs, StateCode::success);
}

State Follow::FollowPersonnel(
  const std::string _preset_name, const double _intimacy)
{
  std::string funs = std::string(__FUNCTION__) + FORMAT(
    "(const std::string preset_name = '%s', const double intimacy = %lf)",
    _preset_name.c_str(), _intimacy);
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  // code ***
  return this->GetState(funs, StateCode::success);
}

State Follow::CancelFollow()
{
  std::string funs = std::string(__FUNCTION__) + "()";
  Info("%s", funs.c_str());
  if (this->state_.code != StateCode::success) {
    return this->GetState(funs, this->state_.code);
  }
  // code ***
  return this->GetState(funs, StateCode::success);
}
}   // namespace cyberdog_visual_programming_abilityset
