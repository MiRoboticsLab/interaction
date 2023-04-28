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
#include <memory>

#include "cyberdog_vp_terminal/visual.hpp"

namespace cyberdog_visual_programming_terminal
{
Visual::Visual(std::string _task, std::string _namespace, bool _ros, std::string _parameters)
{
  try {
    if (this->cyberdog_ptr_ == nullptr) {
      this->cyberdog_ptr_ = std::make_shared<VPA::Cyberdog>(_task, _namespace, _ros, _parameters);
      this->cyberdog_ptr_->task_.Start();
    }
    if (this->cyberdog_ptr_ == nullptr) {
      ERROR("Init cyberdog_ptr_ failed, is nullptr");
      exit(-1);
    }
    if ((!this->debugger_.abilityset_.Init(this->cyberdog_ptr_) ||
      (!this->debugger_.engine_.Init(_namespace))))
    {
      ERROR("Init Visual failed");
      exit(-2);
    }
  } catch (const std::exception & e) {
    ERROR("Creat Visual failed: %s", e.what());
  }
}

Visual::~Visual()
{
}

}   // namespace cyberdog_visual_programming_terminal
