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
#include <memory>
#include "cyberdog_action/cyberdog_hand_action.hpp"

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("cyberdog_action");
  INFO("cyberdog_action node started");
  rclcpp::init(argc, argv);
  auto action_ptr = std::make_shared<cyberdog::interaction::gesture>("cyberdog_action");
  if (!action_ptr->Init()) {
    INFO("init failed,exit 0 ");
    return -1;
  } else {
    INFO("init successfully");
  }
  action_ptr->Run();
  return 0;
}
