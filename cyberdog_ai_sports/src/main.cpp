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
#include "cyberdog_ai_sports/cyberdog_ai_sports.hpp"
int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("cyberdog_ai_sports");
  INFO("cyberdog_ai_sports node started");
  rclcpp::init(argc, argv);
  auto sport_ptr = std::make_shared<cyberdog::interaction::sport>("cyberdog_ai_sports");
  std::thread run_thread(&cyberdog::interaction::sport::Run, std::ref(*sport_ptr));
  if (!sport_ptr->Init()) {
    INFO("cyberdog_ai_sports initialization failure.");
  }
  if (run_thread.joinable()) {
    run_thread.join();
  }
  return 0;
}
