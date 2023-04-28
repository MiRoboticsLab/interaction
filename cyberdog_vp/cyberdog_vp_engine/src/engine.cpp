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
#include "cyberdog_vp_engine/interact.hpp"

int main(int argc, char ** argv)
{
  namespace VPE = cyberdog_visual_programming_engine;
  LOGGER_MAIN_INSTANCE("main@visual_programming_engine");
  rclcpp::init(argc, argv);
  INFO("Visual programming engine node started");
  VPE::Interact();
  INFO("Visual programming engine node is stop.");
  return 0;
}
