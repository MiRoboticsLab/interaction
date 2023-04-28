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
#include <functional>
#include <thread>
// #include <lcm/lcm-cpp.hpp>
#include "cyberdog_common/cyberdog_lcm.hpp"
#include "audio_lcm/lcm_data.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "xpack/json.h"

using LcmServer = cyberdog::common::LcmServer<audio_lcm::lcm_data, audio_lcm::lcm_data>;
using LcmClient = cyberdog::common::LcmClient<audio_lcm::lcm_data, audio_lcm::lcm_data>;

// #define  LCM_ADDR  "udpm://239.255.76.67:7667?ttl=1"

struct local_control_data
{
  int8_t type;
  XPACK(O(type))
};

struct self_check_data
{
  int8_t status;
  XPACK(O(status))
};

struct get_net_status_data
{
  bool result;
  XPACK(O(result))
};

struct net_status_data
{
  bool connected;
  XPACK(O(connected))
};

class LcmCenter final
{
public:
  explicit LcmCenter(std::shared_ptr<lcm::LCM> lcm)
  : _lcm(lcm), counter(0)
  {
    _lcm->subscribe("cyberdog_audio_topic", &LcmCenter::handler, this);
    auto srv_func_ = [this]() {
        server = std::make_shared<LcmServer>(
          "cyberdog_audio_service",
          std::bind(
            &LcmCenter::ServerCallback, this, std::placeholders::_1,
            std::placeholders::_2));
        server->Spin();
      };
    server_thread = std::thread(srv_func_);
    // NetStatus();
    // auto func_ = [this](){
    //     audio_lcm::lcm_data l_d;
    //     local_control_data lc_d;
    //     lc_d.type = 0;
    //     l_d.cmd = "play";
    //     l_d.data = xpack::json::encode(lc_d);
    //     _lcm->publish("audio_cyberdog_topic", &l_d);
    // };
    // pub_thread = std::thread(func_);
  }

private:
  void handler(
    const lcm::ReceiveBuffer * rbuf, const std::string & chan,
    const audio_lcm::lcm_data * msg)
  {
    (void)rbuf;
    INFO(
      "Received message(%d), on channel \"%s\", cmd[%s], data[%s] ",
      counter,
      chan.c_str(), msg->cmd.c_str(), msg->data.c_str());
    ++counter;
    audio_lcm::lcm_data data;
    if (msg->cmd == "play") {
      local_control_data acd;
      acd.type = 0;  // 唤醒
      data.cmd = "local_control";
      data.data = xpack::json::encode(acd);
      _lcm->publish("audio_cyberdog_topic", &data);
    } else if (msg->cmd == "netstatus") {
    }
    // if(counter == 0)
    // start = std::chrono::system_clock::now();
    // lcm_data_1 data;
    // xpack::json::decode(msg->data, data);
    // for(int i = 0; i < 10;c i++)
    // INFO("%10lf", data.e[i]);
    // counter++;
    // if(counter == 100000) {
    // auto end =  std::chrono::system_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::cout << static_cast<double>(duration.count()) << std::endl;
    // }
  }

  void ServerCallback(const audio_lcm::lcm_data & req, audio_lcm::lcm_data & res)
  {
    INFO(
      "Received request, cmd[%s], data[%s] ",
      req.cmd.c_str(), req.data.empty() ? "" : req.data.c_str());
    static int8_t status = -1;
    if (status != 0) {
      status = (std::rand() % 4) - 1;
    }
    if (req.cmd == "self_check") {
      self_check_data data;
      data.status = status;
      res.cmd = req.cmd;
      res.data = xpack::json::encode(data);
    }
  }

  bool ClientRequest(const audio_lcm::lcm_data & req, audio_lcm::lcm_data & res)
  {
    LcmClient client("audio_cyberdog_service");
    auto result = client.Request(req, res, 1000);
    if (!result) {
      INFO("client thread failed once");
      return false;
    } else {
      INFO("client receive: cmd[%s], data[%s]", res.cmd.c_str(), res.data.c_str());
    }
    return true;
  }

  bool NetStatus()
  {
    audio_lcm::lcm_data req;
    audio_lcm::lcm_data res;
    req.cmd = "get_netstatus";
    bool result = ClientRequest(req, res);
    return result;
  }

private:
  std::shared_ptr<lcm::LCM> _lcm;
  uint32_t counter;
  std::shared_ptr<LcmServer> server;
  // std::thread pub_thread;
  std::thread server_thread;
};

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  std::shared_ptr<lcm::LCM> _lcm(new lcm::LCM(LCM_ADDR));
  if (!_lcm->good()) {
    INFO("lcm is not good!");
    return -1;
  }
  LcmCenter listner(_lcm);
  while (0 == _lcm->handle()) {}
  return 0;
}
