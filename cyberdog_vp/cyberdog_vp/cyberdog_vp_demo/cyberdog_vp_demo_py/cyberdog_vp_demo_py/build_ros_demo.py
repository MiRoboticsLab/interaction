# Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from mi.cyberdog_vp.abilityset import Cyberdog
from mi.cyberdog_vp.abilityset import StateCode


def main(args=None):
    print("Cyberdog abilityset demo node started.")
    cyberdog = Cyberdog("abilityset_build_ros_demo", "demo", True, "")
    if cyberdog.state.code != StateCode.success:
        print("Cyberdog abilityset demo node is ", cyberdog.state.describe, ".")
        return 1
    print("Do somthine ...")
    cyberdog.audio.offline_play()
    print("Cyberdog abilityset demo node stopped.")
    cyberdog.shutdown()
    return 0


if __name__ == "__main__":
    main()
