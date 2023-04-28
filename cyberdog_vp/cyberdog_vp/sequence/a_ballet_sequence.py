# Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import time
from mi.cyberdog_bringup.manual import get_namespace
from mi.cyberdog_vp.abilityset import MotionSequence
from mi.cyberdog_vp.abilityset import MotionSequenceGait
from mi.cyberdog_vp.abilityset import MotionSequencePace
from mi.cyberdog_vp.abilityset import Cyberdog
cyberdog = Cyberdog('mi_test_sequence', get_namespace(), True, '')
time.sleep(2)
cyberdog.set_log()
cyberdog.task.start()
cyberdog.audio.play('测试序列任务开始执行',55)
sequ = MotionSequence()
sequ.name = 'test_sequ'
sequ.describe = '测试序列'

# 原始步态配置文件为：./user_gait_00.toml.

gait_meta = MotionSequenceGait()
gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 0
gait_meta.right_hindfoot = 0
gait_meta.left_hindfoot = 1
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 0
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 0
gait_meta.duration = 4
sequ.gait_list.push_back(gait_meta)

gait_meta.right_forefoot = 1
gait_meta.left_forefoot = 1
gait_meta.right_hindfoot = 1
gait_meta.left_hindfoot = 1
gait_meta.duration = 1
sequ.gait_list.push_back(gait_meta)

# 原始步伐配置文件为：./L91_user80_ballet_full.toml.

pace_meta = MotionSequencePace()
pace_meta.twist.linear.x = 0.000000
pace_meta.twist.linear.y = 0.000000
pace_meta.twist.linear.z = 0.000000
pace_meta.centroid.position.x = 0.000000
pace_meta.centroid.position.y = 0.000000
pace_meta.centroid.position.z = 0.020000
pace_meta.centroid.orientation.x = 0.000000
pace_meta.centroid.orientation.y = 0.000000
pace_meta.centroid.orientation.z = 0.000000
pace_meta.weight.linear.x = 50.000000
pace_meta.weight.linear.y = 50.000000
pace_meta.weight.linear.z = 5.000000
pace_meta.weight.angular.x = 10.000000
pace_meta.weight.angular.y = 10.000000
pace_meta.weight.angular.z = 10.000000
pace_meta.left_forefoot.x = -0.030000
pace_meta.left_forefoot.y = -0.050000
pace_meta.left_forefoot.w = 0.030000
pace_meta.right_forefoot.x = 0.030000
pace_meta.right_forefoot.y = 0.050000
pace_meta.right_forefoot.w = 0.030000
pace_meta.left_hindfoot.x = -0.030000
pace_meta.left_hindfoot.y = -0.050000
pace_meta.left_hindfoot.w = 0.030000
pace_meta.right_hindfoot.x = 0.030000
pace_meta.right_hindfoot.y = 0.050000
pace_meta.right_hindfoot.w = 0.030000
pace_meta.friction_coefficient = 1.000000
pace_meta.landing_gain = 4.000000
pace_meta.use_mpc_track = 0
pace_meta.duration = 600
sequ.pace_list.push_back(pace_meta)

pace_meta.twist.linear.x = 0.000000
pace_meta.twist.linear.y = 0.000000
pace_meta.twist.linear.z = 0.300000
pace_meta.centroid.position.x = 0.000000
pace_meta.centroid.position.y = 0.000000
pace_meta.centroid.position.z = 0.040000
pace_meta.centroid.orientation.x = 0.000000
pace_meta.centroid.orientation.y = 0.000000
pace_meta.centroid.orientation.z = 0.000000
pace_meta.weight.linear.x = 50.000000
pace_meta.weight.linear.y = 50.000000
pace_meta.weight.linear.z = 5.000000
pace_meta.weight.angular.x = 10.000000
pace_meta.weight.angular.y = 10.000000
pace_meta.weight.angular.z = 10.000000
pace_meta.left_forefoot.x = -0.060000
pace_meta.left_forefoot.y = -0.100000
pace_meta.left_forefoot.w = 0.008000
pace_meta.right_forefoot.x = 0.060000
pace_meta.right_forefoot.y = 0.100000
pace_meta.right_forefoot.w = 0.008000
pace_meta.left_hindfoot.x = -0.060000
pace_meta.left_hindfoot.y = -0.100000
pace_meta.left_hindfoot.w = 0.008000
pace_meta.right_hindfoot.x = 0.060000
pace_meta.right_hindfoot.y = 0.100000
pace_meta.right_hindfoot.w = 0.008000
pace_meta.friction_coefficient = 1.000000
pace_meta.landing_gain = 4.000000
pace_meta.use_mpc_track = 0
pace_meta.duration = 6000
sequ.pace_list.push_back(pace_meta)

pace_meta.twist.linear.x = 0.000000
pace_meta.twist.linear.y = 0.000000
pace_meta.twist.linear.z = 0.000000
pace_meta.centroid.position.x = 0.000000
pace_meta.centroid.position.y = 0.000000
pace_meta.centroid.position.z = 0.020000
pace_meta.centroid.orientation.x = 0.000000
pace_meta.centroid.orientation.y = 0.000000
pace_meta.centroid.orientation.z = 0.000000
pace_meta.weight.linear.x = 50.000000
pace_meta.weight.linear.y = 50.000000
pace_meta.weight.linear.z = 5.000000
pace_meta.weight.angular.x = 10.000000
pace_meta.weight.angular.y = 10.000000
pace_meta.weight.angular.z = 10.000000
pace_meta.left_forefoot.x = -0.030000
pace_meta.left_forefoot.y = -0.050000
pace_meta.left_forefoot.w = 0.030000
pace_meta.right_forefoot.x = 0.030000
pace_meta.right_forefoot.y = 0.050000
pace_meta.right_forefoot.w = 0.030000
pace_meta.left_hindfoot.x = -0.030000
pace_meta.left_hindfoot.y = -0.050000
pace_meta.left_hindfoot.w = 0.030000
pace_meta.right_hindfoot.x = 0.030000
pace_meta.right_hindfoot.y = 0.050000
pace_meta.right_hindfoot.w = 0.030000
pace_meta.friction_coefficient = 1.000000
pace_meta.landing_gain = 4.000000
pace_meta.use_mpc_track = 0
pace_meta.duration = 300
sequ.pace_list.push_back(pace_meta)

pace_meta.twist.linear.x = 0.000000
pace_meta.twist.linear.y = 0.000000
pace_meta.twist.linear.z = 0.000000
pace_meta.centroid.position.x = 0.000000
pace_meta.centroid.position.y = 0.000000
pace_meta.centroid.position.z = 0.000000
pace_meta.centroid.orientation.x = 0.000000
pace_meta.centroid.orientation.y = 0.000000
pace_meta.centroid.orientation.z = 0.000000
pace_meta.weight.linear.x = 50.000000
pace_meta.weight.linear.y = 50.000000
pace_meta.weight.linear.z = 5.000000
pace_meta.weight.angular.x = 10.000000
pace_meta.weight.angular.y = 10.000000
pace_meta.weight.angular.z = 10.000000
pace_meta.left_forefoot.x = 0.000000
pace_meta.left_forefoot.y = 0.000000
pace_meta.left_forefoot.w = 0.030000
pace_meta.right_forefoot.x = 0.000000
pace_meta.right_forefoot.y = 0.000000
pace_meta.right_forefoot.w = 0.030000
pace_meta.left_hindfoot.x = 0.000000
pace_meta.left_hindfoot.y = 0.000000
pace_meta.left_hindfoot.w = 0.030000
pace_meta.right_hindfoot.x = 0.000000
pace_meta.right_hindfoot.y = 0.000000
pace_meta.right_hindfoot.w = 0.030000
pace_meta.friction_coefficient = 1.000000
pace_meta.landing_gain = 4.000000
pace_meta.use_mpc_track = 0
pace_meta.duration = 300
sequ.pace_list.push_back(pace_meta)

cyberdog.motion.run_sequence(sequ)
cyberdog.audio.play('测试序列任务执行结束',55)
cyberdog.task.stop()
