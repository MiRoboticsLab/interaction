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
import os
import sys
import __main__


def version():
    """ Describe: 列表
    """
    version = "V-1.0.0.0"
    return version

def list():
    """ Describe: 列表
    """
    print('''
 ●
 ┣━━> get_down()
 ┗━━> resume_standing()
''')


# electrochromic(
#      [key]   [value] 0    1    2    3     4      5    6    7     8
#   model       [模式] 闪烁 前后变 随机 后前变 上位机  动态
#   position    [部位] 背部 左后腿 左侧 左前腿 前胸   右前腿 右侧 右后腿  全身
#   rendering   [渲染] 淡出 淡入
#   outset      [起点] 前端 后端
#   duration_ms [时长] 
#   )


def get_down():
    """ Describe: 趴下
    """
    cyberdog = __main__.cyberdog
    for i in range(5):
        cyberdog.skin.electrochromic(4,0,0,0,1000)
        cyberdog.skin.electrochromic(4,1,0,0,1000)
        cyberdog.skin.electrochromic(4,2,0,0,1000)
        cyberdog.skin.electrochromic(4,3,0,0,1000)
        cyberdog.skin.electrochromic(4,4,0,0,1000)
        cyberdog.skin.electrochromic(4,5,0,0,1000)
        cyberdog.skin.electrochromic(4,6,0,0,1000)
        cyberdog.skin.electrochromic(4,7,0,0,1000)


def resume_standing():
    """ Describe: 恢复站立
    """
    cyberdog = __main__.cyberdog
    for i in range(5):
        cyberdog.skin.electrochromic(4,0,1,0,1000)
        cyberdog.skin.electrochromic(4,1,1,0,1000)
        cyberdog.skin.electrochromic(4,2,1,0,1000)
        cyberdog.skin.electrochromic(4,3,1,0,1000)
        cyberdog.skin.electrochromic(4,4,1,0,1000)
        cyberdog.skin.electrochromic(4,5,1,0,1000)
        cyberdog.skin.electrochromic(4,6,1,0,1000)
        cyberdog.skin.electrochromic(4,7,1,0,1000)
