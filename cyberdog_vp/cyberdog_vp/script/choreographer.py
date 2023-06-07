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
import platform
import re
import sys
import toml
import importlib
from ament_index_python.packages import get_package_share_directory
from mi.cyberdog_vp.utils import get_workspace
from mi.cyberdog_vp.abilityset import StateCode
from mi.cyberdog_vp.abilityset import MotionSequenceServiceResponse
sys.path.append(os.path.join(get_workspace(), 'choreographer'))
import dancer

#
# 获取 正负号
#
def get_sign(num):
    if num >= 0:
        return 1
    else:
        return -1


#
# 获取 模板路径
#
def get_template_path():
    return os.path.join(
        get_package_share_directory('cyberdog_vp'),
        'config',
        'choreographer',
    )


#
# 获取 路径下文件
#
def get_file(base):
    for root, ds, fs in os.walk(base):
        for f in fs:
            yield f


#
# 获取 模板文件
#
def get_template_file(base):
    template_path = get_template_path()
    gait=template_path+'/' + base + '_gait.toml'
    pace=template_path+'/' + base + '_pace.toml'
    file = get_workspace() + '/choreographer/dancer.py'
    template = {'return':False,'gait':gait,'pace':pace,'file':file}
    files = get_file(template_path)
    if (gait not in files) and (pace not in files):
        template['return'] = True
    return template


#
# 太空步
# 参数:
# - speed: 速度
# - stride: 步幅
# - number: 次数
#
def moonwalk(x_velocity, y_velocity, stride, number):
    print('Ask moonwalk(x_velocity=%f, y_velocity=%f, stride=%f, number=%d)' % (x_velocity, y_velocity, stride, number))

    if x_velocity < -0.08:
        x_velocity = -0.08
    elif x_velocity > 0.08:
        x_velocity = 0.08

    if y_velocity < -0.05:
        y_velocity = -0.05
    elif y_velocity > 0.05:
        y_velocity = 0.05

    if stride < 0.0:
        stride = 0.0
    elif stride > 0.065:
        stride = 0.065

    if number < 1:
        number = 1

    print('Run moonwalk(x_velocity=%f, y_velocity=%f, stride=%f, number=%d)' % (x_velocity, y_velocity, stride, number))

    choreographer_name_en = sys._getframe().f_code.co_name
    choreographer_name_zh = '太空步'
    template_file = get_template_file(choreographer_name_en)
    if not template_file['return']:
        return False
    print('当前舞蹈配置文件:\n ★ 步态:%s\n ★ 步伐:%s' % (template_file['gait'], template_file['pace']))

    sequence_py = open(template_file['file'], 'w+')
    sequence_py.write('# Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.')
    sequence_py.write('\n#')
    sequence_py.write('\n# Licensed under the Apache License, Version 2.0 (the \'License\');')
    sequence_py.write('\n# you may not use this file except in compliance with the License.')
    sequence_py.write('\n# You may obtain a copy of the License at')
    sequence_py.write('\n#')
    sequence_py.write('\n#     http://www.apache.org/licenses/LICENSE-2.0')
    sequence_py.write('\n#')
    sequence_py.write('\n# Unless required by applicable law or agreed to in writing, software')
    sequence_py.write('\n# distributed under the License is distributed on an \'AS IS\' BASIS,')
    sequence_py.write('\n# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.')
    sequence_py.write('\n# See the License for the specific language governing permissions and')
    sequence_py.write('\n# limitations under the License.\n')
    sequence_py.write('\nimport _ctypes')
    sequence_py.write('\nfrom mi.cyberdog_vp.abilityset import MotionSequence')
    sequence_py.write('\nfrom mi.cyberdog_vp.abilityset import MotionSequenceGait')
    sequence_py.write('\nfrom mi.cyberdog_vp.abilityset import MotionSequencePace')
    sequence_py.write('\n# 当前舞蹈配置文件:')
    sequence_py.write('\n# ★ 步态:%s' % (template_file['gait']))
    sequence_py.write('\n# ★ 步伐:%s' % (template_file['pace']))
    sequence_py.write('\ndef show(cyberdog_motion_id):')
    sequence_py.write('\n    """ Describe: Responsible for caching arbitrary choreography. """')
    sequence_py.write('\n    cyberdog_motion = _ctypes.PyObj_FromPtr(cyberdog_motion_id)')
    sequence_py.write('\n    sequ = MotionSequence()')
    sequence_py.write('\n    sequ.name = \'%s\'' % choreographer_name_en)
    sequence_py.write('\n    sequ.describe = \'%s\'\n' % choreographer_name_zh)

    gait_toml = toml.load(template_file['gait'])
    head = int(gait_toml['info']['head'])
    body = int(gait_toml['info']['body'])
    tail = int(gait_toml['info']['tail'])
    sequence_py.write('\n    gait_meta = MotionSequenceGait()')
    for index in range(0, head):
        sequence_py.write('\n    # contact[head][0][%d]' %                     int(index))
        sequence_py.write('\n    gait_meta.right_forefoot = %d' %           bool(gait_toml['section'][index]['contact'][0]))
        sequence_py.write('\n    gait_meta.left_forefoot = %d' %            bool(gait_toml['section'][index]['contact'][1]))
        sequence_py.write('\n    gait_meta.right_hindfoot = %d' %           bool(gait_toml['section'][index]['contact'][2]))
        sequence_py.write('\n    gait_meta.left_hindfoot = %d' %            bool(gait_toml['section'][index]['contact'][3]))
        sequence_py.write('\n    gait_meta.duration = %d' %                 int(gait_toml['section'][index]['duration']))
        sequence_py.write('\n    sequ.gait_list.push_back(gait_meta)\n')

    for count in range(int(number)):
        begin = head
        end = head + body
        for _index in range(begin, end):
            index = head+_index
            sequence_py.write('\n    # contact[body][%d][%d]' %             (int(count), int(_index)))
            sequence_py.write('\n    gait_meta.right_forefoot = %d' %       bool(gait_toml['section'][index]['contact'][0]))
            sequence_py.write('\n    gait_meta.left_forefoot = %d' %        bool(gait_toml['section'][index]['contact'][1]))
            sequence_py.write('\n    gait_meta.right_hindfoot = %d' %       bool(gait_toml['section'][index]['contact'][2]))
            sequence_py.write('\n    gait_meta.left_hindfoot = %d' %        bool(gait_toml['section'][index]['contact'][3]))
            sequence_py.write('\n    gait_meta.duration = %d' %             int(gait_toml['section'][index]['duration']))
            sequence_py.write('\n    sequ.gait_list.push_back(gait_meta)\n')

    begin = head+body
    end = head + body + tail
    for index in range(begin, end):
        sequence_py.write('\n    # contact[tail][0][%d]' %                     int(index))
        sequence_py.write('\n    gait_meta.right_forefoot = %d' %           bool(gait_toml['section'][index]['contact'][0]))
        sequence_py.write('\n    gait_meta.left_forefoot = %d' %            bool(gait_toml['section'][index]['contact'][1]))
        sequence_py.write('\n    gait_meta.right_hindfoot = %d' %           bool(gait_toml['section'][index]['contact'][2]))
        sequence_py.write('\n    gait_meta.left_hindfoot = %d' %            bool(gait_toml['section'][index]['contact'][3]))
        sequence_py.write('\n    gait_meta.duration = %d' %                 int(gait_toml['section'][index]['duration']))
        sequence_py.write('\n    sequ.gait_list.push_back(gait_meta)\n')

    pace_toml = toml.load(template_file['pace'])
    head = int(pace_toml['info']['head'])
    body = int(pace_toml['info']['body'])
    tail = int(pace_toml['info']['tail'])
    sequence_py.write('\n    pace_meta = MotionSequencePace()')
    for index in range(0, head):
        sequence_py.write('\n    # step[head][0][%d]' %                        int(index))
        right_forefoot_height = float(format(pace_toml['step'][index]['step_height'][0] / 1e6, '.3f'))
        left_forefoot_height = float(format((pace_toml['step'][index]['step_height'][0] / 1e6 - right_forefoot_height) * 1e3, '.3f'))
        right_hindfoot_height = float(format(pace_toml['step'][index]['step_height'][1] / 1e6, '.3f'))
        left_hindfoot_height = float(format((pace_toml['step'][index]['step_height'][1] / 1e6 - right_hindfoot_height) * 1e3, '.3f'))
        sequence_py.write('\n    pace_meta.twist.linear.x = %f' %           float(x_velocity))
        sequence_py.write('\n    pace_meta.twist.linear.y = %f' %           float(y_velocity))
        sequence_py.write('\n    pace_meta.twist.linear.z = %f' %           float(pace_toml['step'][index]['vel_des'][2]))
        sequence_py.write('\n    pace_meta.centroid.position.x = %f' %      float(pace_toml['step'][index]['pos_des'][0]))
        sequence_py.write('\n    pace_meta.centroid.position.y = %f' %      float(pace_toml['step'][index]['pos_des'][1]))
        sequence_py.write('\n    pace_meta.centroid.position.z = %f' %      float(pace_toml['step'][index]['pos_des'][2]))
        sequence_py.write('\n    pace_meta.centroid.orientation.x = %f' %   float(pace_toml['step'][index]['rpy_des'][0]))
        sequence_py.write('\n    pace_meta.centroid.orientation.y = %f' %   float(pace_toml['step'][index]['rpy_des'][1]))
        sequence_py.write('\n    pace_meta.centroid.orientation.z = %f' %   float(pace_toml['step'][index]['rpy_des'][2]))
        sequence_py.write('\n    pace_meta.weight.linear.x = %f' %          float(pace_toml['step'][index]['acc_des'][0]))
        sequence_py.write('\n    pace_meta.weight.linear.y = %f' %          float(pace_toml['step'][index]['acc_des'][1]))
        sequence_py.write('\n    pace_meta.weight.linear.z = %f' %          float(pace_toml['step'][index]['acc_des'][2]))
        sequence_py.write('\n    pace_meta.weight.angular.x = %f' %         float(pace_toml['step'][index]['acc_des'][3]))
        sequence_py.write('\n    pace_meta.weight.angular.y = %f' %         float(pace_toml['step'][index]['acc_des'][4]))
        sequence_py.write('\n    pace_meta.weight.angular.z = %f' %         float(pace_toml['step'][index]['acc_des'][5]))
        sequence_py.write('\n    pace_meta.left_forefoot.x = %f' %          float(pace_toml['step'][index]['foot_pose'][2]))
        sequence_py.write('\n    pace_meta.left_forefoot.y = %f' %          float(pace_toml['step'][index]['foot_pose'][3]))
        sequence_py.write('\n    pace_meta.left_forefoot.w = %f' %          float(left_forefoot_height))
        sequence_py.write('\n    pace_meta.right_forefoot.x = %f' %         float(pace_toml['step'][index]['foot_pose'][0]))
        sequence_py.write('\n    pace_meta.right_forefoot.y = %f' %         float(pace_toml['step'][index]['foot_pose'][1]))
        sequence_py.write('\n    pace_meta.right_forefoot.w = %f' %         float(right_forefoot_height))
        sequence_py.write('\n    pace_meta.left_hindfoot.x = %f' %          float(pace_toml['step'][index]['ctrl_point'][0]))
        sequence_py.write('\n    pace_meta.left_hindfoot.y = %f' %          float(pace_toml['step'][index]['ctrl_point'][1]))
        sequence_py.write('\n    pace_meta.left_hindfoot.w = %f' %          float(left_hindfoot_height))
        sequence_py.write('\n    pace_meta.right_hindfoot.x = %f' %         float(pace_toml['step'][index]['foot_pose'][4]))
        sequence_py.write('\n    pace_meta.right_hindfoot.y = %f' %         float(pace_toml['step'][index]['foot_pose'][5]))
        sequence_py.write('\n    pace_meta.right_hindfoot.w = %f' %         float(right_hindfoot_height))
        sequence_py.write('\n    pace_meta.friction_coefficient = %f' %     float(pace_toml['step'][index]['ctrl_point'][2]))
        sequence_py.write('\n    pace_meta.landing_gain = %f' %             float(pace_toml['step'][index]['contact'] / 1e1))
        sequence_py.write('\n    pace_meta.use_mpc_track = %d' %            int(pace_toml['step'][index]['value']))
        sequence_py.write('\n    pace_meta.duration = %ld' %                int(pace_toml['step'][index]['duration']))
        sequence_py.write('\n    sequ.pace_list.push_back(pace_meta)\n')

    for count in range(int(number)):
        begin = head
        end = head + body
        for _index in range(begin, end):
            index = head+_index
            sequence_py.write('\n    # step[body][%d][%d]' %             (int(count), int(_index)))
            right_forefoot_height = float(format(pace_toml['step'][index]['step_height'][0] / 1e6, '.3f'))
            left_forefoot_height = float(format((pace_toml['step'][index]['step_height'][0] / 1e6 - right_forefoot_height) * 1e3, '.3f'))
            right_hindfoot_height = float(format(pace_toml['step'][index]['step_height'][1] / 1e6, '.3f'))
            left_hindfoot_height = float(format((pace_toml['step'][index]['step_height'][1] / 1e6 - right_hindfoot_height) * 1e3, '.3f'))
            sequence_py.write('\n    pace_meta.twist.linear.x = %f' %           float(x_velocity))
            sequence_py.write('\n    pace_meta.twist.linear.y = %f' %           float(y_velocity))
            sequence_py.write('\n    pace_meta.twist.linear.z = %f' %           float(pace_toml['step'][index]['vel_des'][2]))
            sequence_py.write('\n    pace_meta.centroid.position.x = %f' %      float(pace_toml['step'][index]['pos_des'][0]))
            sequence_py.write('\n    pace_meta.centroid.position.y = %f' %      float(pace_toml['step'][index]['pos_des'][1]))
            sequence_py.write('\n    pace_meta.centroid.position.z = %f' %      float(pace_toml['step'][index]['pos_des'][2]))
            sequence_py.write('\n    pace_meta.centroid.orientation.x = %f' %   float(pace_toml['step'][index]['rpy_des'][0]))
            sequence_py.write('\n    pace_meta.centroid.orientation.y = %f' %   float(pace_toml['step'][index]['rpy_des'][1]))
            sequence_py.write('\n    pace_meta.centroid.orientation.z = %f' %   float(pace_toml['step'][index]['rpy_des'][2]))
            sequence_py.write('\n    pace_meta.weight.linear.x = %f' %          float(pace_toml['step'][index]['acc_des'][0]))
            sequence_py.write('\n    pace_meta.weight.linear.y = %f' %          float(pace_toml['step'][index]['acc_des'][1]))
            sequence_py.write('\n    pace_meta.weight.linear.z = %f' %          float(pace_toml['step'][index]['acc_des'][2]))
            sequence_py.write('\n    pace_meta.weight.angular.x = %f' %         float(pace_toml['step'][index]['acc_des'][3]))
            sequence_py.write('\n    pace_meta.weight.angular.y = %f' %         float(pace_toml['step'][index]['acc_des'][4]))
            sequence_py.write('\n    pace_meta.weight.angular.z = %f' %         float(pace_toml['step'][index]['acc_des'][5]))
            sequence_py.write('\n    pace_meta.left_forefoot.x = %f' %          float(get_sign(pace_toml['step'][index]['foot_pose'][2]) * stride))
            sequence_py.write('\n    pace_meta.left_forefoot.y = %f' %          float(pace_toml['step'][index]['foot_pose'][3]))
            sequence_py.write('\n    pace_meta.left_forefoot.w = %f' %          float(left_forefoot_height))
            sequence_py.write('\n    pace_meta.right_forefoot.x = %f' %         float(get_sign(pace_toml['step'][index]['foot_pose'][0]) * stride))
            sequence_py.write('\n    pace_meta.right_forefoot.y = %f' %         float(pace_toml['step'][index]['foot_pose'][1]))
            sequence_py.write('\n    pace_meta.right_forefoot.w = %f' %         float(right_forefoot_height))
            sequence_py.write('\n    pace_meta.left_hindfoot.x = %f' %          float(get_sign(pace_toml['step'][index]['ctrl_point'][0]) * stride))
            sequence_py.write('\n    pace_meta.left_hindfoot.y = %f' %          float(pace_toml['step'][index]['ctrl_point'][1]))
            sequence_py.write('\n    pace_meta.left_hindfoot.w = %f' %          float(left_hindfoot_height))
            sequence_py.write('\n    pace_meta.right_hindfoot.x = %f' %         float(get_sign(pace_toml['step'][index]['foot_pose'][4]) * stride))
            sequence_py.write('\n    pace_meta.right_hindfoot.y = %f' %         float(pace_toml['step'][index]['foot_pose'][5]))
            sequence_py.write('\n    pace_meta.right_hindfoot.w = %f' %         float(right_hindfoot_height))
            sequence_py.write('\n    pace_meta.friction_coefficient = %f' %     float(pace_toml['step'][index]['ctrl_point'][2]))
            sequence_py.write('\n    pace_meta.landing_gain = %f' %             float(pace_toml['step'][index]['contact'] / 1e1))
            sequence_py.write('\n    pace_meta.use_mpc_track = %d' %            int(pace_toml['step'][index]['value']))
            sequence_py.write('\n    pace_meta.duration = %ld' %                int(pace_toml['step'][index]['duration']))
            sequence_py.write('\n    sequ.pace_list.push_back(pace_meta)\n')

    begin = head+body
    end = head + body + tail
    for index in range(begin, end):
        sequence_py.write('\n    # step[tail][0][%d]' %                     int(index))
        right_forefoot_height = float(format(pace_toml['step'][index]['step_height'][0] / 1e6, '.3f'))
        left_forefoot_height = float(format((pace_toml['step'][index]['step_height'][0] / 1e6 - right_forefoot_height) * 1e3, '.3f'))
        right_hindfoot_height = float(format(pace_toml['step'][index]['step_height'][1] / 1e6, '.3f'))
        left_hindfoot_height = float(format((pace_toml['step'][index]['step_height'][1] / 1e6 - right_hindfoot_height) * 1e3, '.3f'))
        sequence_py.write('\n    pace_meta.twist.linear.x = %f' %           float(pace_toml['step'][index]['vel_des'][0]))
        sequence_py.write('\n    pace_meta.twist.linear.y = %f' %           float(pace_toml['step'][index]['vel_des'][1]))
        sequence_py.write('\n    pace_meta.twist.linear.z = %f' %           float(pace_toml['step'][index]['vel_des'][2]))
        sequence_py.write('\n    pace_meta.centroid.position.x = %f' %      float(pace_toml['step'][index]['pos_des'][0]))
        sequence_py.write('\n    pace_meta.centroid.position.y = %f' %      float(pace_toml['step'][index]['pos_des'][1]))
        sequence_py.write('\n    pace_meta.centroid.position.z = %f' %      float(pace_toml['step'][index]['pos_des'][2]))
        sequence_py.write('\n    pace_meta.centroid.orientation.x = %f' %   float(pace_toml['step'][index]['rpy_des'][0]))
        sequence_py.write('\n    pace_meta.centroid.orientation.y = %f' %   float(pace_toml['step'][index]['rpy_des'][1]))
        sequence_py.write('\n    pace_meta.centroid.orientation.z = %f' %   float(pace_toml['step'][index]['rpy_des'][2]))
        sequence_py.write('\n    pace_meta.weight.linear.x = %f' %          float(pace_toml['step'][index]['acc_des'][0]))
        sequence_py.write('\n    pace_meta.weight.linear.y = %f' %          float(pace_toml['step'][index]['acc_des'][1]))
        sequence_py.write('\n    pace_meta.weight.linear.z = %f' %          float(pace_toml['step'][index]['acc_des'][2]))
        sequence_py.write('\n    pace_meta.weight.angular.x = %f' %         float(pace_toml['step'][index]['acc_des'][3]))
        sequence_py.write('\n    pace_meta.weight.angular.y = %f' %         float(pace_toml['step'][index]['acc_des'][4]))
        sequence_py.write('\n    pace_meta.weight.angular.z = %f' %         float(pace_toml['step'][index]['acc_des'][5]))
        sequence_py.write('\n    pace_meta.left_forefoot.x = %f' %          float(pace_toml['step'][index]['foot_pose'][2]))
        sequence_py.write('\n    pace_meta.left_forefoot.y = %f' %          float(pace_toml['step'][index]['foot_pose'][3]))
        sequence_py.write('\n    pace_meta.left_forefoot.w = %f' %          float(left_forefoot_height))
        sequence_py.write('\n    pace_meta.right_forefoot.x = %f' %         float(pace_toml['step'][index]['foot_pose'][0]))
        sequence_py.write('\n    pace_meta.right_forefoot.y = %f' %         float(pace_toml['step'][index]['foot_pose'][1]))
        sequence_py.write('\n    pace_meta.right_forefoot.w = %f' %         float(right_forefoot_height))
        sequence_py.write('\n    pace_meta.left_hindfoot.x = %f' %          float(pace_toml['step'][index]['ctrl_point'][0]))
        sequence_py.write('\n    pace_meta.left_hindfoot.y = %f' %          float(pace_toml['step'][index]['ctrl_point'][1]))
        sequence_py.write('\n    pace_meta.left_hindfoot.w = %f' %          float(left_hindfoot_height))
        sequence_py.write('\n    pace_meta.right_hindfoot.x = %f' %         float(pace_toml['step'][index]['foot_pose'][4]))
        sequence_py.write('\n    pace_meta.right_hindfoot.y = %f' %         float(pace_toml['step'][index]['foot_pose'][5]))
        sequence_py.write('\n    pace_meta.right_hindfoot.w = %f' %         float(right_hindfoot_height))
        sequence_py.write('\n    pace_meta.friction_coefficient = %f' %     float(pace_toml['step'][index]['ctrl_point'][2]))
        sequence_py.write('\n    pace_meta.landing_gain = %f' %             float(pace_toml['step'][index]['contact'] / 1e1))
        sequence_py.write('\n    pace_meta.use_mpc_track = %d' %            int(pace_toml['step'][index]['value']))
        sequence_py.write('\n    pace_meta.duration = %ld' %                int(pace_toml['step'][index]['duration']))
        sequence_py.write('\n    sequ.pace_list.push_back(pace_meta)\n')

    sequence_py.write('\n    return cyberdog_motion.run_sequence(sequ)\n')
    sequence_py.close()
    return True


#
# 俯卧撑
# 参数:
# - frequency: 频率
# - number: 次数
#
def push_up(frequency, number):
    print('Ask moonwalk(frequency=%d, number=%d)' % (frequency, number))

    if frequency < 10:
        frequency = 10
    elif frequency > 60:
        frequency = 60
    duration = int(5000 / frequency)
    if number < 2:
        number = 2
    elif number > 10:
        number = 10

    print('Run moonwalk(frequency=%d, number=%d)' % (frequency, number))

    choreographer_name_en = sys._getframe().f_code.co_name
    choreographer_name_zh = '俯卧撑'
    template_file = get_template_file(choreographer_name_en)
    if not template_file['return']:
        return False
    print('当前舞蹈配置文件:\n ★ 步态:%s\n ★ 步伐:%s' % (template_file['gait'], template_file['pace']))

    sequence_py = open(template_file['file'], 'w+')
    sequence_py.write('# Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.')
    sequence_py.write('\n#')
    sequence_py.write('\n# Licensed under the Apache License, Version 2.0 (the \'License\');')
    sequence_py.write('\n# you may not use this file except in compliance with the License.')
    sequence_py.write('\n# You may obtain a copy of the License at')
    sequence_py.write('\n#')
    sequence_py.write('\n#     http://www.apache.org/licenses/LICENSE-2.0')
    sequence_py.write('\n#')
    sequence_py.write('\n# Unless required by applicable law or agreed to in writing, software')
    sequence_py.write('\n# distributed under the License is distributed on an \'AS IS\' BASIS,')
    sequence_py.write('\n# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.')
    sequence_py.write('\n# See the License for the specific language governing permissions and')
    sequence_py.write('\n# limitations under the License.\n')
    sequence_py.write('\nimport _ctypes')
    sequence_py.write('\nfrom mi.cyberdog_vp.abilityset import MotionSequence')
    sequence_py.write('\nfrom mi.cyberdog_vp.abilityset import MotionSequenceGait')
    sequence_py.write('\nfrom mi.cyberdog_vp.abilityset import MotionSequencePace')
    sequence_py.write('\n# 当前舞蹈配置文件:')
    sequence_py.write('\n# ★ 步态:%s' % (template_file['gait']))
    sequence_py.write('\n# ★ 步伐:%s' % (template_file['pace']))
    sequence_py.write('\ndef show(cyberdog_motion_id):')
    sequence_py.write('\n    """ Describe: Responsible for caching arbitrary choreography. """')
    sequence_py.write('\n    cyberdog_motion = _ctypes.PyObj_FromPtr(cyberdog_motion_id)')
    sequence_py.write('\n    sequ = MotionSequence()')
    sequence_py.write('\n    sequ.name = \'%s\'' % choreographer_name_en)
    sequence_py.write('\n    sequ.describe = \'%s\'\n' % choreographer_name_zh)

    gait_toml = toml.load(template_file['gait'])
    head = int(gait_toml['info']['head'])
    body = int(gait_toml['info']['body'])
    tail = int(gait_toml['info']['tail'])
    sequence_py.write('\n    gait_meta = MotionSequenceGait()')
    for index in range(0, head):
        sequence_py.write('\n    # contact[head][0][%d]' %                     int(index))
        sequence_py.write('\n    gait_meta.right_forefoot = %d' %           bool(gait_toml['section'][index]['contact'][0]))
        sequence_py.write('\n    gait_meta.left_forefoot = %d' %            bool(gait_toml['section'][index]['contact'][1]))
        sequence_py.write('\n    gait_meta.right_hindfoot = %d' %           bool(gait_toml['section'][index]['contact'][2]))
        sequence_py.write('\n    gait_meta.left_hindfoot = %d' %            bool(gait_toml['section'][index]['contact'][3]))
        sequence_py.write('\n    gait_meta.duration = %d' %                 int(gait_toml['section'][index]['duration']))
        sequence_py.write('\n    sequ.gait_list.push_back(gait_meta)\n')

    for count in range(int(number)):
        begin = head
        end = head + body
        for _index in range(begin, end):
            index = head+_index
            sequence_py.write('\n    # contact[body][%d][%d]' %             (int(count), int(_index)))
            sequence_py.write('\n    gait_meta.right_forefoot = %d' %       bool(gait_toml['section'][index]['contact'][0]))
            sequence_py.write('\n    gait_meta.left_forefoot = %d' %        bool(gait_toml['section'][index]['contact'][1]))
            sequence_py.write('\n    gait_meta.right_hindfoot = %d' %       bool(gait_toml['section'][index]['contact'][2]))
            sequence_py.write('\n    gait_meta.left_hindfoot = %d' %        bool(gait_toml['section'][index]['contact'][3]))
            sequence_py.write('\n    gait_meta.duration = %d' %             int(gait_toml['section'][index]['duration']))
            sequence_py.write('\n    sequ.gait_list.push_back(gait_meta)\n')

    begin = head+body
    end = head + body + tail
    for index in range(begin, end):
        sequence_py.write('\n    # contact[tail][0][%d]' %                     int(index))
        sequence_py.write('\n    gait_meta.right_forefoot = %d' %           bool(gait_toml['section'][index]['contact'][0]))
        sequence_py.write('\n    gait_meta.left_forefoot = %d' %            bool(gait_toml['section'][index]['contact'][1]))
        sequence_py.write('\n    gait_meta.right_hindfoot = %d' %           bool(gait_toml['section'][index]['contact'][2]))
        sequence_py.write('\n    gait_meta.left_hindfoot = %d' %            bool(gait_toml['section'][index]['contact'][3]))
        sequence_py.write('\n    gait_meta.duration = %d' %                 int(gait_toml['section'][index]['duration']))
        sequence_py.write('\n    sequ.gait_list.push_back(gait_meta)\n')

    pace_toml = toml.load(template_file['pace'])
    head = int(pace_toml['info']['head'])
    body = int(pace_toml['info']['body'])
    tail = int(pace_toml['info']['tail'])
    sequence_py.write('\n    pace_meta = MotionSequencePace()')
    for index in range(0, head):
        sequence_py.write('\n    # step[head][0][%d]' %                        int(index))
        right_forefoot_height = float(format(pace_toml['step'][index]['step_height'][0] / 1e6, '.3f'))
        left_forefoot_height = float(format((pace_toml['step'][index]['step_height'][0] / 1e6 - right_forefoot_height) * 1e3, '.3f'))
        right_hindfoot_height = float(format(pace_toml['step'][index]['step_height'][1] / 1e6, '.3f'))
        left_hindfoot_height = float(format((pace_toml['step'][index]['step_height'][1] / 1e6 - right_hindfoot_height) * 1e3, '.3f'))
        sequence_py.write('\n    pace_meta.twist.linear.x = %f' %           float(pace_toml['step'][index]['vel_des'][0]))
        sequence_py.write('\n    pace_meta.twist.linear.y = %f' %           float(pace_toml['step'][index]['vel_des'][1]))
        sequence_py.write('\n    pace_meta.twist.linear.z = %f' %           float(pace_toml['step'][index]['vel_des'][2]))
        sequence_py.write('\n    pace_meta.centroid.position.x = %f' %      float(pace_toml['step'][index]['pos_des'][0]))
        sequence_py.write('\n    pace_meta.centroid.position.y = %f' %      float(pace_toml['step'][index]['pos_des'][1]))
        sequence_py.write('\n    pace_meta.centroid.position.z = %f' %      float(pace_toml['step'][index]['pos_des'][2]))
        sequence_py.write('\n    pace_meta.centroid.orientation.x = %f' %   float(pace_toml['step'][index]['rpy_des'][0]))
        sequence_py.write('\n    pace_meta.centroid.orientation.y = %f' %   float(pace_toml['step'][index]['rpy_des'][1]))
        sequence_py.write('\n    pace_meta.centroid.orientation.z = %f' %   float(pace_toml['step'][index]['rpy_des'][2]))
        sequence_py.write('\n    pace_meta.weight.linear.x = %f' %          float(pace_toml['step'][index]['acc_des'][0]))
        sequence_py.write('\n    pace_meta.weight.linear.y = %f' %          float(pace_toml['step'][index]['acc_des'][1]))
        sequence_py.write('\n    pace_meta.weight.linear.z = %f' %          float(pace_toml['step'][index]['acc_des'][2]))
        sequence_py.write('\n    pace_meta.weight.angular.x = %f' %         float(pace_toml['step'][index]['acc_des'][3]))
        sequence_py.write('\n    pace_meta.weight.angular.y = %f' %         float(pace_toml['step'][index]['acc_des'][4]))
        sequence_py.write('\n    pace_meta.weight.angular.z = %f' %         float(pace_toml['step'][index]['acc_des'][5]))
        sequence_py.write('\n    pace_meta.left_forefoot.x = %f' %          float(pace_toml['step'][index]['foot_pose'][2]))
        sequence_py.write('\n    pace_meta.left_forefoot.y = %f' %          float(pace_toml['step'][index]['foot_pose'][3]))
        sequence_py.write('\n    pace_meta.left_forefoot.w = %f' %          float(left_forefoot_height))
        sequence_py.write('\n    pace_meta.right_forefoot.x = %f' %         float(pace_toml['step'][index]['foot_pose'][0]))
        sequence_py.write('\n    pace_meta.right_forefoot.y = %f' %         float(pace_toml['step'][index]['foot_pose'][1]))
        sequence_py.write('\n    pace_meta.right_forefoot.w = %f' %         float(right_forefoot_height))
        sequence_py.write('\n    pace_meta.left_hindfoot.x = %f' %          float(pace_toml['step'][index]['ctrl_point'][0]))
        sequence_py.write('\n    pace_meta.left_hindfoot.y = %f' %          float(pace_toml['step'][index]['ctrl_point'][1]))
        sequence_py.write('\n    pace_meta.left_hindfoot.w = %f' %          float(left_hindfoot_height))
        sequence_py.write('\n    pace_meta.right_hindfoot.x = %f' %         float(pace_toml['step'][index]['foot_pose'][4]))
        sequence_py.write('\n    pace_meta.right_hindfoot.y = %f' %         float(pace_toml['step'][index]['foot_pose'][5]))
        sequence_py.write('\n    pace_meta.right_hindfoot.w = %f' %         float(right_hindfoot_height))
        sequence_py.write('\n    pace_meta.friction_coefficient = %f' %     float(pace_toml['step'][index]['ctrl_point'][2]))
        sequence_py.write('\n    pace_meta.landing_gain = %f' %             float(pace_toml['step'][index]['contact'] / 1e1))
        sequence_py.write('\n    pace_meta.use_mpc_track = %d' %            int(pace_toml['step'][index]['value']))
        sequence_py.write('\n    pace_meta.duration = %ld' %                int(pace_toml['step'][index]['duration']))
        sequence_py.write('\n    sequ.pace_list.push_back(pace_meta)\n')

    for count in range(int(number)):
        begin = head
        end = head + body
        for _index in range(begin, end):
            index = head+_index
            sequence_py.write('\n    # step[body][%d][%d]' %             (int(count), int(_index)))
            right_forefoot_height = float(format(pace_toml['step'][index]['step_height'][0] / 1e6, '.3f'))
            left_forefoot_height = float(format((pace_toml['step'][index]['step_height'][0] / 1e6 - right_forefoot_height) * 1e3, '.3f'))
            right_hindfoot_height = float(format(pace_toml['step'][index]['step_height'][1] / 1e6, '.3f'))
            left_hindfoot_height = float(format((pace_toml['step'][index]['step_height'][1] / 1e6 - right_hindfoot_height) * 1e3, '.3f'))
            sequence_py.write('\n    pace_meta.twist.linear.x = %f' %           float(pace_toml['step'][index]['vel_des'][0]))
            sequence_py.write('\n    pace_meta.twist.linear.y = %f' %           float(pace_toml['step'][index]['vel_des'][1]))
            sequence_py.write('\n    pace_meta.twist.linear.z = %f' %           float(pace_toml['step'][index]['vel_des'][2]))
            sequence_py.write('\n    pace_meta.centroid.position.x = %f' %      float(pace_toml['step'][index]['pos_des'][0]))
            sequence_py.write('\n    pace_meta.centroid.position.y = %f' %      float(pace_toml['step'][index]['pos_des'][1]))
            sequence_py.write('\n    pace_meta.centroid.position.z = %f' %      float(pace_toml['step'][index]['pos_des'][2]))
            sequence_py.write('\n    pace_meta.centroid.orientation.x = %f' %   float(pace_toml['step'][index]['rpy_des'][0]))
            sequence_py.write('\n    pace_meta.centroid.orientation.y = %f' %   float(pace_toml['step'][index]['rpy_des'][1]))
            sequence_py.write('\n    pace_meta.centroid.orientation.z = %f' %   float(pace_toml['step'][index]['rpy_des'][2]))
            sequence_py.write('\n    pace_meta.weight.linear.x = %f' %          float(pace_toml['step'][index]['acc_des'][0]))
            sequence_py.write('\n    pace_meta.weight.linear.y = %f' %          float(pace_toml['step'][index]['acc_des'][1]))
            sequence_py.write('\n    pace_meta.weight.linear.z = %f' %          float(pace_toml['step'][index]['acc_des'][2]))
            sequence_py.write('\n    pace_meta.weight.angular.x = %f' %         float(pace_toml['step'][index]['acc_des'][3]))
            sequence_py.write('\n    pace_meta.weight.angular.y = %f' %         float(pace_toml['step'][index]['acc_des'][4]))
            sequence_py.write('\n    pace_meta.weight.angular.z = %f' %         float(pace_toml['step'][index]['acc_des'][5]))
            sequence_py.write('\n    pace_meta.left_forefoot.x = %f' %          float(pace_toml['step'][index]['foot_pose'][2]))
            sequence_py.write('\n    pace_meta.left_forefoot.y = %f' %          float(pace_toml['step'][index]['foot_pose'][3]))
            sequence_py.write('\n    pace_meta.left_forefoot.w = %f' %          float(left_forefoot_height))
            sequence_py.write('\n    pace_meta.right_forefoot.x = %f' %         float(pace_toml['step'][index]['foot_pose'][0]))
            sequence_py.write('\n    pace_meta.right_forefoot.y = %f' %         float(pace_toml['step'][index]['foot_pose'][1]))
            sequence_py.write('\n    pace_meta.right_forefoot.w = %f' %         float(right_forefoot_height))
            sequence_py.write('\n    pace_meta.left_hindfoot.x = %f' %          float(pace_toml['step'][index]['ctrl_point'][0]))
            sequence_py.write('\n    pace_meta.left_hindfoot.y = %f' %          float(pace_toml['step'][index]['ctrl_point'][1]))
            sequence_py.write('\n    pace_meta.left_hindfoot.w = %f' %          float(left_hindfoot_height))
            sequence_py.write('\n    pace_meta.right_hindfoot.x = %f' %         float(pace_toml['step'][index]['foot_pose'][4]))
            sequence_py.write('\n    pace_meta.right_hindfoot.y = %f' %         float(pace_toml['step'][index]['foot_pose'][5]))
            sequence_py.write('\n    pace_meta.right_hindfoot.w = %f' %         float(right_hindfoot_height))
            sequence_py.write('\n    pace_meta.friction_coefficient = %f' %     float(pace_toml['step'][index]['ctrl_point'][2]))
            sequence_py.write('\n    pace_meta.landing_gain = %f' %             float(pace_toml['step'][index]['contact'] / 1e1))
            sequence_py.write('\n    pace_meta.use_mpc_track = %d' %            int(pace_toml['step'][index]['value']))
            if ((count < 1) and ((index - begin) < 2)) or ((number - count == 1) and ((end - index) < 2)):
                sequence_py.write('\n    pace_meta.duration = %ld' %            int(pace_toml['step'][index]['duration']))
            else:
                sequence_py.write('\n    pace_meta.duration = %ld' %            int(duration))
            sequence_py.write('\n    sequ.pace_list.push_back(pace_meta)\n')

    begin = head+body
    end = head + body + tail
    for index in range(begin, end):
        sequence_py.write('\n    # step[tail][0][%d]' %                     int(index))
        right_forefoot_height = float(format(pace_toml['step'][index]['step_height'][0] / 1e6, '.3f'))
        left_forefoot_height = float(format((pace_toml['step'][index]['step_height'][0] / 1e6 - right_forefoot_height) * 1e3, '.3f'))
        right_hindfoot_height = float(format(pace_toml['step'][index]['step_height'][1] / 1e6, '.3f'))
        left_hindfoot_height = float(format((pace_toml['step'][index]['step_height'][1] / 1e6 - right_hindfoot_height) * 1e3, '.3f'))
        sequence_py.write('\n    pace_meta.twist.linear.x = %f' %           float(pace_toml['step'][index]['vel_des'][0]))
        sequence_py.write('\n    pace_meta.twist.linear.y = %f' %           float(pace_toml['step'][index]['vel_des'][1]))
        sequence_py.write('\n    pace_meta.twist.linear.z = %f' %           float(pace_toml['step'][index]['vel_des'][2]))
        sequence_py.write('\n    pace_meta.centroid.position.x = %f' %      float(pace_toml['step'][index]['pos_des'][0]))
        sequence_py.write('\n    pace_meta.centroid.position.y = %f' %      float(pace_toml['step'][index]['pos_des'][1]))
        sequence_py.write('\n    pace_meta.centroid.position.z = %f' %      float(pace_toml['step'][index]['pos_des'][2]))
        sequence_py.write('\n    pace_meta.centroid.orientation.x = %f' %   float(pace_toml['step'][index]['rpy_des'][0]))
        sequence_py.write('\n    pace_meta.centroid.orientation.y = %f' %   float(pace_toml['step'][index]['rpy_des'][1]))
        sequence_py.write('\n    pace_meta.centroid.orientation.z = %f' %   float(pace_toml['step'][index]['rpy_des'][2]))
        sequence_py.write('\n    pace_meta.weight.linear.x = %f' %          float(pace_toml['step'][index]['acc_des'][0]))
        sequence_py.write('\n    pace_meta.weight.linear.y = %f' %          float(pace_toml['step'][index]['acc_des'][1]))
        sequence_py.write('\n    pace_meta.weight.linear.z = %f' %          float(pace_toml['step'][index]['acc_des'][2]))
        sequence_py.write('\n    pace_meta.weight.angular.x = %f' %         float(pace_toml['step'][index]['acc_des'][3]))
        sequence_py.write('\n    pace_meta.weight.angular.y = %f' %         float(pace_toml['step'][index]['acc_des'][4]))
        sequence_py.write('\n    pace_meta.weight.angular.z = %f' %         float(pace_toml['step'][index]['acc_des'][5]))
        sequence_py.write('\n    pace_meta.left_forefoot.x = %f' %          float(pace_toml['step'][index]['foot_pose'][2]))
        sequence_py.write('\n    pace_meta.left_forefoot.y = %f' %          float(pace_toml['step'][index]['foot_pose'][3]))
        sequence_py.write('\n    pace_meta.left_forefoot.w = %f' %          float(left_forefoot_height))
        sequence_py.write('\n    pace_meta.right_forefoot.x = %f' %         float(pace_toml['step'][index]['foot_pose'][0]))
        sequence_py.write('\n    pace_meta.right_forefoot.y = %f' %         float(pace_toml['step'][index]['foot_pose'][1]))
        sequence_py.write('\n    pace_meta.right_forefoot.w = %f' %         float(right_forefoot_height))
        sequence_py.write('\n    pace_meta.left_hindfoot.x = %f' %          float(pace_toml['step'][index]['ctrl_point'][0]))
        sequence_py.write('\n    pace_meta.left_hindfoot.y = %f' %          float(pace_toml['step'][index]['ctrl_point'][1]))
        sequence_py.write('\n    pace_meta.left_hindfoot.w = %f' %          float(left_hindfoot_height))
        sequence_py.write('\n    pace_meta.right_hindfoot.x = %f' %         float(pace_toml['step'][index]['foot_pose'][4]))
        sequence_py.write('\n    pace_meta.right_hindfoot.y = %f' %         float(pace_toml['step'][index]['foot_pose'][5]))
        sequence_py.write('\n    pace_meta.right_hindfoot.w = %f' %         float(right_hindfoot_height))
        sequence_py.write('\n    pace_meta.friction_coefficient = %f' %     float(pace_toml['step'][index]['ctrl_point'][2]))
        sequence_py.write('\n    pace_meta.landing_gain = %f' %             float(pace_toml['step'][index]['contact'] / 1e1))
        sequence_py.write('\n    pace_meta.use_mpc_track = %d' %            int(pace_toml['step'][index]['value']))
        sequence_py.write('\n    pace_meta.duration = %ld' %                int(pace_toml['step'][index]['duration']))
        sequence_py.write('\n    sequ.pace_list.push_back(pace_meta)\n')

    sequence_py.write('\n    return cyberdog_motion.run_sequence(sequ)\n')
    sequence_py.close()
    return True


#
# 舞蹈
# 参数:
# - cyberdog_motion_id: cyberdog_motion_id
# - type: 类型
# - args: 参数
#
def dance_args(cyberdog_motion_id, type, args):
    ret = MotionSequenceServiceResponse()
    ret.state.code = StateCode.fail
    ret.state.describe = "Ask dance_args(cyberdog_motion_id={}, type={}, args={})".format(cyberdog_motion_id, type, args)
    print(ret.state.describe)
    make = False
    if type == 'moonwalk':
        if len(args) == 4:
            x_velocity = args[0]
            y_velocity = args[1]
            stride = args[2]
            number = args[3]
            make = moonwalk(x_velocity, y_velocity, stride, number)
    elif type == 'push_up':
        if len(args) == 2:
            frequency = args[0]
            number = args[1]
            make = push_up(frequency, number)
    else:
        ret.state.describe = '\n - The current choreography type is invalid.'

    if make:
        importlib.reload(dancer)
        ret = dancer.show(cyberdog_motion_id)
    else:
        ret.state.describe = '\n - The current choreographer failed.'

    return ret


#
# 舞蹈
# 参数:
# - cyberdog_motion_id: cyberdog_motion_id
# - kwargs: 参数
#
def dance_kwargs(cyberdog_motion_id, kwargs):
    ret = MotionSequenceServiceResponse()
    ret.state.code = StateCode.fail
    ret.state.describe = "Ask dance_args(cyberdog_motion_id={}, kwargs={})".format(cyberdog_motion_id, kwargs)
    print(ret.state.describe)
    make = False
    type = kwargs.get('type')
    if type == 'moonwalk':
        make = moonwalk(kwargs.get('x_velocity'), kwargs.get('y_velocity'), kwargs.get('stride'), kwargs.get('number'))
    elif type == 'push_up':
        make = push_up(kwargs.get('frequency'), kwargs.get('number'))
    else:
        ret.state.describe = '\n - The current choreography type is invalid.'

    if make:
        importlib.reload(dancer)
        ret = dancer.show(cyberdog_motion_id)
    else:
        ret.state.describe = '\n - The current choreographer failed.'

    return ret
