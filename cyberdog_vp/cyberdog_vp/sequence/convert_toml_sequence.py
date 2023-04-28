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

import sys
import os
import time
import toml
import threading

def findAllFile(base):
    for root, ds, fs in os.walk(base):
        for f in fs:
            yield f

def main():
    base='./'
    num,log_cnt = 0, 0
    filelist=[]
    print(' ● 当前程序功能为：将运控配置文件转为可视化编程的序列文件:')
    print(' ● 当前路径下可选文件列表:')
    for file in findAllFile(base):
        if file.endswith('.toml'):
            filelist.append(file)
    filelist.sort()
    num = 0
    for file in filelist:
        print('   ☆ ['+str(num)+'] : '+str(filelist[num]))
        num=num+1

    sequence_py = open('./test_sequence.py', 'w+')
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
    sequence_py.write('\nimport os')
    sequence_py.write('\nimport sys')
    sequence_py.write('\nimport time')
    sequence_py.write('\nfrom mi.cyberdog_bringup.manual import get_namespace')
    sequence_py.write('\nfrom mi.cyberdog_vp.abilityset import MotionSequence')
    sequence_py.write('\nfrom mi.cyberdog_vp.abilityset import MotionSequenceGait')
    sequence_py.write('\nfrom mi.cyberdog_vp.abilityset import MotionSequencePace')
    sequence_py.write('\nfrom mi.cyberdog_vp.abilityset import Cyberdog')
    sequence_py.write('\ncyberdog = Cyberdog(\'mi_test_sequence\', get_namespace(), True, \'\')')
    sequence_py.write('\ntime.sleep(2)')
    sequence_py.write('\ncyberdog.set_log()')
    sequence_py.write('\ncyberdog.task.start()')
    sequence_py.write('\ncyberdog.audio.play(\'测试序列任务开始执行\',55)')
    sequence_py.write('\nsequ = MotionSequence()')
    sequence_py.write('\nsequ.name = \'test_sequ\'')
    sequence_py.write('\nsequ.describe = \'测试序列\'\n')

    print(' ★ 请输入步态配置文件编号：', end='')
    numInput=int(input())
    gait_file = os.path.join(base,filelist[numInput])
    sections = toml.load(gait_file)
    sequence_py.write('\n# 原始步态配置文件为：%s.\n' % gait_file)
    sequence_py.write('\ngait_meta = MotionSequenceGait()')
    for section in sections['section']:
        sequence_py.write('\ngait_meta.right_forefoot = %d' % bool(section['contact'][0]))
        sequence_py.write('\ngait_meta.left_forefoot = %d' % bool(section['contact'][1]))
        sequence_py.write('\ngait_meta.right_hindfoot = %d' % bool(section['contact'][2]))
        sequence_py.write('\ngait_meta.left_hindfoot = %d' % bool(section['contact'][3]))
        sequence_py.write('\ngait_meta.duration = %d' % int(section['duration']))
        sequence_py.write('\nsequ.gait_list.push_back(gait_meta)\n')

    print(' ★ 请输入步伐配置文件编号：', end='')
    numInput=int(input())
    pace_file = os.path.join(base,filelist[numInput])
    steps = toml.load(pace_file)
    sequence_py.write('\n# 原始步伐配置文件为：%s.\n' % pace_file)
    sequence_py.write('\npace_meta = MotionSequencePace()')
    for step in steps['step']:
        right_forefoot_height = float(format(step['step_height'][0] / 1e6, '.3f'))
        left_forefoot_height = float(format((step['step_height'][0] / 1e6 - right_forefoot_height) * 1e3, '.3f'))
        right_hindfoot_height = float(format(step['step_height'][1] / 1e6, '.3f'))
        left_hindfoot_height = float(format((step['step_height'][1] / 1e6 - right_hindfoot_height) * 1e3, '.3f'))
        sequence_py.write('\npace_meta.twist.linear.x = %f' %           (step['vel_des'][0]))
        sequence_py.write('\npace_meta.twist.linear.y = %f' %           (step['vel_des'][1]))
        sequence_py.write('\npace_meta.twist.linear.z = %f' %           (step['vel_des'][2]))
        sequence_py.write('\npace_meta.centroid.position.x = %f' %      (step['pos_des'][0]))
        sequence_py.write('\npace_meta.centroid.position.y = %f' %      (step['pos_des'][1]))
        sequence_py.write('\npace_meta.centroid.position.z = %f' %      (step['pos_des'][2]))
        sequence_py.write('\npace_meta.centroid.orientation.x = %f' %   (step['rpy_des'][0]))
        sequence_py.write('\npace_meta.centroid.orientation.y = %f' %   (step['rpy_des'][1]))
        sequence_py.write('\npace_meta.centroid.orientation.z = %f' %   (step['rpy_des'][2]))
        sequence_py.write('\npace_meta.weight.linear.x = %f' %          (step['acc_des'][0]))
        sequence_py.write('\npace_meta.weight.linear.y = %f' %          (step['acc_des'][1]))
        sequence_py.write('\npace_meta.weight.linear.z = %f' %          (step['acc_des'][2]))
        sequence_py.write('\npace_meta.weight.angular.x = %f' %         (step['acc_des'][3]))
        sequence_py.write('\npace_meta.weight.angular.y = %f' %         (step['acc_des'][4]))
        sequence_py.write('\npace_meta.weight.angular.z = %f' %         (step['acc_des'][5]))
        sequence_py.write('\npace_meta.left_forefoot.x = %f' %          (step['foot_pose'][2]))
        sequence_py.write('\npace_meta.left_forefoot.y = %f' %          (step['foot_pose'][3]))
        sequence_py.write('\npace_meta.left_forefoot.w = %f' %          (left_forefoot_height))
        sequence_py.write('\npace_meta.right_forefoot.x = %f' %         (step['foot_pose'][0]))
        sequence_py.write('\npace_meta.right_forefoot.y = %f' %         (step['foot_pose'][1]))
        sequence_py.write('\npace_meta.right_forefoot.w = %f' %         (right_forefoot_height))
        sequence_py.write('\npace_meta.left_hindfoot.x = %f' %          (step['ctrl_point'][0]))
        sequence_py.write('\npace_meta.left_hindfoot.y = %f' %          (step['ctrl_point'][1]))
        sequence_py.write('\npace_meta.left_hindfoot.w = %f' %          (left_hindfoot_height))
        sequence_py.write('\npace_meta.right_hindfoot.x = %f' %         (step['foot_pose'][4]))
        sequence_py.write('\npace_meta.right_hindfoot.y = %f' %         (step['foot_pose'][5]))
        sequence_py.write('\npace_meta.right_hindfoot.w = %f' %         (right_hindfoot_height))
        sequence_py.write('\npace_meta.friction_coefficient = %f' %     (step['ctrl_point'][2]))
        sequence_py.write('\npace_meta.landing_gain = %f' %             (step['contact'] / 1e1))
        sequence_py.write('\npace_meta.use_mpc_track = %d' %            (step['value']))
        sequence_py.write('\npace_meta.duration = %ld' %                (step['duration']))
        sequence_py.write('\nsequ.pace_list.push_back(pace_meta)\n')

    sequence_py.write('\ncyberdog.motion.run_sequence(sequ)')
    sequence_py.write('\ncyberdog.audio.play(\'测试序列任务执行结束\',55)')
    sequence_py.write('\ncyberdog.task.stop()\n')
    sequence_py.close()
    print(' ● 原始文件:')
    print('   ○ 步态文件: %s' % gait_file)
    print('   ◎ 步伐文件: %s' % pace_file)
    print(' ● 产出文件: %s ' %  sequence_py.name)
if __name__ == '__main__':
    main()
