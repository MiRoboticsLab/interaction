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

from ament_index_python.packages import get_package_share_directory

#
# 获取 工作空间
#
def get_workspace():
    workspace = os.path.join(get_package_share_directory('cyberdog_vp'), 'workspace')
    module_toml = os.path.join(
        get_package_share_directory('cyberdog_vp'),
        'config',
        'abilityset.toml',
    )
    toml_dictionary = toml.load(module_toml)
    workspace_params = toml_dictionary['vp']['init']['environment']['workspace']
    if len(workspace_params.strip('/'))>0:
        workspace = workspace_params + '/workspace'
    return workspace


#
# 获取 argv
#
def get_argv():
    task_id = ''
    task_parameters = ''
    user_env_var = 'USERNAME' if platform.system() == 'Windows' else 'USER'
    user = os.environ[user_env_var]
    argv = sys.argv[0:]
    sys.argv = sys.argv[0:1]
    print('[', user, '] get_task(', str(argv), ')...')
    for now_arg in argv:
        if re.match(r'^(cyberdog_vp_task:=)+.*$', now_arg):
            task_id = re.sub(r'^(cyberdog_vp_task:=)+', '', now_arg)
        elif now_arg == '--ros-args':
            task_parameters += now_arg
        elif task_parameters != '':
            task_parameters += ' ' + now_arg
        else:
            continue
    return (task_id, task_parameters)


#
# 导入 modules 模块
#
def import_modules(_task_body):
    dependent_modules = ["sys.path.append(os.path.join('" + get_workspace() + "', 'module', 'src'))"]
    module_toml = os.path.join(get_workspace(), 'module', 'module.toml')
    module = 'module'
    mode = 'mode'
    condition = 'condition'
    file = 'file'
    toml_dictionary = toml.load(module_toml)
    for module_id in toml_dictionary[module]:
        mode_type = toml_dictionary[module][module_id][mode]
        if (mode_type == 'common') or (mode_type == 'sequence'):
            # for key,value in toml_dictionary[module][module_id].items():
            #     print(key,':',value)
            interface = toml_dictionary[module][module_id][condition].split('(')[0] + '('
            dependent_interface = False
            if _task_body == 'all':
                dependent_interface = True
            elif interface in _task_body:
                dependent_interface = True
            if dependent_interface:
                import_interface = (
                    'from '
                    + toml_dictionary[module][module_id][file].split('/')[-1].split('.')[0]
                    + ' import '
                    + toml_dictionary[module][module_id][condition].split('(')[0]
                )
                dependent_modules.append(import_interface)
    return dependent_modules


#
# 导入 time 模块
#
def import_time(_task_body):
    dependent_modules = []
    modules_label = [
        'time.time(',
        'time.asctime('
        'time.localtime(',
        'time.sleep(',
        'time.strftime(',
        'time.pthread_getcpuclockid(',
        'time.clock_getres(',
        'time.clock_gettime(',
        'time.clock_gettime_ns(',
        'time.clock_settime(',
        'time.clock_settime_ns(',
        'time.ctime(',
        'time.get_clock_info(',
        'time.mktime(',
        'time.monotonic(',
        'time.monotonic_ns(',
        'time.perf_counter(',
        'time.perf_counter_ns(',
        'time.process_time(',
        'time.process_time_ns(',
        'time.strptime(',
        'time.struct_time(',
        'time.time_ns(',
        'time.thread_time(',
        'time.thread_time_ns(',
        'time.tzset(',
        'time.CLOCK_BOOTTIME',
        'time.CLOCK_HIGHRES',
        'time.CLOCK_MONOTONIC',
        'time.CLOCK_MONOTONIC_RAW',
        'time.CLOCK_PROCESS_CPUTIME_ID',
        'time.CLOCK_PROF',
        'time.CLOCK_TAI',
        'time.CLOCK_THREAD_CPUTIME_ID',
        'time.CLOCK_UPTIME',
        'time.CLOCK_UPTIME_RAW',
        'time.CLOCK_REALTIME',
        'time.altzone',
        'time.daylight',
        'time.timezone',
        'time.tzname',
        ]
    for label in modules_label:
        if label in _task_body:
            dependent_modules.append('import time')
            break
    return dependent_modules


#
# 导入 cyberdog_vp_abilityset 模块
#
def import_cyberdog_vp_abilityset(_task_body):
    dependent_modules = []
    modules_label = {
        'StateCode.': 'from mi.cyberdog_vp.abilityset import StateCode',
        'LedConstraint.': 'from mi.cyberdog_vp.abilityset import LedConstraint',
        'MotionSequence(': 'from mi.cyberdog_vp.abilityset import MotionSequence',
        'MotionSequenceGait(': 'from mi.cyberdog_vp.abilityset import MotionSequenceGait',
        'MotionSequencePace(': 'from mi.cyberdog_vp.abilityset import MotionSequencePace'
    }
    for key, value in modules_label.items():
        if key in _task_body:
            dependent_modules.append(value)
    return dependent_modules


#
# 获取 任务 头部
#
def get_task_header(_ros, _task_id, _task_body):
    # 获取 头部
    header = [
        '',
        'import os',
        'import sys',
        'import time',
        'from mi.cyberdog_bringup.manual import get_namespace',
        'from mi.cyberdog_vp.utils import get_argv',
        'from mi.cyberdog_vp.abilityset import Cyberdog',
    ]
    header += import_modules(_task_body)
    header += import_cyberdog_vp_abilityset(_task_body)
    header.append('')
    header.append('now_task_id, now_task_parameters = get_argv()')
    header.append("task_id = now_task_id if len(now_task_id) != 0 else '" + _task_id + "'")
    header.append("task_parameters = now_task_parameters if len(now_task_parameters) != 0 else ''")
    header.append('')
    header.append('print(time.strftime("任务开始时间为：%Y年%m月%d日 %H点%M分%S秒", time.localtime()))')
    header.append("cyberdog = Cyberdog(task_id, get_namespace(), " + _ros + ", task_parameters)")
    header.append('cyberdog.set_log(False)')
    # for msg in header:
    #     print(msg)
    return header


#
# 获取 模块 头部
#
def get_module_header(_interface, _describe, _body):
    header = [
        'import os',
        'import sys',
        'import __main__',
    ]
    header += import_time(_body)
    header += import_modules(_body)
    header += import_cyberdog_vp_abilityset(_body)
    message = 'def ' + _interface + ':'
    header.append(message)
    message = '    """ Describe: ' + _describe + ''
    header.append(message)
    header.append('    """')
    header.append('    cyberdog = __main__.cyberdog')
    return header


#
# 获取 终端 头部
#
def get_terminal_header(_ros, _task_id, _task_body):
    task_id = _task_id
    task_parameters = ''
    # 获取 头部
    header = [
        'import os',
        'import sys',
        'import time',
        'from IPython.lib.demo import Demo',
        'from mi.cyberdog_bringup.manual import get_namespace',
        'from mi.cyberdog_vp.abilityset import StateCode',
        'from mi.cyberdog_vp.abilityset import LedConstraint',
        'from mi.cyberdog_vp.abilityset import MotionSequence',
        'from mi.cyberdog_vp.abilityset import MotionSequenceGait',
        'from mi.cyberdog_vp.abilityset import MotionSequencePace',
        'from mi.cyberdog_vp.abilityset import Cyberdog',
        'from mi.cyberdog_vp.terminal import Visual',
    ]
    header += import_modules(_task_body)
    header.append("print('运行起始时间为：', time.strftime('%Y-%m-%d %H:%M:%S', time.localtime()))")
    message = (
        "cyberdog = Cyberdog('terminal_"
        + task_id
        + "', get_namespace(), "
        + _ros
        + ", '"
        + task_parameters
        + "')"
    )
    header.append(message)
    header.append('cyberdog.task.start()')
    _ros = 'False'
    message = (
        "visual = Visual('visual_" + task_id
        + "', get_namespace(), " + _ros + ", '" + task_parameters + "')"
    )
    header.append(message)
    # for msg in header:
    #     print(msg)
    return header


#
# 生成 衍生 demo 文件(自动'_automatic.demo.py'/手动'_manual.demo.py')
#
def generate_derivative_demo_file(_absolute_path_file, _is_automatic):
    if _absolute_path_file.split('.')[-1] not in ['py']:
        return False
    file_type = '_manual.demo.py'
    if _is_automatic:
        file_type = '_automatic.demo.py'
    target_file_path = _absolute_path_file.replace('.py', file_type)
    print('Derivative file is being generated:', target_file_path)
    shapus = '####################'
    # demo_auto_all = '\n# <demo> auto_all\n' # 仅用于第一个块中，使整个演示完全自动化
    demo_auto = '\n# <demo> auto\n'  # 使块自动执行，但仍在打印
    demo_silent = '\n# <demo> silent\n\n'  # 使块静默执行，且不会打印
    demo_stop = '# <demo> stop\n'  # 停止执行文件并返回的点
    target_file = open(target_file_path, mode='w+', encoding='utf-8')
    file_nam = target_file_path.split('/')[-1]
    file_beg = shapus + ' BEGIN DEMO <' + file_nam + '> ' + shapus + '\n'
    file_dec = "'''Interactive presentation script for the current task.'''\n"
    file_end = '\n\n' + shapus + '  END  DEMO <' + file_nam + '> ' + shapus + '\n'
    target_file.write(file_beg)
    target_file.write(file_dec)
    target_file.write(demo_silent)
    with open(_absolute_path_file, mode='r', encoding='utf-8') as f:
        line = f.readline()
        while line:
            if re.match(r'^(\s)*(cyberdog.task.block()+.*)$', line):
                target_file.write(demo_stop)
                if _is_automatic:
                    target_file.write(demo_auto)
            if re.match(r'^(\s)*(cyberdog = cyberdog_vp_abilityset_py.Cyberdog()+.*)$', line):
                target_file.write(line.replace('True', 'False'))
            else:
                target_file.write(line)
            line = f.readline()
    target_file.write(file_end)
    target_file.close()
    return True


#
# 生成 衍生 单步调试 文件
#
def generate_derivative_single_step_file(_absolute_path_file):
    if _absolute_path_file.split('.')[-1] not in ['py']:
        return False
    target_file_path = _absolute_path_file.replace('.py', '_single_step.py')
    print('Derivative file is being generated:', target_file_path)
    target_file = open(target_file_path, mode='w+', encoding='utf-8')
    with open(_absolute_path_file, mode='r', encoding='utf-8') as f:
        line = f.readline()
        while line:
            if re.match(r'^(\s)*(cyberdog.task.block()+.*)$', line):
                target_file.write(line.replace('.block(', '.breakpoint_block('))
            else:
                target_file.write(line)
            line = f.readline()
    target_file.close()
    return True


#
# 生成 衍生 文件
#
def generate_derivative_file(_absolute_path_file):
    print(
        'A derivative demo file of the object file is being generated:',
        _absolute_path_file,
    )
    # if generate_derivative_demo_file(
    #     _absolute_path_file, True
    # ) and generate_derivative_demo_file(_absolute_path_file, False):
    #     return True
    if generate_derivative_single_step_file(_absolute_path_file):
        return True
    return False
