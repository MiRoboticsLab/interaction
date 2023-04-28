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

import base64

def findAllFile(base):
    for root, ds, fs in os.walk(base):
        for f in fs:
            yield f

def main():
    base='./'
    num,log_cnt = 0, 0
    filelist=[]
    print(' ● 当前程序功能为：将图片转为 base64 格式的字符串:')
    print(' ● 当前路径下可用文件列表:')
    for file in findAllFile(base):
        if file.endswith('.png') or file.endswith('.jpg') or file.endswith('.jpeg'):
            filelist.append(file)
    filelist.sort()
    num = 0
    for file in filelist:
        print('   ☆ ['+str(num)+'] : '+str(filelist[num]))
        num=num+1

    sequence_py = open('./base64_image.md', 'w+')
    sequence_py.write('# base64 image')
    sequence_py.write('\n---')
    sequence_py.write('\n\n```')
    sequence_py.write('\nCopyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.')
    sequence_py.write('\n')
    sequence_py.write('\nLicensed under the Apache License, Version 2.0 (the \'License\');')
    sequence_py.write('\nyou may not use this file except in compliance with the License.')
    sequence_py.write('\nYou may obtain a copy of the License at')
    sequence_py.write('\n')
    sequence_py.write('\n    http://www.apache.org/licenses/LICENSE-2.0')
    sequence_py.write('\n')
    sequence_py.write('\nUnless required by applicable law or agreed to in writing, software')
    sequence_py.write('\ndistributed under the License is distributed on an \'AS IS\' BASIS,')
    sequence_py.write('\nWITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.')
    sequence_py.write('\nSee the License for the specific language governing permissions and')
    sequence_py.write('\nlimitations under the License.\n')
    sequence_py.write('```')

    num = 0
    for file in filelist:
        image=open(str(filelist[num]),'rb')  #二进制方式打开图文件
        image_base64=base64.b64encode(image.read())  #读取文件内容，转换为base64编码
        image.close()
        sequence_py.write('\n## %s.\n' % str(filelist[num]))
        sequence_py.write('\n![avatar][%s.base64]\n' % str(filelist[num]))
        sequence_py.write('\n[%s.base64]:data:image/png;base64,%s\n' % (str(filelist[num]), str(image_base64, 'utf-8')))
        # sequence_py.write('\n![avatar](data:image/png;base64,%s)' % str(image_base64, 'utf-8'))
        num=num+1

    sequence_py.close()
    print(' ● 产出文件: %s ' %  sequence_py.name)
if __name__ == '__main__':
    main()
