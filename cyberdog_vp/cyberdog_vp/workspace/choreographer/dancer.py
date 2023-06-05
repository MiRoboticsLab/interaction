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

import _ctypes
from mi.cyberdog_vp.abilityset import MotionSequence
from mi.cyberdog_vp.abilityset import MotionSequenceGait
from mi.cyberdog_vp.abilityset import MotionSequencePace

def show(cyberdog_motion_id):
    """ Describe: Responsible for caching arbitrary choreography. """
    cyberdog_motion = _ctypes.PyObj_FromPtr(cyberdog_motion_id)
