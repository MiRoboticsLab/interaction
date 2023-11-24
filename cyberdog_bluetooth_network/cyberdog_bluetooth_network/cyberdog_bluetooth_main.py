#!/usr/bin/python3
#
# Copyright (c) 2023 Xiaomi Corporation
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

import rclpy
from rclpy.executors import MultiThreadedExecutor

# from . import bt_core
from . import bluetooth_node


def main(args=None):
    rclpy.init(args=args)
    bt_node = bluetooth_node.BluetoothNode('cyberdog_bluetooth_network')
    print('[BluetoothCore]: Hi from cyberdog_bluetooth.')
    executor = MultiThreadedExecutor()
    executor.add_node(bt_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        bt_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
