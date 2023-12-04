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
import subprocess
import threading
import time
from time import sleep

from mi.cyberdog_bringup.manual import get_namespace
from protocol.msg import BluetoothStatus
from protocol.msg import NotifyToApp
from protocol.msg import WifiInfo
from protocol.srv import BmsCmd
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
# from sensor_msgs.msg import BatteryState, Joy
# from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Trigger

from . import bt_core


class BluetoothNode(Node):

    def __init__(self, node_name: str):
        Node.__init__(self, node_name)
        self.__logger = self.get_logger()  # ROS2 logger
        self.__logger.info('[BluetoothCore]: BluetoothNode init')
        self.dog_ssif = ''
        self.dog_ip = ''
        self.__multithread_callback_group = ReentrantCallbackGroup()
        self.__singlethread_callback_group = MutuallyExclusiveCallbackGroup()
        # 创建ROS服务用于打开和关闭蓝牙广播
        self.__openAdvertisingService = self.create_service(
                                BmsCmd, 'ctrl_advertising',
                                self.openAdvertisingCallback,
                                callback_group=self.__singlethread_callback_group)
        self.__openGattService = self.create_service(
                                BmsCmd, 'ctrl_gatt',
                                self.openGattCallback,
                                callback_group=self.__singlethread_callback_group)
        self.__disconnect_app_bt_service = self.create_service(
                                Trigger, 'disconnect_app_bt', self.__disconnect_app_bt_callback,
                                callback_group=self.__singlethread_callback_group)
        # 创建ROS话题用于发布蓝牙广播状态
        self.__bluetoothStatePublisher = self.create_publisher(
                                BluetoothStatus, 'bluetooth_status', 10)
        self.__notify_subscriber = self.create_subscription(
                                NotifyToApp, 'notify_to_app', self.__notify_to_app_callback, 1,
                                callback_group=self.__multithread_callback_group)
        self.__bt_core_lock = threading.Lock()
        self.blue_core = bt_core.BluetoothCore(self.__logger)
        # 开启线程,启动蓝牙mainloop的循环
        self.bluetooth_mainloop_thread = threading.Thread(target=self.blue_core.main_bluetooth)
        self.bluetooth_mainloop_thread.start()
        self.__logger.info('[BluetoothCore]: bluetooth_mainloop_thread start')
        time.sleep(1)
        self.blue_core.RegADV(True)
        sleep(3)
        self.blue_core.RegATT(True)

        # 创建一个定时器，定时发布状态（广播、连接、wifi信息）
        self.__bluetooth_status_timer = self.create_timer(
                                2, self.__bluetooth_status_timer_callback,
                                callback_group=self.__singlethread_callback_group)

    def __disconnect_app_bt_callback(self, request, response):
        self.__logger.info('disconnect_app_bt_callback')
        response.success = True
        response.message = 'disconnect app bt'
        self.blue_core.disconnect_device(self.blue_core.mac_address_get)
        return response

    def __notify_to_app_callback(self, msg):
        self.__logger.info('notify_to_app_callback')
        print(f'ssid:{msg.ssid} ip:{msg.ip} code:{msg.code}')
        self.blue_core.doNotify(msg.ssid, msg.ip, msg.code, self.blue_core.dog_sn)

    def openAdvertisingCallback(self, request, response):
        with self.__bt_core_lock:
            self.__logger.info('openAdvertising  service Callback')
            response.success = True
            response.code = 0
            if request.electric_machine_command == 1:
                if self.blue_core.adv_status == 0:
                    self.__logger.info('openAdvertisingCallback: advertise open')
                    self.blue_core.RegADV(True)
                    time.sleep(1)
                    if self.blue_core.adv_status == 0:
                        response.success = False
                        response.code = -1
                        pass
                else:
                    self.__logger.info('openAdvertisingCallback: is advertising')
            if request.electric_machine_command == 2:
                if self.blue_core.adv_status == 1:
                    self.__logger.info('penAdvertisingCallback: advertise close')
                    self.blue_core.RegADV(False)
                    time.sleep(1)
                    if self.blue_core.adv_status == 1:
                        response.success = False
                        response.code = -1
                        pass
                else:
                    self.__logger.info('openAdvertisingCallback: is not advertising')
            return response

    def openGattCallback(self, request, response):
        self.__logger.info('openGattCallback  service Callback')
        response.success = True
        response.code = 0
        if request.electric_machine_command == 1:
            self.__logger.info('openGattCallback: openGatt')
            self.blue_core.RegATT(True)
        elif request.electric_machine_command == 2:
            self.__logger.info('openGattCallback: closeGatt')
            self.blue_core.RegATT(False)
        pass

    def __bluetooth_status_timer_callback(self):
        connect_status, adv_status = self.blue_core.bluetooth_status()
        msg = BluetoothStatus()
        msg.connectable = connect_status
        msg.advtiseable = adv_status
        self.__bluetoothStatePublisher.publish(msg)
        pass


class PublisherPhoneIP(Node):

    def __init__(self,):
        now_namespace = get_namespace()
        super().__init__('phonepublisher', namespace=now_namespace)
        self.publisher_ = self.create_publisher(WifiInfo, 'app_send_network_data', 1)

    def publisher(self, msg):
        self.publisher_.publish(msg)
