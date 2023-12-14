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

import json
import re
import subprocess
import threading
import time

import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool, Int8, String
from protocol.msg import WifiInfo

from . import bluetooth_node

try:
    from gi.repository import GObject
except ImportError:
    import gobject as GObject

ad_manager = None
service_manager = None
robotAdv = None
robotApp = None
mainloop = None


notifyChar = None
# gnotifystr = {'ssid': None, 'bssid': None, 'ip': None, 'status': 0}

# 反馈给APP的错误码
code = 0
g_current_mac = ''
connected = 0
g_connect_status = 0
adv_status = 0

app_send_mac = ''
app_send_ssid = ''
app_send_ip = ''
app_send_pwd = ''
app_send_type = ''
mac_address_get = ''

# 获取wifi信息的状态反馈
WIFI_INFO_GET_SUCCESS = 1000  # 获取成功
WIFI_INFO_SSID_ERROR = 1001  # 获取ssid失败
WIFI_INFO_PWD_ERROR = 1002  # 获取pwd失败
WIFI_INFO_IP_ERROR = 1003  # 获取ip失败
WIFI_INFO_MAC_ERROR = 1004  # 获取mac失败

# 网络状态反馈
NETWORK_CONNECT_SUCCESS = 2000  # 连接成功，且能访问外网
NETWORK_CONNECT_FAILED = 2001  # 连接失败，连不上这个网络
NETWORK_NOT_CONNECT_INTERNET = 2002  # 连接失败，连上了这个网络，但是没有连上互联网
NETWORK_DISCONNECT = 2003  # 断开连接


NETWORK_PATH = '/etc/NetworkManager/system-connections/'

BLUEZ_SERVICE_NAME = 'org.bluez'
ADAPTER_INTERFACE = BLUEZ_SERVICE_NAME + '.Adapter1'
DEVICE_INTERFACE = BLUEZ_SERVICE_NAME + '.Device1'
GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
DBUS_OM_IFACE = 'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE = 'org.freedesktop.DBus.Properties'
LE_ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'
LE_ADVERTISEMENT_IFACE = 'org.bluez.LEAdvertisement1'
GATT_SERVICE_IFACE = 'org.bluez.GattService1'
GATT_CHRC_IFACE = 'org.bluez.GattCharacteristic1'
GATT_DESC_IFACE = 'org.bluez.GattDescriptor1'


class InvalidArgsException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.freedesktop.DBus.Error.InvalidArgs'


class NotSupportedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.NotSupported'


class NotPermittedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.NotPermitted'


class InvalidValueLengthException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.InvalidValueLength'


class FailedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.Failed'


class Advertisement(dbus.service.Object):
    PATH_BASE = '/org/bluez/example/advertisement'

    def __init__(self, bus, index, advertising_type):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.ad_type = advertising_type
        self.service_uuids = None
        self.manufacturer_data = None
        self.solicit_uuids = None
        self.service_data = None
        self.local_name = '铁蛋'
        self.include_tx_power = None
        self.discoverable = True
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        properties = {}
        properties['Type'] = self.ad_type
        if self.service_uuids is not None:
            properties['ServiceUUIDs'] = dbus.Array(self.service_uuids,
                                                    signature='s')
        if self.solicit_uuids is not None:
            properties['SolicitUUIDs'] = dbus.Array(self.solicit_uuids,
                                                    signature='s')
        if self.manufacturer_data is not None:
            properties['ManufacturerData'] = dbus.Dictionary(
                self.manufacturer_data, signature='qv')
        if self.service_data is not None:
            properties['ServiceData'] = dbus.Dictionary(self.service_data,
                                                        signature='sv')
        if self.local_name is not None:
            properties['LocalName'] = dbus.String(self.local_name)
        if self.include_tx_power is not None:
            properties['IncludeTxPower'] = dbus.Boolean(self.include_tx_power)
        return {LE_ADVERTISEMENT_IFACE: properties}

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service_uuid(self, uuid):
        if not self.service_uuids:
            self.service_uuids = []
        self.service_uuids.append(uuid)

    def add_solicit_uuid(self, uuid):
        if not self.solicit_uuids:
            self.solicit_uuids = []
        self.solicit_uuids.append(uuid)

    def add_manufacturer_data(self, manuf_code, data):
        if not self.manufacturer_data:
            self.manufacturer_data = dbus.Dictionary({}, signature='qv')
        self.manufacturer_data[manuf_code] = dbus.Array(data, signature='y')

    def add_service_data(self, uuid, data):
        if not self.service_data:
            self.service_data = dbus.Dictionary({}, signature='sv')
        self.service_data[uuid] = dbus.Array(data, signature='y')

    def add_local_name(self, name):
        if not self.local_name:
            self.local_name = ''
        self.local_name = dbus.String(name)

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        print('GetAll')
        if interface != LE_ADVERTISEMENT_IFACE:
            raise InvalidArgsException()
        print('returning props')
        return self.get_properties()[LE_ADVERTISEMENT_IFACE]

    @dbus.service.method(LE_ADVERTISEMENT_IFACE,
                         in_signature='',
                         out_signature='')
    def Release(self):
        print('%s: Released!' % self.path)


class RobotAdvertisement(Advertisement):

    def __init__(self, bus, index, g_bt_local_name):
        Advertisement.__init__(self, bus, index, 'peripheral')
        self.add_service_uuid('C420')  # 添加广播的服务,UUID
        self.add_manufacturer_data(0xffff, [0x00, 0x01, 0x02, 0x03, 0x04])
        self.add_service_data('9999', [0x00, 0x01, 0x02, 0x03, 0x04])
        self.add_local_name('铁蛋' + g_bt_local_name)  # 设置蓝牙广播名称
        self.include_tx_power = True


class RobotApplication(dbus.service.Object):
    # org.bluez.GattApplication1 interface implementation

    def __init__(self, bus, bluetoothCore):
        self.path = '/'
        self.services = []
        dbus.service.Object.__init__(self, bus,
                                     self.path)
        print('add service: ' + str(bus))
        self.add_service(RobotService(bus, 0, bluetoothCore))  # 将服务添加到服务列表中

    def get_path(self):
        print('get_path: ' + str(self))
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        print('add_service: ' + str(service))
        self.services.append(service)

    # 用于获取应用程序及其包含的服务、特征和描述符的对象信息。
    @dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            chrcs = service.get_characteristics()
            for chrc in chrcs:
                response[chrc.get_path()] = chrc.get_properties()
                descs = chrc.get_descriptors()
                for desc in descs:
                    response[desc.get_path()] = desc.get_properties()

        return response


class Service(dbus.service.Object):
    # org.bluez.GattService1 interface implementation

    PATH_BASE = '/org/bluez/example/service'

    def __init__(self, bus, index, uuid, primary):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_SERVICE_IFACE: {
                'UUID': self.uuid,
                'Primary': self.primary,
                'Characteristics': dbus.Array(
                    self.get_characteristic_paths(),
                    signature='o')
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, characteristic):
        self.characteristics.append(characteristic)

    def get_characteristic_paths(self):
        result = []
        for chrc in self.characteristics:
            result.append(chrc.get_path())
        return result

    def get_characteristics(self):
        return self.characteristics

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_SERVICE_IFACE:
            raise InvalidArgsException()

        return self.get_properties()[GATT_SERVICE_IFACE]


class Characteristic(dbus.service.Object):
    # org.bluez.GattCharacteristic1 interface implementation

    def __init__(self, bus, index, uuid, flags, service):
        self.path = service.path + '/char' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = flags
        self.descriptors = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_CHRC_IFACE: {
                'Service': self.service.get_path(),
                'UUID': self.uuid,
                'Flags': self.flags,
                'Descriptors': dbus.Array(
                    self.get_descriptor_paths(),
                    signature='o')
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_descriptor(self, descriptor):
        self.descriptors.append(descriptor)

    def get_descriptor_paths(self):
        result = []
        for desc in self.descriptors:
            result.append(desc.get_path())
        return result

    def get_descriptors(self):
        return self.descriptors

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE:
            raise InvalidArgsException()

        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.method(GATT_CHRC_IFACE,
                         in_signature='a{sv}',
                         out_signature='ay')
    def ReadValue(self, options):
        global log
        print('Default ReadValue called, returning error')
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        global log
        print('Default WriteValue called, returning error')
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        global log
        print('Default StartNotify called, returning error')
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        global log
        print('Default StopNotify called, returning error')
        raise NotSupportedException()

    @dbus.service.signal(DBUS_PROP_IFACE,
                         signature='sa{sv}as')
    def PropertiesChanged(self, interface, changed, invalidated):
        pass


class Descriptor(dbus.service.Object):

    def __init__(self, bus, index, uuid, flags, characteristic):
        self.path = characteristic.path + '/desc' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.flags = flags
        self.chrc = characteristic
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
            GATT_DESC_IFACE: {
                'Characteristic': self.chrc.get_path(),
                'UUID': self.uuid,
                'Flags': self.flags,
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_DESC_IFACE:
            raise InvalidArgsException()

        return self.get_properties()[GATT_DESC_IFACE]

    @dbus.service.method(GATT_DESC_IFACE,
                         in_signature='a{sv}',
                         out_signature='ay')
    def ReadValue(self, options):
        global log
        print('Default ReadValue called, returning error')
        raise NotSupportedException()

    @dbus.service.method(GATT_DESC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        global log
        print('Default WriteValue called, returning error')
        raise NotSupportedException()


# RobotService for ATT
class RobotService(Service):
    ROBOT_SVC_UUID = '0000C420-0000-1000-8000-00805F9B34FB'

    def __init__(self, bus, index, bluetoothCore):
        Service.__init__(self, bus, index, self.ROBOT_SVC_UUID, True)
        self.add_characteristic(writeWifiParameterFromPhone(bus, 0, self, bluetoothCore))
        self.add_characteristic(notifyWifiStatusToPhone(bus, 1, self))


class writeWifiParameterFromPhone(Characteristic):
    WIFI_CHRC_UUID_WRITE = '0000C421-0000-1000-8000-00805F9B34FB'

    def __init__(self, bus, index, service, bluetoothCore):
        Characteristic.__init__(
            self, bus, index,
            self.WIFI_CHRC_UUID_WRITE,
            ['write-without-response'],
            service)
        self.pub_info = bluetooth_node.PublisherPhoneIP()
        self.logger = bluetoothCore.get_logger()
        self.bluetooth_core = bluetoothCore
        self.sn = bluetoothCore.dog_sn

    def WriteValue(self, value, options):
        global code, app_send_ssid, app_send_pwd, app_send_ip, app_send_mac, app_send_type
        try:
            format_value = bytes(value).decode('utf-8')
            self.logger.info('WriteValue format  all value :%s' % format_value)
            text = json.loads(format_value)
        except UnicodeDecodeError:
            self.logger.info('WriteValue UnicodeDecodeError')
        except json.JSONDecodeError:
            self.logger.info('WriteValue JSONDecodeError')

        ssid = text.get('ssid')
        pwd = text.get('pwd')
        ip = text.get('ip')
        mac = text.get('mac')
        type_receive = text.get('type')

        code = WIFI_INFO_GET_SUCCESS
        if ssid is not None:
            # self.logger.info('WriteValue ssid %s' % repr(text['ssid']))
            app_send_ssid = ssid
        else:
            self.logger.info('WriteValue ssid is none!!!')
            app_send_ssid = ''
            code = WIFI_INFO_SSID_ERROR

        if pwd is not None:
            # self.logger.info('WriteValue pwd %s' % repr(text['pwd']))
            app_send_pwd = pwd
        else:
            self.logger.info('WriteValue pwd is none!!!')
            app_send_pwd = ''
            # code = WIFI_INFO_PWD_ERROR

        if ip is not None:
            # self.logger.info('WriteValue ip %s' % repr(text['ip']))
            app_send_ip = ip
        else:
            self.logger.info('WriteValue ip is none!!!')
            app_send_ip = ''
            code = WIFI_INFO_IP_ERROR

        if mac is not None:
            # self.logger.info('WriteValue mac %s' % repr(text['mac']))
            app_send_mac = mac
        else:
            self.logger.info('WriteValue ip is none!!!')
            app_send_mac = ''
            code = WIFI_INFO_MAC_ERROR

        if type_receive is not None:
            # self.logger.info('WriteValue type %s' % repr(text['type']))
            app_send_type = type_receive
        else:
            self.logger.info('WriteValue ip is none!!!')
            app_send_type = ''
            code = 1005
        # 如果接收的数据都不为空，且type为wifi或者hotspot，就发送给app
        if(code == WIFI_INFO_GET_SUCCESS):
            if(app_send_type == 'wifi' or app_send_type == 'hotspot'):
                # 如果快连模块没有初始化，通过蓝牙返回6000错误码
                if self.bluetooth_core.get_connect_init_status is False:
                    self.logger.info('connector is not init')
                    self.doNotifyOnce('', '', 6000, self.sn)
                    return
                msg = WifiInfo()
                msg.ssid = app_send_ssid
                msg.ip = app_send_ip
                msg.pwd = app_send_pwd
                msg.mac = app_send_mac
                msg.type = app_send_type
                self.pub_info.publisher(msg)
        if(app_send_mac != '' and app_send_type != 'heartBeat' and
           (app_send_ip == '' or app_send_ssid == '' or app_send_pwd == '')):
            code = 3000
            self.doNotifyOnce(app_send_ssid, app_send_ip, code, self.sn)
        # 蓝牙心跳
        if(app_send_type == 'heartBeat'):
            # self.logger.info('send heartBeat')
            code = 5000
            self.doNotifyOnce(app_send_ssid, app_send_ip, code, self.sn)
        self.bluetooth_core.GetAppData(
            app_send_ssid, app_send_pwd, app_send_ip, app_send_mac, app_send_type)
        return

    def doNotifyOnce(self, ssid, ip, code, sn):
        gnotifystr = {
                'ssid': ssid,
                'ip': ip,
                'code': code,
                'sn': sn}
        self.logger.info('doNotifyOnce %s' % repr(gnotifystr))
        arraySend = convert_to_dbus_array(json.dumps(gnotifystr))
        # 发送Dbus通知
        notifyChar.PropertiesChanged(GATT_CHRC_IFACE, {'Value': arraySend}, [])


# notify wlan state
class notifyWifiStatusToPhone(Characteristic):
    WIFI_CHRC_UUID_NOTIFY = '0000C422-0000-1000-8000-00805F9B34FB'

    def __init__(self, bus, index, service):
        Characteristic.__init__(
            self, bus, index,
            self.WIFI_CHRC_UUID_NOTIFY,
            ['notify'],
            service)

        global notifyChar
        notifyChar = self

    # phone start notify
    def StartNotify(self):
        # RegADV(False)  # unregister adv and update led
        # notify once immediately when phone connected
        gnotifystr = {
            'ssid': 'strat notify',
            'ip': 'strat notify',
            'code': 4000,
            'sn': 'strat notify'}
        arraySend = convert_to_dbus_array(json.dumps(gnotifystr))
        # 发送Dbus通知
        notifyChar.PropertiesChanged(GATT_CHRC_IFACE, {'Value': arraySend}, [])

    # phone stop notify
    def StopNotify(self):
        pass
        # global bleStatus
        # if BLE_STATUS_DISCONNECTED != bleStatus:
        #     bleStatus = BLE_STATUS_DISCONNECTED
        #     print(' StopNotify: set bleStatus ')
        #     # RegATT(False)
        # else:
        #     print(' StopNotify: Current bleStatus is: ' + repr(bleStatus))


class TimeoutTimer:

    def __init__(self, _timeout):
        self.userTimeoutFunc = None
        self.userTimeoutFunc_args = None
        self.timeoutAlarm: bool = False
        self.timeoutHandle = threading.Timer(_timeout, self.timeoutFunc)

    def addUserTimeoutFunc(self, _user_timeout_func, *_args):
        self.userTimeoutFunc = _user_timeout_func
        self.userTimeoutFunc_args = _args

    def startTimer(self):
        self.timeoutHandle.start()

    def timeoutFunc(self):
        self.timeoutAlarm = True
        if self.userTimeoutFunc is not None:
            self.userTimeoutFunc(*self.userTimeoutFunc_args)

    def getAlarm(self):
        return self.timeoutAlarm

    def stopTimer(self):
        if self.timeoutHandle is not None:
            self.timeoutHandle.cancel()
        self.timeoutAlarm = False

    def resetTimer(self):
        if self.timeoutHandle and self.timeoutHandle.is_alive():
            self.timeoutHandle.cancel()


def convert_to_dbus_array(string):
    value = []
    for c in string:
        value.append(dbus.Byte(c.encode()))
    return value


class BluetoothCore:

    def __init__(self, logger):
        self.__logger = logger
        self.__logger.info('BluetoothCore: __init__')
        self.g_bt_local_name = '铁蛋'
        self.adv_status = 0
        self.g_current_mac = ''
        self.mac_address_get = ''
        self.receive_mac = ''
        self.receive_ssid = ''
        self.receive_pwd = ''
        self.receive_ip = ''
        self.dog_sn = ''
        self.connected = 0
        self.masters = []
        self.slavers = []
        self.get_connect_init_status = False
        command = 'factory-tool -f /usr/share/factory_cmd_config/system.xml -i SN'
        self.g_bt_local_name = self.runCommand(command)
        self.dog_sn = self.g_bt_local_name.rstrip('\n')
        self.g_bt_local_name = (self.g_bt_local_name[4] +
                                self.g_bt_local_name[5] +
                                self.g_bt_local_name[9] +
                                self.g_bt_local_name[10] +
                                self.g_bt_local_name[11])
        self.__logger.info('g_bt_local_name: %s' % self.g_bt_local_name)
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        self.bus = dbus.SystemBus()
        self.remote_om = dbus.Interface(self.bus.get_object(
                         BLUEZ_SERVICE_NAME, '/'), DBUS_OM_IFACE)
        self.adapter = self.find_adapter(self.bus)
        self.counter = 0
        self.__logger.info('start find Gattmanager1 interface')
        while not self.adapter and self.counter < 120:
            self.__logger.info('GattManager1 interface not found')
            self.__logger.info('sleep 500ms, find adapter again')
            time.sleep(0.5)
            self.adapter = self.find_adapter(self.bus)
            self.counter = self.counter + 1
        if not self.adapter:
            self.__logger.info('GattManager1 interface not found')
            return
        self.service_manager = dbus.Interface(
            self.bus.get_object(BLUEZ_SERVICE_NAME, self.adapter),
            GATT_MANAGER_IFACE)
        self.robotApp = RobotApplication(self.bus, self)
        self.adapter_props = dbus.Interface(self.bus.get_object(BLUEZ_SERVICE_NAME, self.adapter),
                                            'org.freedesktop.DBus.Properties')
        self.adapter_props.Set('org.bluez.Adapter1', 'Powered', dbus.Boolean(1))
        self.ad_manager = dbus.Interface(self.bus.get_object(BLUEZ_SERVICE_NAME, self.adapter),
                                         LE_ADVERTISING_MANAGER_IFACE)
        self.bus.add_signal_receiver(self.properties_changed,
                                     dbus_interface='org.freedesktop.DBus.Properties',
                                     signal_name='PropertiesChanged',
                                     path_keyword='path')

        self.bus.add_signal_receiver(self.interfaces_added,
                                     dbus_interface='org.freedesktop.DBus.ObjectManager',
                                     signal_name='InterfacesAdded')
        self.robotAdv = RobotAdvertisement(self.bus, 0, self.g_bt_local_name)
        self.connected_timer = None

    def find_adapter(self, bus):
        # remote_om = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, '/'),
        #                         DBUS_OM_IFACE)
        objects = self.remote_om.GetManagedObjects()

        for o, props in objects.items():
            if GATT_MANAGER_IFACE in props.keys():
                if LE_ADVERTISING_MANAGER_IFACE in props:
                    return o
        return None

    def runCommand(self, cmd: str):
        self.__logger.info(' run command:%s' % cmd)
        output = subprocess.Popen(
            cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        output.wait()  # wait for cmd done
        tmp = str(output.stdout.read(), encoding='utf-8')
        return tmp

    def extract_mac_address_from_path(self, path):
        parts = path.split('/')
        if len(parts) == 5:
            return parts[-1].replace('_', ':').upper()
        return None

    def parse_connections_output(self, output):
        masters_connect = re.findall(r'< LE (\S+) handle \d+ state \d+ lm MASTER', output)
        slaves_connect = re.findall(r'> LE (\S+) handle \d+ state \d+ lm SLAVE', output)
        return masters_connect, slaves_connect

    def disconnect_device(self, mac_address):
        script = f"""
        bluetoothctl << EOF
        disconnect {mac_address}
        exit
        EOF
        """
        self.__logger.info('disconnect_device: %s' % mac_address)
        self.runCommand(script)

    def check_connected(self):
        mac_address = '02:00:00:00:00:00'
        self.connected_timer.stopTimer()
        self.__logger.info('receive_mac: %s' % self.receive_mac)
        if self.receive_mac == mac_address:
            self.__logger.info('app connect success ')
            self.connected = 1
            self.__logger.info('App is connected, close advertise ')
        else:
            self.disconnect_device(self.mac_address_get)
            self.__logger.info(' disconnect :%s' % self.mac_address_get)
            self.connected = 0
        self.__logger.info('receive_mac: %s' % self.receive_mac)
        pass

    def set_connected_status(self, status):
        self.masters, self.slavers = self.parse_connections_output(
                                        self.runCommand('hcitool con'))
        if status == 1:
            self.__logger.info('connected')
            if len(self.slavers) > 0:
                self.__logger.info(' APP is connecting')
                self.connected = 1
                self.connected_timer = TimeoutTimer(10)
                self.connected_timer.addUserTimeoutFunc(self.check_connected)
                self.connected_timer.startTimer()
        else:
            self.__logger.info('disconnected')
            if len(self.slavers) == 0:
                self.connected = 0
                self.receive_mac = ''
                self.__logger.info('App is disconnected, open advertise ')
                pass

    def properties_changed(self, interface, changed, invalidated, path):
        if (interface == DEVICE_INTERFACE):
            if ('Connected' in changed):
                self.mac_address_get = self.extract_mac_address_from_path(path)
                self.mac_address_get = self.splite_mac_address_get(self.mac_address_get)
                self.__logger.info('connect_change')
                self.set_connected_status(changed['Connected'])

    def interfaces_added(self, path, interfaces):
        if DEVICE_INTERFACE in interfaces:
            properties = interfaces[DEVICE_INTERFACE]
            if ('Connected' in properties):
                self.mac_address_get = self.extract_mac_address_from_path(path)
                self.mac_address_get = self.splite_mac_address_get(self.mac_address_get)
                self.__logger.info('interface add')
                self.set_connected_status(properties['Connected'])

    def register_app_cb(self):
        self.__logger.info('register_app_cb: GATT application registered!!!')

    def register_app_error_cb(self, error):
        self.__logger.info('Failed to unregister application: %s' % str(error))

    def unregister_app_cb(self):
        self.__logger.info('register_app_cb: GATT application unregistered!!!')

    def unregister_app_error_cb(self, error):
        self.__logger.info('Failed to unregister application: %s' % str(error))

    def register_ad_cb(self):
        self.__logger.info('register_ad_cb: Advertisement registered!!!')
        self.adv_status = 1

    def register_ad_error_cb(self, error):
        self.__logger.info(' Failed to register advertisement: %s' % str(error))
        self.adv_status = 0

    def unregister_ad_cb(self):
        self.__logger.info('register_ad_cb: Advertisement Unregistered!!!')
        self.adv_status = 0

    def unregister_ad_error_cb(self, error):
        self.__logger.info('Failed to Unregister advertisement: %s' % str(error))
        self.adv_status = 1

    def RegADV(self, state):
        if state is True:
            self.__logger.info('RegADV->RegisterAdvertisement')
            self.ad_manager.RegisterAdvertisement(self.robotAdv.get_path(), {},
                                                  reply_handler=self.register_ad_cb,
                                                  error_handler=self.register_ad_error_cb)
        else:
            self.__logger.info('RegADV->UnregisterAdvertisement')
            self.ad_manager.UnregisterAdvertisement(self.robotAdv.get_path(),
                                                    reply_handler=self.unregister_ad_cb,
                                                    error_handler=self.unregister_ad_error_cb)
            time.sleep(3)

    # 功能：注册或销毁GATT服务
    def RegATT(self, state):
        if state is True:
            self.__logger.info('RegATT->RegisterApplication')
            self.service_manager.RegisterApplication(self.robotApp.get_path(), {},
                                                     reply_handler=self.register_app_cb,
                                                     error_handler=self.register_app_error_cb)
        else:
            self.__logger.info('RegATT->UnregisterApplication')
            self.service_manager.UnregisterApplication(self.robotApp.get_path(),
                                                       reply_handler=self.unregister_app_cb,
                                                       error_handler=self.unregister_app_error_cb)

    def bluetooth_status(self):
        return self.connected, self.adv_status

    def main_bluetooth(self):
        mainloop = GObject.MainLoop()
        mainloop.run()

    def splite_mac_address_get(self, address):
        self.__logger.info('splite_mac_address_get %s' % repr(address))
        match = re.match(r'DEV:(\w+)$', address)
        if match:
            target_str = match.group(1)
            return target_str
        else:
            self.__logger.info('not search match mac address')
            return ''

    def get_logger(self):
        return self.__logger

    def doNotify(self, ssid, ip, code, sn):
        gnotifystr = {
                'ssid': ssid,
                'ip': ip,
                'code': code,
                'sn': sn}
        self.__logger.info('doNotifyOnce %s' % repr(gnotifystr))
        arraySend = convert_to_dbus_array(json.dumps(gnotifystr))
        notifyChar.PropertiesChanged(GATT_CHRC_IFACE, {'Value': arraySend}, [])

    def GetAppData(self, ssid, pwd, ip, mac, type_receive):
        self.receive_ssid = ssid
        self.receive_pwd = pwd
        self.receive_ip = ip
        self.receive_mac = mac
        self.receive_type = type_receive
        # self.__logger.info('GetAppData: %s %s %s %s %s' %
        #                    (ssid, pwd, ip, mac, type_receive))

    def GetConnectStatus(self, status):
        self.get_connect_init_status = status
        # self.__logger.info('GetConnectStatus: %s' % status)
