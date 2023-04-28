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

import base64

from Crypto.Cipher import AES


def pading(key, text):
    return text + (len(key) - len(text) % len(key)) * chr(
        len(key) - len(text) % len(key)
    )


def unpading(text):
    return text[0:-ord(text[-1:])]


def getKey(key):
    key_len = len(key)
    if key_len <= 16:
        key += '0' * (16 - key_len)
    elif 16 < key_len <= 24:
        key += '0' * (24 - key_len)
    elif key_len <= 32:
        key += '0' * (32 - key_len)
    else:
        key = key[:32]
    return key


def encrypt(key, text):
    try:
        cryptor = AES.new(getKey(key).encode('utf-8'), AES.MODE_ECB)  # ECB 模式
        ciphertext = cryptor.encrypt(bytes(pading(text), encoding='utf-8'))
        encrypt_string = str(base64.b64encode(ciphertext)).lstrip('b')
    except ValueError as tar_error:
        print('[encrypt]:Key incorrect or message corrupted, error :', tar_error)
        encrypt_string = ''
    return encrypt_string


def decrypt(key, text):
    try:
        cryptor = AES.new(getKey(key).encode('utf-8'), AES.MODE_ECB)  # ECB 模式
        plain_text = cryptor.decrypt(base64.b64decode(text.encode('utf-8')))
        plain_text = plain_text.strip()
        plain_text = plain_text.decode('utf-8')
        decrypt_string = plain_text.strip(b'\x00'.decode())
    except ValueError as tar_error:
        print('[decrypt]:Key incorrect or message corrupted, error :', tar_error)
        decrypt_string = ''
    return decrypt_string
