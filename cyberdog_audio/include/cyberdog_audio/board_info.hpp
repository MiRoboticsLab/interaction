// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef CYBERDOG_AUDIO__BOARD_INFO_HPP_
#define CYBERDOG_AUDIO__BOARD_INFO_HPP_

#include <string>
#include <fstream>
#include <vector>

#define   INFO_FILE_NAME    "/dev/disk/by-partlabel/misc"
#define   IOT_CRC_LEN       68
#define   MAC_LEN           6
#define   DID_LEN           8
#define   KEY_LEN           16

namespace cyberdog
{
namespace interaction
{
class BoardInfo final
{
public:
  BoardInfo() = delete;
  static std::string Get_Sn()
  {
    char buf[256] = {0};
    std::ifstream infile(INFO_FILE_NAME, std::ifstream::in);
    if (infile.is_open()) {
      infile.seekg(10485760, std::ios::beg);
      infile.get(buf, sizeof(buf));
    }
    return buf;
  }
  static bool Get_Iot(std::string & mac, std::string & did, std::string & key)
  {
    char buf[IOT_CRC_LEN + 1] = {0};
    std::ifstream infile(INFO_FILE_NAME, std::ifstream::in);
    if (infile.is_open()) {
      infile.seekg(11534336, std::ios::beg);
      infile.get(buf, sizeof(buf));
    } else {
      return false;
    }
    uint8_t tmp_mac[MAC_LEN + 1] = {0};
    uint64_t tmp_did = 0;
    uint8_t tmp_key[KEY_LEN + 1] = {0};
    uint32_t code = parse_otpstr(buf, tmp_mac, &tmp_did, tmp_key);
    // mac.assign((char*)tmp_mac, MAC_LEN);
    mac = hex_str(tmp_mac, MAC_LEN);
    did = std::to_string(tmp_did);
    key.assign(reinterpret_cast<char *>(tmp_key), KEY_LEN);
    return code == 0;
  }

private:
  static std::string hex_str(const uint8_t * data, const size_t size)
  {
    if (data == NULL) {
      return "";
    }
    std::string buff;
    const int len = size;
    for (int j = 0; j < len; j++) {
      int high = data[j] / 16, low = data[j] % 16;
      buff += (high < 10) ? ('0' + high) : ('a' + high - 10);
      buff += (low < 10) ? ('0' + low) : ('a' + low - 10);
    }
    return buff;
  }
  static int unhexify(const char * hexstr, uint8_t hexlen, uint8_t * out_buff)
  {
    unsigned char c, c2;
    if (hexlen > strlen(hexstr)) {
      hexlen = strlen(hexstr);
    }
    int len = hexlen / 2;
    if (0 != hexlen % 2) { /* must be even number of bytes */
      return -1;
    }
    for (int i = 0; i < len; ++i) {
      c = *(hexstr + i * 2);
      if (c >= '0' && c <= '9') {
        c -= '0';
      } else if (c >= 'a' && c <= 'f') {
        c -= 'a' - 10;
      } else if (c >= 'A' && c <= 'F') {
        c -= 'A' - 10;
      } else {return -1;}
      c2 = *(hexstr + i * 2 + 1);
      if (c2 >= '0' && c2 <= '9') {
        c2 -= '0';
      } else if (c2 >= 'a' && c2 <= 'f') {
        c2 -= 'a' - 10;
      } else if (c2 >= 'A' && c2 <= 'F') {
        c2 -= 'A' - 10;
      } else {
        return -1;
      }
      *out_buff++ = ( c << 4 ) | c2;
    }
    return len;
  }
  static uint32_t crc32(const uint8_t * p_data, uint32_t size, uint32_t init_crc)
  {
    uint32_t crc = init_crc; uint32_t i, j;
    for (i = 0; i < size; i++) {
      crc = crc ^ p_data[i];
      for (j = 8; j > 0; j--) {
        crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
      }
    }
    return crc;
  }
  static uint32_t parse_otpstr(
    const char * otphexstr, uint8_t * out_mac, uint64_t * out_did,
    uint8_t * out_key)
  {
    if (NULL == otphexstr || NULL == out_mac || NULL == out_did || NULL == out_key) {
      return -1;
    }
    if (strlen(otphexstr) != (MAC_LEN + DID_LEN + KEY_LEN + 4) * 2) {
      return -2;
    }
    uint8_t otp_mac[MAC_LEN] = {0};
    uint8_t otp_did[DID_LEN] = {0};
    uint8_t otp_key[KEY_LEN + 1] = {0};
    uint8_t otp_crc[4] = {0};
    char did_hex[DID_LEN * 2 + 1] = {0};
    char crc_hex[9] = {0};
    strncpy(did_hex, otphexstr + MAC_LEN * 2, DID_LEN * 2);
    did_hex[DID_LEN * 2] = '\0';
    strncpy(crc_hex, otphexstr + (MAC_LEN + DID_LEN + KEY_LEN) * 2, 8);
    crc_hex[8] = '\0';
    unhexify(otphexstr, MAC_LEN * 2, otp_mac);
    unhexify(did_hex, DID_LEN * 2, otp_did);
    unhexify(otphexstr + (MAC_LEN + DID_LEN) * 2, KEY_LEN * 2, otp_key);
    unhexify(crc_hex, 8, otp_crc);
    uint8_t tripledata[MAC_LEN + DID_LEN + KEY_LEN] = {0};
    char * endptr = NULL;
    uint32_t otp_crc_val = 0;
    uint32_t sum_crc_val = 0;
    memcpy(tripledata, otp_mac, MAC_LEN);
    memcpy(tripledata + MAC_LEN, otp_did, DID_LEN);
    memcpy(tripledata + MAC_LEN + DID_LEN, otp_key, KEY_LEN);
    otp_crc_val = strtol(crc_hex, &endptr, 16);
    sum_crc_val = crc32(tripledata, MAC_LEN + DID_LEN + KEY_LEN, 0);
    if (otp_crc_val != sum_crc_val) {
      return -3;
    }
    memcpy(out_mac, otp_mac, MAC_LEN);
    *out_did = strtoul(did_hex, &endptr, 16);
    memcpy(out_key, otp_key, KEY_LEN);
    return 0;
  }
};
}  // namespace interaction
}  // namespace cyberdog

#endif  // CYBERDOG_AUDIO__BOARD_INFO_HPP_
