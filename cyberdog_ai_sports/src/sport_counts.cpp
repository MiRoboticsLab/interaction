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
#include <math.h>
#include <vector>
#include <algorithm>
#include "cyberdog_ai_sports/sport_counts.hpp"

namespace cyberdog
{
namespace interaction
{
std::vector<float> MedianFilter(std::vector<float> & n, int & size)
{
  std::vector<float> num;
  if (size % 2 == 0) {
    // cout  << "Please enter odd size!" <<endl;
    return num;
  }
  if (size == 1) {
    num = n;
    return num;
  }
  for (int i = 0; i < n.size(); i++) {
    std::vector<float> data;
    for (auto j = -(size / 2); j <= size / 2; j++) {
      if (i + j < 0) {
        data.push_back(0);
      } else if (i + j > n.size()) {
        data.push_back(0);
      } else {
        data.push_back(n[i + j]);
      }
    }
    std::sort(data.begin(), data.end());
    num.push_back(data[size / 2]);
  }
  return num;
}
std::vector<int> DataFusion(std::vector<float> & num, int size)
{
  std::vector<int> data;
  int length = num.size();
  int s1 = length / size;
  int s2 = length % size;
  for (int i = 0; i < s1; i++) {
    int sum = 0;
    for (int j = 0; j < size; j++) {
      sum += num[i * size + j];
    }
    int average = sum / size;
    data.push_back(average);
  }
  if (s2 != 0) {
    int sum = 0;
    for (int i = 0; i < s2; i++) {
      sum += num[length - i - 1];
    }
    int average = sum / s2;
    data.push_back(average);
  }
  return data;
}
void SquatCounts(std::vector<std::vector<int>> & num, Squart & sports)
{
  int size = num[0].size();
  if (size < 1) {
    return;
  }
  for (int i = 0; i < size; i++) {
    if (sports.endState == 0 && num[0][i] <= sports.threMaxAngle &&
      num[1][i] <= sports.threMaxAngle)
    {
      sports.midState = 1;
    }
    if (sports.midState == 1 && num[0][i] <= sports.threMinAngle &&
      num[1][i] <= sports.threMinAngle)
    {
      sports.midState = 0;
      sports.endState = 1;
    }
    if (sports.midState == 1 && (num[0][i] > sports.threMaxAngle &&
      num[1][i] > sports.threMaxAngle))
    {
      sports.initialState = 1;
      if (sports.midState == 1 && sports.initialState == 1) {
        // std::cout<< "请在蹲下去一点" <<std::endl;
        sports.initialState = 0;
        sports.midState = 0;
      }
    }
    if (sports.endState == 1 && (num[0][i] > sports.threMaxAngle &&
      num[1][i] > sports.threMaxAngle))
    {
      sports.initialState = 1;
      if (sports.endState == 1 && sports.initialState == 1) {
        sports.count += 1;
        sports.initialState = 0;
        sports.endState = 0;
      }
    }
  }
}
void SitUpCounts(std::vector<std::vector<int>> & num, SitUp & sports)
{
  int size = num[0].size();
  if (size < 1) {
    return;
  }
  for (int i = 0; i < size; i++) {
    if ((sports.threMinAngle <= num[0][i] && num[0][i] <= sports.threMidAngle1) &&
      (sports.threMinAngle <= num[1][i] && num[1][i] <= sports.threMidAngle1))
    {
      sports.endState = 1;
    }
    if (sports.endState == 1 && (sports.threMidAngle2 <= num[0][i]) &&
      (num[0][i] <= sports.threMaxAngle) && (sports.threMidAngle2 <= num[1][i]) &&
      (num[1][i] <= sports.threMaxAngle))
    {
      sports.initialState = 1;
      if (sports.endState == 1 && sports.initialState == 1) {
        sports.count++;
        sports.initialState = 0;
        sports.endState = 0;
      }
    }
  }
}
void HighKneesCounts(std::vector<std::vector<int>> & num, HighKnees & sports)
{
  int size = num[0].size();
  int length = num.size();
  if (size < 1) {
    return;
  }
  for (int j = 0; j < size; j++) {
    for (int i = 0; i < length; i++) {
      if (sports.endState == 0 && (num[i][j] <= sports.threMaxAngle &&
        num[length - i - 1][j] > sports.threMaxAngle))
      {
        sports.midState = 1;
      }
      if (sports.midState == 1 && (num[i][j] <= sports.threMinAngle &&
        num[length - i - 1][j] > sports.threMinAngle))
      {
        sports.midState = 0;
        sports.endState = 1;
      }
      if (sports.midState == 1 && (num[i][j] > sports.threMaxAngle &&
        num[length - i - 1][j] > sports.threMaxAngle))
      {
        sports.initialState = 1;
        if (sports.midState == 1 && sports.initialState == 1) {
          // std::cout<< "高抬腿不标准" <<std::endl;
          sports.initialState = 0;
          sports.midState = 0;
        }
      }
      if (sports.endState == 1 && (num[i][j] > sports.threMaxAngle &&
        num[length - i - 1][j] > sports.threMaxAngle))
      {
        sports.initialState = 1;
        if (sports.endState == 1 && sports.initialState == 1) {
          sports.count += 1;
          sports.initialState = 0;
          sports.endState = 0;
        }
      }
    }
  }
}
void PlankTime(std::vector<std::vector<int>> & num, Plank & sports)
{
  int size = num[0].size();
  if (size < 1) {
    return;
  }
  for (int i = 0; i < size; i++) {
    if ((sports.threMinAngle <= num[0][i] && num[0][i] <= sports.threMaxAngle) &&
      (sports.threMinAngle <= num[1][i] && num[1][i] <= sports.threMaxAngle))
    {
      sports.endState = 1;
      sports.count++;
    }
    if (sports.endState == 1 && (sports.threMaxAngle <= num[0][i]) &&
      (sports.threMaxAngle <= num[1][i]))
    {
      sports.endState = 0;
    }
  }
}
void PushUpCounts(std::vector<std::vector<int>> & num, PushUp & sports)
{
  int size = num[0].size();
  if (size < 1) {
    return;
  }
  for (int i = 0; i < size; i++) {
    if (num[0][i] > sports.threMaxAngle && num[1][i] > sports.threMaxAngle) {
      int angle = num[0][i] > num[1][i] ? num[0][i] : num[1][i];
      sports.peckValue = sports.peckValue > angle ? sports.peckValue : angle;
      sports.endState = 1;
      if (sports.endState == 1 && (abs(num[0][i] - sports.peckValue) > 15 &&
        abs(num[0][i] - sports.peckValue) < 25) || (abs(num[1][i] - sports.peckValue) > 15 &&
        abs(num[1][i] - sports.peckValue) < 25))
      {
        sports.count++;
        sports.peckValue = 0;
        sports.endState = 0;
      }
    }
  }
}
void JumpJackCounts(std::vector<std::vector<int>> & num, JumpJack & sports)
{
  int size = num[0].size();
  if (size < 1) {
    return;
  }
  for (int i = 0; i < size; i++) {
    if ((num[0][i] >= sports.threMidAngle2 && num[0][i] <= sports.threMidAngle3) &&
      (num[1][i] >= sports.threMidAngle2 && num[1][i] <= sports.threMidAngle3) &&
      (num[2][i] >= sports.threMidAngle6 && num[2][i] <= sports.threMidAngle7))
    {
      sports.endState = 1;
    }
    if (sports.endState == 1 && (num[0][i] <= sports.threMidAngle1) &&
      (num[1][i] <= sports.threMidAngle1) &&
      (num[2][i] >= sports.threMidAngle4 && num[2][i] <= sports.threMidAngle5))
    {
      sports.initialState = 1;
      if (sports.initialState == 1 && sports.endState == 1) {
        sports.count++;
        sports.initialState = 0;
        sports.endState = 0;
      }
    }
  }
}
}  // namespace interaction
}  // namespace cyberdog
