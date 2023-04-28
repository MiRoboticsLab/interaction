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
#ifndef CYBERDOG_AI_SPORTS__SPORT_COUNTS_HPP_
#define CYBERDOG_AI_SPORTS__SPORT_COUNTS_HPP_
#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

namespace cyberdog
{
namespace interaction
{
typedef struct Squart
{
  Squart(int fusion, int Filter, int mix, int max)
  {
    initialState = 0;
    midState = 0;
    endState = 0;
    count = 0;
    fusionNumber = fusion;
    filterNumber = Filter;
    threMinAngle = mix;
    threMaxAngle = max;
  }
  int initialState;
  int midState;
  int endState;
  int count;
  int fusionNumber;
  int filterNumber;
  int threMinAngle;
  int threMaxAngle;
} Squart;
typedef struct PushUp
{
  PushUp(int fusion, int filter, int difmix, int difmax, int max)
  {
    initialState = 0;
    endState = 0;
    peckValue = 0;
    count = 0;
    fusionNumber = fusion;
    filterNumber = filter;
    difMinAngle = difmix;
    difMaxAngle = difmax;
    threMaxAngle = max;
  }
  int initialState;
  int endState;
  int peckValue;
  int count;
  int fusionNumber;
  int filterNumber;
  int difMinAngle;
  int difMaxAngle;
  int threMaxAngle;
} PushUp;
typedef struct SitUp
{
  SitUp(int fusion, int filter, int mix, int mid1, int mid2, int max)
  {
    initialState = 0;
    // midState = 0;
    endState = 0;
    count = 0;
    fusionNumber = fusion;
    filterNumber = filter;
    threMinAngle = mix;
    threMidAngle1 = mid1;
    threMidAngle2 = mid2;
    threMaxAngle = max;
  }
  int initialState;
  // int midState;
  int endState;
  int count;
  int fusionNumber;
  int filterNumber;
  int threMinAngle;
  int threMidAngle1;
  int threMidAngle2;
  int threMaxAngle;
} SitUp;
typedef struct HighKnees
{
  HighKnees(int fusion, int filter, int mix, int max)
  {
    initialState = 0;
    midState = 0;
    endState = 0;
    count = 0;
    fusionNumber = fusion;
    filterNumber = filter;
    threMinAngle = mix;
    threMaxAngle = max;
  }
  int initialState;
  int midState;
  int endState;
  int count;
  int fusionNumber;
  int filterNumber;
  int threMinAngle;
  int threMaxAngle;
} HighKnees;
typedef struct Plank
{
  Plank(int fusion, int filter, int mix, int max)
  {
    initialState = 0;
    // midState = 0;
    endState = 0;
    count = 0;
    fusionNumber = fusion;
    filterNumber = filter;
    threMinAngle = mix;
    threMaxAngle = max;
  }
  int initialState;
  // int midState;
  int endState;
  int count;
  int fusionNumber;
  int filterNumber;
  int threMinAngle;
  int threMaxAngle;
} Plank;
typedef struct JumpJack
{
  JumpJack(
    int fusion, int filter, int value1, int value2, int value3,
    int value4, int value5, int value6, int value7)
  {
    initialState = 0;
    // midState = 0;
    endState = 0;
    count = 0;
    fusionNumber = fusion;
    filterNumber = filter;
    threMidAngle1 = value1;
    threMidAngle2 = value2;
    threMidAngle3 = value3;
    threMidAngle4 = value4;
    threMidAngle5 = value5;
    threMidAngle6 = value6;
    threMidAngle7 = value7;
  }
  int initialState;
  // int midState;
  int endState;
  int count;
  int fusionNumber;
  int filterNumber;
  int threMidAngle1;
  int threMidAngle2;
  int threMidAngle3;
  int threMidAngle4;
  int threMidAngle5;
  int threMidAngle6;
  int threMidAngle7;
} jumpJackJumpJack;
template<class Type>
Type stringToNum(const std::string str);
std::vector<float> MedianFilter(std::vector<float> & n, int & size);      // 中值滤波
std::vector<int> DataFusion(std::vector<float> & num, int size);          // 数据融合
void SquatCounts(std::vector<std::vector<int>> & num, Squart & sports);
void SitUpCounts(std::vector<std::vector<int>> & num, SitUp & sports);
void HighKneesCounts(std::vector<std::vector<int>> & num, HighKnees & sports);
void PlankTime(std::vector<std::vector<int>> & num, Plank & sports);
void PushUpCounts(std::vector<std::vector<int>> & num, PushUp & sports);
void JumpJackCounts(std::vector<std::vector<int>> & num, JumpJack & sports);
}  // namespace interaction
}  // namespace cyberdog
#endif  // CYBERDOG_AI_SPORTS__SPORT_COUNTS_HPP_
