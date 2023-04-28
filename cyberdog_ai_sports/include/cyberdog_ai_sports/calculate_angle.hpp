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
#ifndef CYBERDOG_AI_SPORTS__CALCULATE_ANGLE_HPP_
#define CYBERDOG_AI_SPORTS__CALCULATE_ANGLE_HPP_
#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>


#define PI acos(-1)
namespace cyberdog
{
namespace interaction
{
struct XMPoint
{
  XMPoint()
  : x(0), y(0) {}
  XMPoint(float x_val, float y_val)
  : x(x_val), y(y_val) {}
  float x;
  float y;
};
class Angle
{
public:
  Angle(/* args */);
  ~Angle();
  std::vector<XMPoint> Get_Angle_Point(std::vector<XMPoint> & keypoint, const std::string & pos);
  float Angle_Between_Points(XMPoint p0, XMPoint p1, XMPoint p2);
  float Length_Between_Points(XMPoint p0, XMPoint p1);
  float Angle_Nose(std::vector<XMPoint> & keypoint);

  float Angle_Left_Elbow(std::vector<XMPoint> & keypoint);
  float Angle_Left_Knee(std::vector<XMPoint> & keypoint);
  float Angle_Left_Ankle(std::vector<XMPoint> & keypoint);
  float Angle_Left_Shoulder(std::vector<XMPoint> & keypoint);
  float Angle_Left_Hip(std::vector<XMPoint> & keypoint);
  float Angle_Left_PushUps(std::vector<XMPoint> & keypoint);

  float Angle_Right_Elbow(std::vector<XMPoint> & keypoint);
  float Angle_Right_Knee(std::vector<XMPoint> & keypoint);
  float Angle_Right_Ankle(std::vector<XMPoint> & keypoint);
  float Angle_Right_Shoulder(std::vector<XMPoint> & keypoint);
  float Angle_Right_Hip(std::vector<XMPoint> & keypoint);
  float Angle_Right_PushUps(std::vector<XMPoint> & keypoint);
};
}  // namespace interaction
}  // namespace cyberdog
#endif  // CYBERDOG_AI_SPORTS__CALCULATE_ANGLE_HPP_
