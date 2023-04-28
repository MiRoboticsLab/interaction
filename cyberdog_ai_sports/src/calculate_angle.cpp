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
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <cstring>
#include "cyberdog_ai_sports/calculate_angle.hpp"


namespace cyberdog
{
namespace interaction
{
Angle::Angle(/* args */) {}
Angle::~Angle() {}

float Angle::Angle_Between_Points(XMPoint p0, XMPoint p1, XMPoint p2)
{
  float a = (p1.x - p0.x) * (p1.x - p0.x) + (p1.y - p0.y) * (p1.y - p0.y);
  float b = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
  float c = (p2.x - p0.x) * (p2.x - p0.x) + (p2.y - p0.y) * (p2.y - p0.y);
  if (a * b == 0) {
    return -1.0;
  }
  float angleValue = (a + b - c) / sqrt(4 * a * b);
  if (angleValue > 1) {
    angleValue = 1.0;
  } else if (angleValue < -1) {
    angleValue = -1.0;
  }
  return std::acos(angleValue) * 180 / PI;
}
std::vector<XMPoint> Angle::Get_Angle_Point(
  std::vector<XMPoint> & keypoint,
  const std::string & pos)
{
  std::vector<XMPoint> pnts;
  if (pos == "left_elbow") {
    pnts.push_back(keypoint[7]);
    pnts.push_back(keypoint[9]);
    pnts.push_back(keypoint[11]);
  }
  if (pos == "left_knee") {
    pnts.push_back(keypoint[13]);
    pnts.push_back(keypoint[15]);
    pnts.push_back(keypoint[17]);
  }
  if (pos == "left_ankle") {
    pnts.push_back(keypoint[6]);
    pnts.push_back(keypoint[12]);
    pnts.push_back(keypoint[16]);
  }
  if (pos == "left_shoulder") {
    pnts.push_back(keypoint[9]);
    pnts.push_back(keypoint[7]);
    pnts.push_back(keypoint[13]);
  }
  if (pos == "left_hip") {
    pnts.push_back(keypoint[7]);
    pnts.push_back(keypoint[13]);
    pnts.push_back(keypoint[15]);
  }
  if (pos == "left_push_ups") {
    pnts.push_back(keypoint[0]);
    pnts.push_back(keypoint[7]);
    pnts.push_back(keypoint[1]);
  }
  if (pos == "rihgt_elbow") {
    pnts.push_back(keypoint[8]);
    pnts.push_back(keypoint[10]);
    pnts.push_back(keypoint[12]);
  }
  if (pos == "rihgt_knee") {
    pnts.push_back(keypoint[14]);
    pnts.push_back(keypoint[16]);
    pnts.push_back(keypoint[18]);
  }
  if (pos == "rihgt_ankle") {
    pnts.push_back(keypoint[7]);
    pnts.push_back(keypoint[13]);
    pnts.push_back(keypoint[17]);
  }
  if (pos == "rihgt_shoulder") {
    pnts.push_back(keypoint[10]);
    pnts.push_back(keypoint[8]);
    pnts.push_back(keypoint[14]);
  }
  if (pos == "rihgt_hip") {
    pnts.push_back(keypoint[8]);
    pnts.push_back(keypoint[14]);
    pnts.push_back(keypoint[16]);
  }
  if (pos == "rihgt_push_ups") {
    pnts.push_back(keypoint[0]);
    pnts.push_back(keypoint[8]);
    pnts.push_back(keypoint[1]);
  }
  if (pos == "nose") {
    pnts.push_back(keypoint[17]);
    pnts.push_back(keypoint[2]);
    pnts.push_back(keypoint[18]);
  }

  return pnts;
}
float Angle::Length_Between_Points(XMPoint p0, XMPoint p1)
{
  return sqrt(pow(p1.x - p0.x, 2) + (p1.y - p0.y, 2));
}

float Angle::Angle_Nose(std::vector<XMPoint> & keypoint)
{
  Angle angle;
  std::vector<XMPoint> pnts = angle.Get_Angle_Point(keypoint, "nose");
  if (pnts.size() != 3) {
    // std::cout << "component incomplete" << std::endl;
    return -500;
  }
  float CalculateAngle = 0.0;
  if (pnts.size() != 0) {
    CalculateAngle = angle.Angle_Between_Points(pnts[0], pnts[1], pnts[2]);
    // std::cout << "nose angle:" << CalculateAngle << std::endl;
  }
  return CalculateAngle;
}
float Angle::Angle_Left_Elbow(std::vector<XMPoint> & keypoint)
{
  Angle angle;
  std::vector<XMPoint> pnts = angle.Get_Angle_Point(keypoint, "left_elbow");
  if (pnts.size() != 3) {
    // std::cout << "component incomplete" << std::endl;
    return -500;
  }
  float CalculateAngle = 0.0;
  if (pnts.size() != 0) {
    if (pnts.size() != 0) {
      CalculateAngle = angle.Angle_Between_Points(pnts[0], pnts[1], pnts[2]);
      // std::cout << "left elbow angle:" << CalculateAngle << std::endl;
    }
    return CalculateAngle;
  }
}
float Angle::Angle_Left_Knee(std::vector<XMPoint> & keypoint)
{
  Angle angle;
  std::vector<XMPoint> pnts = angle.Get_Angle_Point(keypoint, "left_knee");
  if (pnts.size() != 3) {
    // std::cout << "component incomplete" << std::endl;
    return -500;
  }
  float CalculateAngle = 0.0;
  if (pnts.size() != 0) {
    CalculateAngle = angle.Angle_Between_Points(pnts[0], pnts[1], pnts[2]);
    // std::cout << "left knee angle:" << CalculateAngle << endl;
  }
  return CalculateAngle;
}
float Angle::Angle_Left_Ankle(std::vector<XMPoint> & keypoint)
{
  Angle angle;
  std::vector<XMPoint> pnts = angle.Get_Angle_Point(keypoint, "left_ankle");
  if (pnts.size() != 3) {
    // std::cout << "component incomplete" << endl;
    return -500;
  }
  float CalculateAngle = 0.0;
  if (pnts.size() != 0) {
    CalculateAngle = angle.Angle_Between_Points(pnts[0], pnts[1], pnts[2]);
    // std::cout << "left ankle angle:" << CalculateAngle << endl;
  }
  return CalculateAngle;
}
float Angle::Angle_Left_Shoulder(std::vector<XMPoint> & keypoint)
{
  Angle angle;
  std::vector<XMPoint> pnts = angle.Get_Angle_Point(keypoint, "left_shoulder");
  if (pnts.size() != 3) {
    // std::cout << "component incomplete" << endl;
    return -500;
  }
  float CalculateAngle = 0.0;
  if (pnts.size() != 0) {
    CalculateAngle = angle.Angle_Between_Points(pnts[0], pnts[1], pnts[2]);
    // std::cout << "left shoulder angle:" << CalculateAngle << endl;
  }
  return CalculateAngle;
}
float Angle::Angle_Left_Hip(std::vector<XMPoint> & keypoint)
{
  Angle angle;
  std::vector<XMPoint> pnts = angle.Get_Angle_Point(keypoint, "left_hip");
  if (pnts.size() != 3) {
    // std::cout << "component incomplete" << endl;
    return -500;
  }
  float CalculateAngle = 0.0;
  if (pnts.size() != 0) {
    CalculateAngle = angle.Angle_Between_Points(pnts[0], pnts[1], pnts[2]);
    // std::cout << "left hip angle:" << CalculateAngle << endl;
  }
  return CalculateAngle;
}
float Angle::Angle_Left_PushUps(std::vector<XMPoint> & keypoint)
{
  Angle angle;
  std::vector<XMPoint> pnts = angle.Get_Angle_Point(keypoint, "left_push_ups");
  if (pnts.size() != 3) {
    // std::cout << "component incomplete" << endl;
    return -500;
  }
  float CalculateAngle = 0.0;
  if (pnts.size() != 0) {
    CalculateAngle = angle.Angle_Between_Points(pnts[0], pnts[1], pnts[2]);
    // std::cout << "left push ups angle:" << CalculateAngle << endl;
  }
  return CalculateAngle;
}
float Angle::Angle_Right_Elbow(std::vector<XMPoint> & keypoint)
{
  Angle angle;
  std::vector<XMPoint> pnts = angle.Get_Angle_Point(keypoint, "rihgt_elbow");
  if (pnts.size() != 3) {
    // std::cout << "component incomplete" << endl;
    return -500;
  }
  float CalculateAngle = 0.0;
  if (pnts.size() != 0) {
    CalculateAngle = angle.Angle_Between_Points(pnts[0], pnts[1], pnts[2]);
    // std::cout << "right elbow angle:" << CalculateAngle << endl;
  }
  return CalculateAngle;
}
float Angle::Angle_Right_Knee(std::vector<XMPoint> & keypoint)
{
  Angle angle;
  std::vector<XMPoint> pnts = angle.Get_Angle_Point(keypoint, "rihgt_knee");
  if (pnts.size() != 3) {
    // std::cout << "component incomplete" << endl;
    return -500;
  }
  float CalculateAngle = 0.0;
  if (pnts.size() != 0) {
    CalculateAngle = angle.Angle_Between_Points(pnts[0], pnts[1], pnts[2]);
    // std::cout << "right knee angle:" << CalculateAngle << endl;
  }
  return CalculateAngle;
}
float Angle::Angle_Right_Ankle(std::vector<XMPoint> & keypoint)
{
  Angle angle;
  std::vector<XMPoint> pnts = angle.Get_Angle_Point(keypoint, "rihgt_ankle");
  if (pnts.size() != 3) {
    // std::cout << "component incomplete" << endl;
    return -500;
  }
  float CalculateAngle = 0.0;
  if (pnts.size() != 0) {
    CalculateAngle = angle.Angle_Between_Points(pnts[0], pnts[1], pnts[2]);
    // std::cout << "right ankle angle:" << CalculateAngle << endl;
  }
  return CalculateAngle;
}
float Angle::Angle_Right_Shoulder(std::vector<XMPoint> & keypoint)
{
  Angle angle;
  std::vector<XMPoint> pnts = angle.Get_Angle_Point(keypoint, "rihgt_shoulder");
  if (pnts.size() != 3) {
    // std::cout << "component incomplete" << endl;
    return -500;
  }
  float CalculateAngle = 0.0;
  if (pnts.size() != 0) {
    CalculateAngle = angle.Angle_Between_Points(pnts[0], pnts[1], pnts[2]);
    // std::cout << "rihgt shoulder angle:" << CalculateAngle << endl;
  }
  return CalculateAngle;
}
float Angle::Angle_Right_Hip(std::vector<XMPoint> & keypoint)
{
  Angle angle;
  std::vector<XMPoint> pnts = angle.Get_Angle_Point(keypoint, "rihgt_hip");
  if (pnts.size() != 3) {
    // std::cout << "component incomplete" << endl;
    return -500;
  }
  float CalculateAngle = 0.0;
  if (pnts.size() != 0) {
    CalculateAngle = angle.Angle_Between_Points(pnts[0], pnts[1], pnts[2]);
    // std::cout << "rihgt hip angle:" << CalculateAngle << endl;
  }
  return CalculateAngle;
}
float Angle::Angle_Right_PushUps(std::vector<XMPoint> & keypoint)
{
  Angle angle;
  std::vector<XMPoint> pnts = angle.Get_Angle_Point(keypoint, "rihgt_push_ups");
  if (pnts.size() != 3) {
    // std::cout << "component incomplete" << endl;
    return -500;
  }
  float CalculateAngle = 0.0;
  if (pnts.size() != 0) {
    CalculateAngle = angle.Angle_Between_Points(pnts[0], pnts[1], pnts[2]);
    // std::cout << "rihgt push ups angle:" << CalculateAngle << endl;
  }
  return CalculateAngle;
}
}  // namespace interaction
}  // namespace cyberdog
