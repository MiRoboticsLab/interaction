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

#ifndef API__VPAPY_HPP_
#define API__VPAPY_HPP_

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include <pybind11/stl_bind.h>

namespace cyberdog_visual_programming_abilityset_py
{
namespace py = pybind11;
// using namespace py::literals;                  /*!< 短符号参数命名 */

void DefineBuiltinInterfaces(py::object);         /*!< 定义 builtin_interfaces */
void DefineTimer(py::object);                     /*!< 定义 Timer */

void DefineStdMsgs(py::object);                   /*!< 定义 std_msgs */
void DefineHeader(py::object);                    /*!< 定义 Header */

void DefineSensorMsgs(py::object);                /*!< 定义 sensor_msgs */
void DefineMsgRange(py::object);                  /*!< 定义 Range */
void DefineMsgLaserScan(py::object);              /*!< 定义 LaserScan */
void DefineMsgImu(py::object);                    /*!< 定义 Imu */

void DefineGeometryMsgs(py::object);              /*!< 定义 geometry_msgs */
void DefinePoint(py::object);                     /*!< 定义 Point */
void DefineQuaternion(py::object);                /*!< 定义 Quaternion */
void DefinePose(py::object);                      /*!< 定义 Pose */
void DefineVector3(py::object);                   /*!< 定义 Vector3 */
void DefineTwist(py::object);                     /*!< 定义 Twist */
void DefinePoseWithCovariance(py::object);        /*!< 定义 PoseWithCovariance */
void DefineTwistWithCovariance(py::object);       /*!< 定义 TwistWithCovariance */

void DefineNavMsgs(py::object);                   /*!< 定义 nav_msgs */
void DefineMsgOdometry(py::object);               /*!< 定义 MsgOdometry */

void DefineProtocol(py::object);                  /*!< 定义 protocol */
void DefineMsgBmsStatus(py::object);              /*!< 定义 MsgBmsStatus */
void DefineMsgTouchStatus(py::object);            /*!< 定义 MsgTouchStatus */
void DefineMsgConnectorStatus(py::object);        /*!< 定义 MsgConnectorStatus */
void DefineMsgMotionSequenceGait(py::object);     /*!< 定义 MotionSequencePace */
void DefineMsgMotionSequencePace(py::object);     /*!< 定义 MotionSequencePace */
void DefineMsgGpsPayload(py::object);             /*!< 定义 GpsPayload */
void DefineMsgSingleTofPayload(py::object);       /*!< 定义 SingleTofPayload */
void DefineMsgHeadTofPayload(py::object);         /*!< 定义 HeadTofPayload */
void DefineMsgRearTofPayload(py::object);         /*!< 定义 RearTofPayload */
void DefineMsgSport(py::object);                  /*!< 定义 MsgSport */
void DefineMsgTrainingWords(py::object);          /*!< 定义 MsgTrainingWords */

void DefineSrvFaceRecResponse(py::object);        /*!< 定义 SrvFaceRecResponse */
void DefineSrvSportResponse(py::object);          /*!< 定义 SrvSportResponse */
void DefineSrvTrainingWordsResponse(py::object);  /*!< 定义 SrvTrainingWordsResponse */
void DefineSrvAudioTextPlayResponse(py::object);  /*!< 定义 SrvAudioTextPlayResponse */
void DefineSrvLedExecuteResponse(py::object);     /*!< 定义 SrvLedExecuteResponse */
void DefineSrvMotionResultCmdResponse(
  py::object
);                                                /*!< 定义 SrvMotionResultCmdResponse */
void DefineSrvMotionSequenceShowResponse(
  py::object);                                    /*!< 定义 SrvMotionSequenceShowResponse */

void DefineActNavigationResult(py::object);       /*!< 定义 ActNavigation::Result */

void DefineMsgFaceRes(py::object);                /*!< 定义 MsgFaceRes */
void DefineMsgPreset(py::object);                 /*!< 定义 MsgPreset */

void DefineCommonType(py::object);                /*!< 定义 通用类型 */
void DefineState(py::object);                     /*!< 定义 State */
void DefineAudioPlaySeviceResponse(py::object);   /*!< 定义 AudioPlaySeviceResponse */
void DefineAudioGetVolumeSeviceResponse(
  py::object);                                    /*!< 定义 AudioGetVolumeSeviceResponse */
void DefineAudioSetVolumeSeviceResponse(
  py::object);                                    /*!< 定义 AudioSetVolumeSeviceResponse */
void DefineLedConstraint(py::object);             /*!< 定义 LedConstraint */
void DefineLedSeviceResponse(py::object);         /*!< 定义 LedSeviceResponse */
void DefineMotionResultServiceResponse(
  py::object);                                    /*!< 定义 MotionResultServiceResponse */
void DefineMotionSequenceServiceResponse(
  py::object);                                    /*!< 定义 MotionSequenceServiceResponse */
void DefineMotionServoCmdResponse(py::object);    /*!< 定义 MotionServoCmdResponse */
void DefineMotionParams(py::object);              /*!< 定义 MotionParams */
void DefineMotionId(py::object);                  /*!< 定义 MotionId */

void DefineMsgMotionSequenceGaitList(
  py::object
);                                                /*!< 定义 MsgMotionSequenceGaitList */
void DefineMsgMotionSequencePaceList(
  py::object
);                                                /*!< 定义 MsgMotionSequencePaceList */
void DefineMotionSequence(py::object);            /*!< 定义 ObstacleMeta */
void DefineObstacleMeta(py::object);              /*!< 定义 TofObstacle */
void DefineTofObstacle(py::object);               /*!< 定义 MotionSequence */
void DefineTofPayload(py::object);                /*!< 定义 TofPayload */

void DefineMsgPersonnel(py::object);              /*!< 定义 MsgPersonnel */
void DefineMsgPersonnelList(py::object);          /*!< 定义 MsgPersonnelList */
void DefineSrvPersonnelResponse(py::object);      /*!< 定义 SrvPersonnelResponse */

void DefineFaceSeviceResponse(
  py::object);                                    /*!< 定义 FaceSeviceResponse */
void DefineFaceRecognizedSeviceResponse(
  py::object);                                    /*!< 定义 FaceRecognizedSeviceResponse */
void DefineVoiceprintRecognizedResponse(
  py::object);                                    /*!< 定义 VoiceprintRecognizedResponse */

void DefineGestureData(py::object);               /*!< 定义 GestureData */
void DefineGestureType(py::object);               /*!< 定义 GestureType */
void DefineGestureRecognizedSeviceResponse(
  py::object);                                    /*!< 定义 GestureRecognizedSeviceResponse */
void DefineGestureRecognizedMessageResponse(
  py::object);                                    /*!< 定义 GestureRecognizedMessageResponse */
void DefineSkeletonType(py::object);              /*!< 定义 SkeletonType */
void DefineSkeletonRecognizedSeviceResponse(
  py::object);                                    /*!< 定义 SkeletonRecognizedSeviceResponse */
void DefineSkeletonRecognizedMessageResponse(
  py::object);                                    /*!< 定义 SkeletonRecognizedMessageResponse */

void DefineTrainingWordsRecognizedSeviceResponse(
  py::object);                                    /*!< 定义 TrainingWordsRecognizedSevice... */
void DefineTrainingWordsRecognizedMessageResponse(
  py::object);                                    /*!< 定义 TrainingWordsRecognizedMessag... */

void DefineMapPresetSeviceResponse(py::object);   /*!< 定义 MapPresetSeviceResponse */
void DefineNavigationActionResponse(py::object);  /*!< 定义 NavigationActionResponse */

void DefineBase(py::object);                      /*!< 定义 Base */

void DefineNetwork(py::object);                   /*!< 定义 Network */
void DefineFollow(py::object);                    /*!< 定义 Follow */
void DefineMotion(py::object);                    /*!< 定义 Motion */
void DefineNavigation(py::object);                /*!< 定义 Navigation */
void DefineTask(py::object);                      /*!< 定义 Task */
void DefineTrain(py::object);                     /*!< 定义 Train */
void DefinePersonnel(py::object);                 /*!< 定义 Personnel */
void DefineGesture(py::object);                   /*!< 定义 Gesture */
void DefineSkeleton(py::object);                  /*!< 定义 Skeleton */

void DefineAudio(py::object);                     /*!< 定义 Audio */
void DefineLed(py::object);                       /*!< 定义 Led */
void DefineBms(py::object);                       /*!< 定义 Bms */
void DefineTouch(py::object);                     /*!< 定义 Touch */
void DefineGps(py::object);                       /*!< 定义 Gps */
void DefineTof(py::object);                       /*!< 定义 Tof */
void DefineLidar(py::object);                     /*!< 定义 Lidar */
void DefineUltrasonic(py::object);                /*!< 定义 Ultrasonic */
void DefineOdometer(py::object);                  /*!< 定义 Odometer */
void DefineImu(py::object);                       /*!< 定义 Imu */

void DefineCyberdog(py::object);                  /*!< 定义 Cyberdog */
}  // namespace cyberdog_visual_programming_abilityset_py

#endif  // API__VPAPY_HPP_
