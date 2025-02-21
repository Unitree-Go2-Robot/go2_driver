// Copyright 2024 Intelligent Robotics Lab
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

#ifndef GO2_DRIVER__GO2_API_ID_HPP
#define GO2_DRIVER__GO2_API_ID_HPP

namespace go2_driver
{

enum class Mode
{
  Damp = 1001,
  BalanceStand = 1002,
  StopMove = 1003,
  StandUp = 1004,
  StandDown = 1005,
  RecoveryStand = 1006,
  Euler = 1007,
  Move = 1008,
  Sit = 1009,
  RiseSit = 1010,
  SwitchGait = 1011,
  Trigger = 1012,
  BodyHeight = 1013,
  FootRaiseHeight = 1014,
  SpeedLevel = 1015,
  Hello = 1016,
  Stretch = 1017,
  TrajectoryFollow = 1018,
  ContinuousGait = 1019,
  Content = 1020,
  Wallow = 1021,
  Dance1 = 1022,
  Dance2 = 1023,
  GetBodyHeight = 1024,
  GetFootRaiseHeight = 1025,
  GetSpeedLevel = 1026,
  SwitchJoystick = 1027,
  Pose = 1028,
  Scrape = 1029,
  FrontFlip = 1030,
  FrontJump = 1031,
  FrontPounce = 1032,
  WiggleHips = 1033,
  GetState = 1034,
  EconomicGait = 1035,
  FingerHeart = 1036,
};

}  // namespace go2_driver

#endif // GO2_DRIVER__GO2_API_ID_HPP
