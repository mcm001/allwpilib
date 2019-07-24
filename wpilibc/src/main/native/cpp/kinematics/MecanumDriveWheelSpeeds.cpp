/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/kinematics/MecanumDriveWheelSpeeds.h"

#include <algorithm>
#include <array>
#include <cmath>

using namespace frc;

void MecanumDriveWheelSpeeds::Normalize(double attainableMaxSpeed) {
  std::array<double, 4> wheelSpeeds{frontLeft, frontRight, rearLeft, rearRight};
  double realMaxSpeed = *std::max_element(
      wheelSpeeds.begin(), wheelSpeeds.end(),
      [](const auto& a, const auto& b) { return std::abs(a) < std::abs(b); });

  if (realMaxSpeed > attainableMaxSpeed) {
    for (int i = 0; i < 4; ++i) {
      wheelSpeeds[i] = wheelSpeeds[i] / realMaxSpeed * attainableMaxSpeed;
    }
    frontLeft = wheelSpeeds[0];
    frontRight = wheelSpeeds[1];
    rearLeft = wheelSpeeds[2];
    rearRight = wheelSpeeds[3];
  }
}
