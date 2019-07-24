/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/kinematics/DifferentialDriveWheelSpeeds.h"

#include <algorithm>
#include <cmath>

using namespace frc;

void DifferentialDriveWheelSpeeds::Normalize(double attainableMaxSpeed) {
  double realMaxSpeed = std::max(std::abs(left), std::abs(right));

  if (realMaxSpeed > attainableMaxSpeed) {
    left = left / realMaxSpeed * attainableMaxSpeed;
    right = right / realMaxSpeed * attainableMaxSpeed;
  }
}
