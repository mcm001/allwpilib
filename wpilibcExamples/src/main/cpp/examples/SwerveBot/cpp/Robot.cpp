/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

#include "SwerveDrive.h"

class Robot : public frc::TimedRobot {
 public:
  void AutonomousPeriodic() override { TeleopPeriodic(); }

  void TeleopPeriodic() override {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const auto xSpeed =
        -m_controller.GetY(frc::GenericHID::kLeftHand) * SwerveDrive::kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const auto ySpeed =
        -m_controller.GetX(frc::GenericHID::kLeftHand) * SwerveDrive::kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const auto rot = -m_controller.GetX(frc::GenericHID::kRightHand) *
                     SwerveDrive::kMaxAngularSpeed;

    m_swerve.Drive(xSpeed, ySpeed, rot);
    m_swerve.UpdateOdometry();
  }

 private:
  frc::XboxController m_controller{0};
  SwerveDrive m_swerve;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
