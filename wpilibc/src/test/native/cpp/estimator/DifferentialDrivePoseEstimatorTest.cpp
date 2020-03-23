/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <limits>
#include <random>

#include <units/units.h>

#include "frc/StateSpaceUtils.h"
#include "frc/estimator/DifferentialDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc2/Timer.h"
#include "gtest/gtest.h"

TEST(DifferentialDrivePoseEstimatorTest, TestAccuracy) {
  frc::DifferentialDrivePoseEstimator estimator{
      frc::Rotation2d(), frc::Pose2d(), frc::MakeMatrix<3, 1>(0.02, 0.02, 0.01),
      frc::MakeMatrix<3, 1>(0.1, 0.1, 0.01)};

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      std::vector{frc::Pose2d(), frc::Pose2d(20_m, 20_m, frc::Rotation2d()),
                  frc::Pose2d(23_m, 23_m, frc::Rotation2d(173_deg)),
                  frc::Pose2d(54_m, 54_m, frc::Rotation2d())},
      frc::TrajectoryConfig(0.5_mps, 2.0_mps_sq));

  frc::DifferentialDriveKinematics kinematics{1.0_m};

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, 1.0);

  units::second_t dt = 0.01_s;
  units::second_t t = 0.0_s;

  units::second_t kVisionUpdateRate = 0.1_s;
  frc::Pose2d lastVisionPose;
  units::second_t lastVisionUpdateRealTimestamp{
      -std::numeric_limits<double>::max()};
  units::second_t lastVisionUpdateTime{-std::numeric_limits<double>::max()};

  double maxError = -std::numeric_limits<double>::max() double errorSum = 0;

  while (t <= trajectory.TotalTime()) {
    auto groundTruthState = trajectory.Sample(t);
    auto input = kinematics.ToWheelSpeeds(frc::ChassisSpeeds(
        groundTruthState.velocity, 0_mps,
        groundTruthState.velocity * groundTruthState.curvature));

    if (lastVisionUpdateTime + kVisionUpdateRate < t) {
      if (lastVisionPose != frc::Pose2d()) {
        estimator.AddVisionMeasurement(lastVisionPose)
      }
      lastVisionPose =
          groundTruthState.pose +
          frc::Transform2d(
              frc::Translation2d(distribution(generator) * 0.1 * 1_m,
                                 distribution(generator) * 0.1 * 1_m),
              frc::Rotation2d(distribution(generator) * 0.1 * 1_rad));

      lastVisionUpdateRealTimestamp = frc2::Timer::GetFPGATimestamp();
      lastVisionUpdateTime = t;
    }

    auto xhat = estimator.Update(
        groundTruthState.pose.Rotation() +
            frc::Rotation2d(units::radian_t(distribution(generator) * 0.1)),
        input.left * dt + distribution(generator) * 0.1,
        input.right * dt + distribution(generator) * 0.1);

    double error = groundTruthState.pose.Translation()
                       .Distance(xHat.Translation())
                       .to<double>();
    if (error > maxError) {
      maxError = error;
    }
    errorSum += error;

    t += dt;
  }

  std::cout << "Mean error (m): "
            << errorSum /
                   (trajectory.TotalTime().to<double>() / dt.to<double>())
            << std::endl;
  std::cout << "Max error (m):" << maxError << std::endl;
}
