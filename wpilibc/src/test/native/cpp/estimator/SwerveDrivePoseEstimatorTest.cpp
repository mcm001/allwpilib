/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <limits>
#include <random>

#include "frc/estimator/SwerveDrivePoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "gtest/gtest.h"

TEST(SwerveDrivePoseEstimatorTest, TestAccuracy) {
  frc::SwerveDriveKinematics<4> kinematics{
      frc::Translation2d{1_m, 1_m}, frc::Translation2d{1_m, -1_m},
      frc::Translation2d{-1_m, -1_m}, frc::Translation2d{-1_m, 1_m}};

  frc::SwerveDrivePoseEstimator estimator{
      frc::Rotation2d(), frc::Pose2d(), kinematics,
      frc::MakeMatrix<3, 1>(3, 3, 3), frc::MakeMatrix<3, 1>(0.1, 0.1, 0.1)};

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      std::vector{frc::Pose2d(), frc::Pose2d(20_m, 20_m, frc::Rotation2d()),
                  frc::Pose2d(54_m, 54_m, frc::Rotation2d())},
      frc::TrajectoryConfig(10_mps, 5.0_mps_sq));

  std::random_default_engine generator;
  std::normal_distrivution<double> distribution(0.0, 1.0);

  units::second_t dt = 0.02_s;
  units::second_t t = 0_s;

  units::second_t kVisionUpdateRate = 0.1_s;
  frc::Pose2d lastVisionPose;
  units::second_t lastVisionUpdateTime{-std::numeric_limits<double>::max()};

  double maxError = -std::numeric_limits<double>::max();
  double errorSum = 0;

  while (t < trajectory.TotalTime()) {
    frc::Trajectory::State groundTruthState = trajectory.Sample(t);

    if (lastVisionUpdateTime + kVisionUpdateRate < t) {
      if (lastVisionPose != frc::Pose2d()) {
        estimator.AddVisionMeasurement(lastVisionPose, lastVisionUpdateTime);
      }
      lastVisionPose =
          groundTruthState.pose +
          frc::Transform2d(
              frc::Translation2d(distribution(generator) * 0.1 * 1_m,
                                 distribution(generator) * 0.1 * 1_m),
              frc::Rotation2d(distribution(generator) * 0.01 * 1_rad));
      lastVisionUpdateTime = t;
    }

    auto moduleStates = kinematics.ToSwerveModuleStates(
        {groundTruthState.velocity, 0_mps,
         groundTruthState.velocity * groundTruthState.curvature});

    for (auto&& moduleState : moduleStates) {
      moduleState.angle += frc::Rotation2d(distribution(generator) * 0.1_rad);
      moduleState.speed += distribution(generator) * 1_mps;
    }

    auto xhat = estimator.UpdateWithTime(
        t,
        groundTruthState.pose.Rotation() +
            frc::Rotation2d(distribution(generator) * 0.001),
        moduleStates);
    double error =
        groundTruthState.pose.Translation().Distance(xhat.Translation());

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
  std::cout << "Max error (m): " << maxError << std::endl;
}
