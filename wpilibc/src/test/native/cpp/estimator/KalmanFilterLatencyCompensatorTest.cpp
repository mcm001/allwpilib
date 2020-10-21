/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/StateSpaceUtil.h"
#include "frc/estimator/KalmanFilterLatencyCompensator.h"
#include "frc/estimator/UnscentedKalmanFilter.h"
#include "frc/system/plant/LinearSystemId.h"
#include "gtest/gtest.h"

TEST(KalmanFilterLatencyCompensatorTest, TestAccuracy) {
  frc::KalmanFilterLatencyCompensator<1, 1, 1,
                                      frc::UnscentedKalmanFilter<1, 1, 1>>
      comp{};
  auto system = frc::LinearSystemId::IdentifyVelocitySystem(1.0, 1.0);
  frc::UnscentedKalmanFilter<1, 1, 1> kf{
      [&](const Eigen::Matrix<double, 1, 1>& x,
          const Eigen::Matrix<double, 1, 1>& u) {
        return system.A() * x + system.B() * u;
      },
      [&](const Eigen::Matrix<double, 1, 1>& x,
          const Eigen::Matrix<double, 1, 1>& u) { return x; },
      {1},
      {1},
      1_s};
  comp.AddObserverState(kf, frc::MakeMatrix<1, 1>(1.0),
                        frc::MakeMatrix<1, 1>(0.0), 0.0_s);
  comp.AddObserverState(kf, frc::MakeMatrix<1, 1>(1.0),
                        frc::MakeMatrix<1, 1>(1.0), 1_s);
  comp.AddObserverState(kf, frc::MakeMatrix<1, 1>(1.0),
                        frc::MakeMatrix<1, 1>(3.0), 2_s);
  comp.ApplyPastMeasurement<1>(
      &kf, 1_s, frc::MakeMatrix<1, 1>(2.0),
      [&](const Eigen::Matrix<double, 1, 1>& u,
          const Eigen::Matrix<double, 1, 1>& y) { kf.Correct(u, y); },
      1.5_s);
}
