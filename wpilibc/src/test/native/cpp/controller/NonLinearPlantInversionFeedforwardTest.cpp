/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Core>
#include <units/units.h>

#include "frc/controller/NonLinearPlantInversionFeedforward.h"

namespace frc {

Vector<2> Dynamics(const Vector<2>& x, const Vector<1>& u) {
  Eigen::Matrix<double, 2, 1> result;

  result = (frc::MakeMatrix<2, 2>(1.0, 0.0, 0.0, 1.0) * x) +
           (frc::MakeMatrix<2, 1>(0.0, 1.0) * u);

  return result;
}

TEST(NonLinearPlantInversionFeedforwardTest, Calculate) {
  std::function<Eigen::Matrix<double, 2, 1>(const Eigen::Matrix<double, 2, 1>&,
                                            const Eigen::Matrix<double, 1, 1>&)>
      modelDynamics = [this](auto& x, auto& u) { return Dynamics(x, u); };

  frc::NonLinearPlantInversionFeedforward<2, 1> feedforward{
      modelDynamics, units::second_t(0.02)};

  Eigen::Matrix<double, 2, 1> r;
  r << 2, 2;
  Eigen::Matrix<double, 2, 1> nextR;
  nextR << 3, 3;

  EXPECT_NEAR(48, feedforward.Calculate(r, nextR)(0, 0), 1e-6);
}

}  // namespace frc
