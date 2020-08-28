/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/system/LinearSystem.h>

#include "Eigen/Core"
#include "units/time.h"
#include "units/voltage.h"

namespace frc {
namespace sim {

template <int N>
using Vector = Eigen::Matrix<double, N, 1>;

class SimDifferentialDrivetrain {
 public:
   SimDifferentialDrivetrain(LinearSystem<2, 2, 2>& plant,
                             DifferentialDriveKinematics& kinematics,
                             DCMotor leftGearbox, double leftGearing,
                             DCMotor rightGearbox, double rightGearing);

   void SetInputs(units::volt_t leftVoltage, units::volt_t rightVoltage);

   void Update(units::second_t dt);

   double GetState(int state);

   Vector<10> GetState();

   Rotation2d GetHeading();

   Pose2d GetEstimatedPosition();

   units::ampere_t GetCurrentDrawAmps();

   Vector<10> Dynamics(const Vector<10>& x, const Vector<2>& u);

   class State {
    public:
        static constexpr int kX = 0;
        static constexpr int kY = 1;
        static constexpr int kHeading = 2;
        static constexpr int kLeftVelocity = 3;
        static constexpr int kRightVelocity = 4;
        static constexpr int kLeftPosition = 5;
        static constexpr int kRightPosition = 6;
        static constexpr int kLeftVoltageError = 7;
        static constexpr int kRightVoltageError = 8;
        static constexpr int kAngularVelocityError = 9;
   };
    
 private:
   LinearSystem<2, 2, 2> m_plant; 
   units::meter_t m_rb;

   DCMotor m_leftMotor;
   DCMotor m_rightMotor;

   double m_leftGearing;
   double m_rightGearing;

   Vector<10> m_x;
   Vector<2> m_u;
};
}  // namespace sim
}  // namespace frc
