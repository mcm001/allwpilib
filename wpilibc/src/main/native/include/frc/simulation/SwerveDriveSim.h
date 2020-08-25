/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>

#include <units/force.h>
#include <units/length.h>
#include <units/mass.h>

#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/system/LinearSystem.h"
#include "frc/system/RungeKutta.h"
#include "frc/system/plant/DCMotor.h"

namespace frc {
namespace sim {

class SimSwerveModule {
 public:
  SimSwerveModule(const DCMotor& motor, double gearing,
                  units::meter_t wheelRadius, const Translation2d& position,
                  const LinearSystem<2, 1, 1>& azimuthSystem);

  Rotation2d GetAzimuthAngle() const;
  units::newton_t EstimateModuleForce(units::meters_per_second_t wheelVelocity,
                                      units::volt_t wheelVoltage) const;
  const Translation2d& GetModulePosition() const;

  Rotation2d Update(units::volt_t azimuthVoltage, units::second_t dt);

 private:
  DCMotor m_driveMotor;
  double m_gearing;
  units::meter_t m_wheelRadius;
  Translation2d m_position;

  LinearSystem<2, 1, 1> m_azimuthSystem;
  Eigen::Matrix<double, 2, 1> m_azimuthState;
};

template <int NumModules>
class SwerveDriveSim {
 public:
  template <typename... Wheels>
  SwerveDriveSim(units::kilogram_t mass,
                 const LinearSystem<2, 1, 1>& azimuthSystem,
                 const DCMotor& driveMotor, double driveMotorGearing,
                 units::meter_t wheelRadius, const Translation2d& wheel,
                 Wheels&&... wheels)
      : m_modules(GetModulesFromInfo(azimuthSystem, driveMotor,
                                     driveMotorGearing, wheelRadius, wheel,
                                     wheels...)),
        m_kinematics(GetKinematicsFromInfo(m_modules)),
        m_mass(mass) {}

  template <typename... Modules>
  SwerveDriveSim(units::kilogram_t mass, const SimSwerveModule& module,
                 Modules&&... modules)
      : SwerveDriveSim{mass, {module, modules...}} {}

  SwerveDriveSim(units::kilogram_t mass,
                 const std::array<SimSwerveModule, NumModules> modules)
      : m_modules(modules),
        m_kinematics(CreateKinematicsFromInfo(m_modules)),
        m_mass(mass) {}

  const SwerveDriveKinematics<NumModules>& GetKinematics() const {
    return m_kinematics;
  }

  const ChassisSpeeds& GetEstimatedSpeed() const { return m_estimatedSpeed; }

  std::array<SwerveModuleState, NumModules> GetEstimatedModuleStates() const {
    return m_kinematics.ToSwerveModuleStates(m_estimatedSpeed);
  }

  const std::array<SimSwerveModule, NumModules>& GetModules() const {
    return m_modules;
  }

  std::array<Rotation2d, NumModules> GetEstimatedModuleAngles() const {
    std::array<Rotation2d, NumModules> angles;
    for (size_t i = 0; i < NumModules; i++) {
      angles[i] = m_modules[i].GetAzimuthAngle();
    }
    return angles;
  }

  template <typename... Voltages>
  void SetModuleDriveVoltages(units::volt_t voltage, Voltages&&... voltages) {
    static_assert(sizeof...(voltages) == (NumModules - 1));
    SetModuleDriveVoltages({voltage, voltages...});
  }

  void SetModuleDriveVoltages(
      const std::array<units::volt_t, NumModules>& voltages) {
    m_driveVoltages = voltages;
  }

  template <typename... Voltages>
  void SetModuleAzimuthVoltages(units::volt_t voltage, Voltages&&... voltages) {
    static_assert(sizeof...(voltages) == (NumModules - 1));
    SetModuleAzimuthVoltages({voltage, voltages...});
  }

  void SetModuleAzimuthVoltages(
      const std::array<units::volt_t, NumModules>& voltages) {
    m_azimuthVoltages = voltages;
  }

  void Update(units::second_t dt) {
    // First, we determine the wheel speeds necessary to be at the current
    // chassis speeds Then, we use this speed to calculate the force each module
    // exerts on the chassis, Which we use to figure out the acceleration-ish of
    // each wheel. We then integrate this acceleration forward with RK4, and use
    // IK to go back to chassis speeds from our new wheel speeds.
    auto wheelSpeeds = m_kinematics.ToSwerveModuleStates(m_estimatedSpeed);
    std::array<SwerveModuleState, NumModules> newStates;

    for (size_t i = 0; i < NumModules; ++i) {
      auto newSpeed = RungeKutta(
          // F = ma  <==> a = F / m
          [&](double x, double u) {
            return m_modules[i]
                       .EstimateModuleForce(wheelSpeeds[i].speed,
                                            m_driveVoltages[i])
                       .template to<double>() /
                   m_mass.to<double>() * NumModules;
          },
          wheelSpeeds[i].speed.template to<double>(),
          m_driveVoltages[i].template to<double>(), dt);
      Rotation2d angle = m_modules[i].Update(m_azimuthVoltages[i], dt);
      newStates[i] =
          SwerveModuleState{units::meters_per_second_t(newSpeed), angle};
    }

    m_estimatedSpeed = m_kinematics.ToChassisSpeeds(newStates);

    // Integrate heading forward with RK4
    m_estimatedHeading = units::radian_t(RungeKutta(
        [&](double heading) {
          return m_estimatedSpeed.omega.template to<double>();
        },
        m_estimatedHeading.Radians().template to<double>(), dt));
  }

  void ResetChassisSpeeds(const ChassisSpeeds& speeds) {
    m_estimatedSpeed = speeds;
  }

 private:
  template <typename... Wheels>
  static std::array<SimSwerveModule, NumModules> GetModulesFromInfo(
      const LinearSystem<2, 1, 1>& azimuthSystem, const DCMotor& driveMotor,
      double driveMotorGearing, units::meter_t wheelRadius,
      const Translation2d& wheel, Wheels&&... wheels) {
    std::array<Translation2d, NumModules> locations{wheel, wheels...};
    std::array<SimSwerveModule, NumModules> modules;

    for (size_t i = 0; i < NumModules; ++i) {
      modules[i] = SimSwerveModule(driveMotor, driveMotorGearing, wheelRadius,
                                   locations[i], azimuthSystem);
    }

    return modules;
  }

  static SwerveDriveKinematics<NumModules> CreateKinematicsFromInfo(
      const std::array<SimSwerveModule, NumModules>& modules) {
    std::array<Translation2d, NumModules> locations;
    for (size_t i = 0; i < NumModules; ++i) {
      locations[i] = modules[i].GetModulePosition();
    }
    return SwerveDriveKinematics<4>(locations);
  }

  std::array<SimSwerveModule, NumModules> m_modules;
  SwerveDriveKinematics<NumModules> m_kinematics;

  units::kilogram_t m_mass;

  std::array<units::volt_t, NumModules> m_driveVoltages;
  std::array<units::volt_t, NumModules> m_azimuthVoltages;

  ChassisSpeeds m_estimatedSpeed;
  Rotation2d m_estimatedHeading;
};  // namespace sim

}  // namespace sim
}  // namespace frc
