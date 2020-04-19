/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/DriverStation.h>
#include <frc/Encoder.h>
#include <frc/GenericHID.h>
#include <frc/PWMVictorSPX.h>
#include <frc/StateSpaceUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/units.h>
#include <wpi/math>

/**
 * This is a sample program to demonstrate how to use a state-space controller
 * to control a flywheel.
 */
class Robot : public frc::TimedRobot {
  const int kMotorPort = 0;
  const int kEncoderAChannel = 0;
  const int kEncoderBChannel = 1;
  const int kJoystickPort = 0;
  const units::radians_per_second_t kSpinupRadPerSec = 500_rpm;

  const double flywheelKv = 0.023;  // kv, volts per radian per second
  const double flywheelKa = 0.001;  // ka, volts per radian per second squared

  /*
  The plant holds a state-space model of our flywheel. In this system the states
  are as follows: States: [velocity], in RPM. Inputs (what we can "put in"):
  [voltage], in volts. Outputs (what we can measure): [velocity], in RPM.
   */
  frc::LinearSystem<1, 1, 1> m_armPlant =
      frc::IdentifyVelocitySystem(flywheelKv, flywheelKa);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  frc::KalmanFilter<1, 1, 1> m_observer{
      m_armPlant,
      {3.0},   // How accurate we think our model is
      {0.01},  // How accurate we think our encoder data is
      20_ms};

  // The LQR combines feedback and model-based feedforward to create voltage
  // commands.
  frc::LinearQuadraticRegulator<1, 1> m_controller{
      m_armPlant,
      {8.0},  // qelms. Velocity error tolerance, in radians per second.
              // Decrease
      // this to more heavily penalize state excursion, or make the controller
      // behave more aggressively.
      1.0,  // rho balances Q and R, or velocity and voltage weights. Increasing
            // this
      // will penalize state excursion more heavily, while decreasing this will
      // penalize control effort more heavily. Useful for balancing weights for
      // systems with more states such as drivetrains.
      {12.0},  // relms. Control effort (voltage) tolerance. Decrease this to
               // more
      // heavily penalize control effort, or make the controller less
      // aggressive. 12 is a good starting point because that is the
      // (approximate) maximum voltage of a battery.
      20_ms};  // Nominal time between loops. 20ms for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer and plant for easy
  // control.
  frc::LinearSystemLoop<1, 1, 1> m_loop{m_armPlant, m_controller, m_observer};

  // An encoder set up to measure flywheel velocity in radians per second.
  frc::Encoder m_encoder{kEncoderAChannel, kEncoderBChannel};

  frc::PWMVictorSPX m_motor{kMotorPort};
  frc::XboxController m_joystick{kJoystickPort};

 public:
  void RobotInit() {
    // we go 2 pi radians per 4096 clicks.
    m_encoder.SetDistancePerPulse(2.0 * wpi::math::pi / 4096.0);

    // reset our loop to make sure it's in a known state.
    m_loop.Reset();
  }

  void TeleopPeriodic() {
    // Sets the target speed of our flywheel. This is similar to setting the
    // setpoint of a PID controller.
    if (m_joystick.GetBumper(frc::GenericHID::kRightHand)) {
      // we pressed the bumper, so let's set our next reference
      m_loop.SetNextR(frc::MakeMatrix<1, 1>(kSpinupRadPerSec.to<double>()));
    } else {
      // we released the bumper, so let's spin down
      m_loop.SetNextR(frc::MakeMatrix<1, 1>(0.0));
    }

    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.Correct(frc::MakeMatrix<1, 1>(m_encoder.GetRate()));

    // Update our LQR to generate new voltage commands and use the voltages to
    // predict the next state with out Kalman filter.
    m_loop.Predict(20_ms);

    // send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.U(0);
    m_motor.SetVoltage(units::volt_t(m_loop.U(0)));
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
