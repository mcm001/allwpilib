/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
#include <frc/system/plant/ElevatorSystem.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/units.h>
#include <wpi/math>

/**
 * This is a sample program to demonstrate how to use a state-space controller
 * to control an elevator.
 */
class Robot : public frc::TimedRobot {
  const int kMotorPort = 0;
  const int kEncoderAChannel = 0;
  const int kEncoderBChannel = 1;
  const int kJoystickPort = 0;

  const units::meter_t kRaisedPosition = 2_ft;
  const units::meter_t kLoweredPosition = 0_ft;

  const units::meter_t kDrumRadius = 0.75_in;

  /*
  The plant holds a state-space model of our flywheel. In this system the states
  are as follows: States: [velocity], in RPM. Inputs (what we can "put in"):
  [voltage], in volts. Outputs (what we can measure): [velocity], in RPM.
   */
  frc::LinearSystem<2, 1, 1> m_elevatorPlant = [this] {
    auto motors = frc::DCMotor::NEO(2);

    // carriage mass
    auto m = 4.5_kg;

    // reduction between motors and encoder,
    // as output over input. If the elevator spins slower than the motors, this
    // number should be greater than one.
    auto G = 6.0;

    return frc::ElevatorSystem(motors, m, kDrumRadius, G, 12_V);
  }();

  // The observer fuses our encoder data and voltage inputs to reject noise.
  frc::KalmanFilter<2, 1, 1> m_observer{
      m_elevatorPlant,
      {0.0508, 0.5},  // How accurate we think our model is
      {0.001},        // How accurate we think our encoder position
      // data is. In this case we very highly trust our encoder position
      // reading.
      20_ms};

  // The LQR combines feedback and model-based feedforward to create voltage
  // commands.
  frc::LinearQuadraticRegulator<2, 1> m_controller{
      m_elevatorPlant,
      {0.0254, 0.254},  // qelms.
      // Position and velocity error tolerances, in radians and radians per
      // second. Decrease this to more heavily penalize state excursion, or make
      // the controller behave more aggressively. In this example we weight
      // position much more highly than velocity, but this can be tuned to
      // balance the two.
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
      20_ms};  // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer and plant for easy
  // control.
  frc::LinearSystemLoop<2, 1, 1> m_loop{m_elevatorPlant, m_controller,
                                        m_observer};

  // An encoder set up to measure flywheel velocity in radians per second.
  frc::Encoder m_encoder{kEncoderAChannel, kEncoderBChannel};

  frc::PWMVictorSPX m_motor{kMotorPort};
  frc::XboxController m_joystick{kJoystickPort};

  frc::TrapezoidProfile<units::meters>::Constraints m_constraints{3_fps,
                                                                  6_fps_sq};

  frc::TrapezoidProfile<units::meters>::State m_lastProfiledReference;

 public:
  void RobotInit() {
    // Circumference = pi * d, so distance per click = pi * d / counts
    m_encoder.SetDistancePerPulse(2.0 * wpi::math::pi *
                                  kDrumRadius.to<double>() / 4096.0);

    // reset our loop to make sure it's in a known state.
    m_loop.Reset();
  }

  void TeleopInit() {
    m_lastProfiledReference = {units::meter_t(m_encoder.GetDistance()),
                               units::meters_per_second_t(m_encoder.GetRate())};
  }

  void TeleopPeriodic() {
    // Sets the target speed of our flywheel. This is similar to setting the
    // setpoint of a PID controller.
    frc::TrapezoidProfile<units::meters>::State goal;
    if (m_joystick.GetBumper(frc::GenericHID::kRightHand)) {
      // we pressed the bumper, so let's set our next reference
      goal = {kRaisedPosition, 0_fps};
    } else {
      // we released the bumper, so let's spin down
      goal = {kLoweredPosition, 0_fps};
    }
    m_lastProfiledReference =
        (frc::TrapezoidProfile<units::meters>(m_constraints, goal,
                                              m_lastProfiledReference))
            .Calculate(20_ms);

    m_loop.SetNextR(
        frc::MakeMatrix<2, 1>(m_lastProfiledReference.position.to<double>(),
                              m_lastProfiledReference.velocity.to<double>()));

    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.Correct(frc::MakeMatrix<1, 1>(m_encoder.GetDistance()));

    // Update our LQR to generate new voltage commands and use the voltages to
    // predict the next state with out Kalman filter.
    m_loop.Predict(20_ms);

    // send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    m_motor.SetVoltage(units::volt_t(m_loop.U(0)));
  }
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
