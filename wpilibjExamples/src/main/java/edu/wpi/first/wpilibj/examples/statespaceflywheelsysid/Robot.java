/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.examples.statespaceflywheelsysid;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;

/**
 * This is a sample program to demonstrate how to use a state-space controller
 * to control a flywheel.
 */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;
  private static final int kJoystickPort = 0;
  private static final double kSpinupRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(500.0);

  private static final double flywheelKv = 0.023; // kv, volts per radian per second
  private static final double flywheelKa = 0.001; // ka, volts per radian per second squared

  /*
  The plant holds a state-space model of our flywheel. In this system the states are as follows:
  States: [velocity], in RPM.
  Inputs (what we can "put in"): [voltage], in volts.
  Outputs (what we can measure): [velocity], in RPM.
  The kV and kA constants are found using the FRC Characterization toolsuite.
   */
  private final LinearSystem<N1, N1, N1> m_flywheelPlant = LinearSystemId.identifyVelocitySystem(
        flywheelKv, flywheelKa);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
        Nat.N1(), Nat.N1(),
        m_flywheelPlant,
        VecBuilder.fill(3.0), // How accurate we think our model is
        VecBuilder.fill(0.01), // How accurate we think our encoder
        // data is
        0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller
        = new LinearQuadraticRegulator<>(m_flywheelPlant,
        VecBuilder.fill(8.0), // Velocity error tolerance
        VecBuilder.fill(12.0), // Control effort (voltage) tolerance
        0.020);

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(
      m_flywheelPlant,
        m_controller,
        m_observer,
        12.0,
        0.020);

  // An encoder set up to measure flywheel velocity in radians per second.
  private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);

  private final SpeedController m_motor = new PWMVictorSPX(kMotorPort);

  private final Joystick m_joystick = new Joystick(kJoystickPort); // A joystick to read the
  // trigger from.

  @Override
  public void robotInit() {
    // we go 2 pi radians per 4096 clicks.
    m_encoder.setDistancePerPulse(
          2.0 * Math.PI / 4096.0);
  }

  @Override
  public void teleopInit() {
    // reset our loop to make sure it's in a known state.
    m_loop.reset(VecBuilder.fill(m_encoder.getRate()));
  }

  @Override
  public void teleopPeriodic() {

    // Sets the target speed of our flywheel. This is similar to setting the setpoint of a
    // PID controller.
    if (m_joystick.getTriggerPressed()) {
      // we just pressed the trigger, so let's set our next reference
      m_loop.setNextR(VecBuilder.fill(kSpinupRadPerSec));
    } else if (m_joystick.getTriggerReleased()) {
      // we just released the trigger, so let's spin down
      m_loop.setNextR(VecBuilder.fill(0.0));
    }

    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(m_encoder.getRate()));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    m_loop.predict(0.020);

    // send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    double currentBatteryVoltage = RobotController.getBatteryVoltage();
    m_motor.set(nextVoltage / currentBatteryVoltage);
  }
}
