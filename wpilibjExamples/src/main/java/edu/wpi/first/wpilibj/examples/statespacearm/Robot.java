/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.examples.statespacearm;

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
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * This is a sample program to demonstrate how to use a state-space controller
 * to control an arm.
 */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;
  private static final int kJoystickPort = 0;
  private static final double kRaisedPosition = Units.degreesToRadians(90.0);
  private static final double kLoweredPosition = Units.degreesToRadians(0.0);

  private static final double kArmMOI = 1.2; // Moment of inertia of the arm, in kg * m^2. Can be
  // estimated with CAD. If finding this constant is difficult, LinearSystem.identifyPositionSystem
  // may be better.
  private static final double kArmGearing = 10.0; // reduction between motors and encoder,
  // as output over input. If the flywheel spins slower than the motors, this number should be
  // greater than one.

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
        Units.degreesToRadians(45), Units.degreesToRadians(90)); // Max arm speed and acceleration.
  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  /*
  The plant holds a state-space model of our flywheel. In this system the states are as follows:
  States: [velocity], in RPM.
  Inputs (what we can "put in"): [voltage], in volts.
  Outputs (what we can measure): [velocity], in RPM.
   */
  private final LinearSystem<N2, N1, N1> m_flywheelPlant =
      LinearSystemId.createSingleJointedArmSystem(
        DCMotor.getNEO(2),
        kArmMOI,
        kArmGearing);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
        Nat.N2(), Nat.N1(),
        m_flywheelPlant,
        VecBuilder.fill(0.015, 0.17), // How accurate we
        // think our model is, in radians and radians/sec
        VecBuilder.fill(0.01), // How accurate we think our encoder position
        // data is. In this case we very highly trust our encoder position reading.
        0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N2, N1, N1> m_controller
        = new LinearQuadraticRegulator<>(m_flywheelPlant,
        VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)), // qelms.
        // Position and velocity error tolerances, in radians and radians per second. Decrease this
        // to more heavily penalize state excursion, or make the controller behave more
        // aggressively. In this example we weight position much more highly than velocity, but this
        // can be tuned to balance the two.
        1.0, // rho balances Q and R, or velocity and voltage weights. Increasing this
        // will penalize state excursion more heavily, while decreasing this will penalize control
        // effort more heavily. Useful for balancing weights for systems with more states such
        // as drivetrains.
        VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
        // heavily penalize control effort, or make the controller less aggressive. 12 is a good
        // starting point because that is the (approximate) maximum voltage of a battery.
        0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(
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
    // We go 2 pi radians in 1 rotation, or 4096 counts.
    m_encoder.setDistancePerPulse(Math.PI * 2 / 4096.0);
  }

  @Override
  public void teleopInit() {
    // reset our loop to make sure it's in a known state.
    m_loop.reset(VecBuilder.fill(m_encoder.getDistance(), m_encoder.getRate()));

    // Reset our last reference to the current state.
    m_lastProfiledReference = new TrapezoidProfile.State(m_encoder.getDistance(),
          m_encoder.getRate());
  }

  @Override
  public void teleopPeriodic() {
    // Sets the target speed of our flywheel. This is similar to setting the setpoint of a
    // PID controller.
    TrapezoidProfile.State goal;
    if (m_joystick.getTrigger()) {
      // the trigger is pressed, so we go to the high goal.
      goal = new TrapezoidProfile.State(kRaisedPosition, 0.0);
    } else {
      // Otherwise, we go to the low goal
      goal = new TrapezoidProfile.State(kLoweredPosition, 0.0);
    }
    // Step our TrapezoidalProfile forward 20ms and set it as our next reference
    m_lastProfiledReference = (new TrapezoidProfile(m_constraints, goal, m_lastProfiledReference))
          .calculate(0.020);
    m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);

    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(m_encoder.getDistance()));

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
