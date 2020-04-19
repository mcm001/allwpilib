/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.examples.statespaceelevator;

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
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

/**
 * This is a sample program to demonstrate how to use a state-space controller
 * to control an elevator.
 */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;
  private static final int kJoystickPort = 0;
  private static final double kHighGoalPosition = Units.feetToMeters(3);
  private static final double kLowGoalPosition = Units.feetToMeters(0);

  private static final double kCarriageMass = 4.5; // kilograms
  private static final double kDrumRadius = 1.5 / 2.0 * 25.4 / 1000.0; // a 1.5in diameter drum
  // has a radius of 0.75in, or 0.019in.
  private static final double kElevatorGearing = 6.0; // reduction between motors and encoder,
  // as output over input. If the elevator spins slower than the motors, this number should be
  // greater than one.

  private TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
        Units.feetToMeters(3.0), Units.feetToMeters(6.0)); // Max elevator speed and acceleration.
  private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

  /*
  The plant holds a state-space model of our flywheel. In this system the states are as follows:
  States: [velocity], in RPM.
  Inputs (what we can "put in"): [voltage], in volts.
  Outputs (what we can measure): [velocity], in RPM.
   */
  private final LinearSystem<N2, N1, N1> m_elevatorPlant = LinearSystem.createElevatorSystem(
        DCMotor.getNEO(2),
        kCarriageMass,
        kDrumRadius,
        kElevatorGearing,
        12.0);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
        Nat.N2(), Nat.N1(),
        m_elevatorPlant,
        VecBuilder.fill(Units.inchesToMeters(2), Units.inchesToMeters(40)), // How accurate we
        // think our model is, in meters and meters/second.
        VecBuilder.fill(0.001), // How accurate we think our encoder position
        // data is. In this case we very highly trust our encoder position reading.
        0.020);

  // The LQR combines feedback and model-based feedforward to create voltage commands.
  private final LinearQuadraticRegulator<N2, N1, N1> m_controller
        = new LinearQuadraticRegulator<>(m_elevatorPlant,
        VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(10.0)), // qelms. Position
        // and velocity error tolerances, in meters and meters per second. Decrease this to more
        // heavily penalize state excursion, or make the controller behave more aggressively. In
        // this example we weight position much more highly than velocity, but this can be
        // tuned to balance the two.
        1.0, // rho balances Q and R, or velocity and voltage weights. Increasing this
        // will penalize state excursion more heavily, while decreasing this will penalize control
        // effort more heavily. Useful for balancing weights for systems with more states such
        // as drivetrains.
        VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
        // heavily penalize control effort, or make the controller less aggressive. 12 is a good
        // starting point because that is the (approximate) maximum voltage of a battery.
        0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer and plant for easy control.
  private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(Nat.N2(),
        m_elevatorPlant,
        m_controller,
        m_observer);

  // An encoder set up to measure flywheel velocity in radians per second.
  private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);

  private final SpeedController m_motor = new PWMVictorSPX(kMotorPort);

  private final Joystick m_joystick = new Joystick(kJoystickPort); // A joystick to read the
  // trigger from.

  @Override
  public void robotInit() {
    // Circumference = pi * d, so distance per click = pi * d / counts
    m_encoder.setDistancePerPulse(Math.PI * 2 * kDrumRadius / 4096.0);

    // reset our loop to make sure it's in a known state.
    m_loop.reset();
  }

  @Override
  public void teleopInit() {
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
      goal = new TrapezoidProfile.State(kHighGoalPosition, 0.0);
    } else {
      // Otherwise, we go to the low goal
      goal = new TrapezoidProfile.State(kLowGoalPosition, 0.0);
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
