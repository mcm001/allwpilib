/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.trajectory.constraint;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A class that enforces constraints on differential drive velocity based on
 * a differential drive {@link LinearSystem} and the drive kinematics.
 *
 * <p>Ensures that the acceleration of any wheel of the robot while
 * following the trajectory is never higher than what can be achieved with
 * the given maximum voltage.
 */
@SuppressWarnings({"ParameterName", "LocalVariableName", "MemberName"})
public class DifferentialDriveVelocitySystemConstraint implements TrajectoryConstraint {
  private final LinearSystem<N2, N2, N2> m_system;
  private final DifferentialDriveKinematics m_kinematics;
  private final double m_maxVoltage;

  /**
   * Creates a new DifferentialDriveVelocitySystemConstraint.
   *
   * @param system     A {@link LinearSystem} representing the drivetrain..
   * @param kinematics A kinematics component describing the drive geometry.
   * @param maxVoltage The maximum voltage available to the motors while following the path.
   *                   Should be somewhat less than the nominal battery voltage (12V) to account
   *                   for "voltage sag" due to current draw.
   */
  public DifferentialDriveVelocitySystemConstraint(LinearSystem<N2, N2, N2> system,
                                                   DifferentialDriveKinematics kinematics,
                                                   double maxVoltage) {
    m_system = requireNonNullParam(system, "system",
      "DifferentialDriveVoltageConstraint");
    m_kinematics = requireNonNullParam(kinematics, "kinematics",
      "DifferentialDriveVoltageConstraint");
    m_maxVoltage = maxVoltage;
  }

  @Override
  public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter,
                                              double velocityMetersPerSecond) {
    // Calculate wheel velocity states from current velocity and curvature
    var speeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(velocityMetersPerSecond, 0, curvatureRadPerMeter));

    var x = VecBuilder.fill(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);

    // If either wheel velocity is greater than its maximum, normalize the wheel
    // speeds to within an achievable range while maintaining the curvature
    x = StateSpaceUtil.normalizeInputVector(x, velocityMetersPerSecond);

    // chassis speed is average of wheel speeds
    return (x.get(0, 0) + x.get(1, 0)) / 2.0;
  }

  @Override
  public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters,
                                                       double curvatureRadPerMeter,
                                                       double velocityMetersPerSecond) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(velocityMetersPerSecond, 0,
        velocityMetersPerSecond
        * curvatureRadPerMeter));

    var x = VecBuilder.fill(wheelSpeeds.leftMetersPerSecond,
        wheelSpeeds.rightMetersPerSecond);

    Matrix<N2, N1> u;

    // dx/dt for minimum u
    u = VecBuilder.fill(-m_maxVoltage, -m_maxVoltage);
    double minAccel = getChassisAccel(x, u);

    // dx/dt for maximum u
    u = VecBuilder.fill(m_maxVoltage, m_maxVoltage);
    double maxAccel = getChassisAccel(x, u);

    return new MinMax(minAccel, maxAccel);
  }

  public double getChassisAccel(Matrix<N2, N1> x, Matrix<N2, N1> u) {
    var xDot = m_system.getA().times(x).plus(m_system.getB().times(u));
    return (xDot.get(0, 0) + xDot.get(1, 0)) / 2.0;
  }
}
