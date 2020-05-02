/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.trajectory.constraint;

import org.ejml.dense.row.CommonOps_DDRM;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
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
   * @param system      A {@link LinearSystem} representing the drivetrain..
   * @param kinematics  A kinematics component describing the drive geometry.
   * @param maxVoltage  The maximum voltage available to the motors while following the path.
   *                    Should be somewhat less than the nominal battery voltage (12V) to account
   *                    for "voltage sag" due to current draw.
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

    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(velocityMetersPerSecond, 0,
                                                    velocityMetersPerSecond
                                                    * curvatureRadPerMeter));

    var x = new MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(wheelSpeeds.leftMetersPerSecond,
        wheelSpeeds.rightMetersPerSecond);

    // Normalize wheel velocities to be in achievable range while maintaining curvature
    if ((Math.abs(x.get(0, 0)) > velocityMetersPerSecond)
        || (Math.abs(x.get(1, 0)) > velocityMetersPerSecond)) {
      x.times(velocityMetersPerSecond / CommonOps_DDRM.elementMaxAbs(x.getStorage().getDDRM()));
    }

    return (x.get(0, 0) + x.get(1, 0)) / 2.0;
  }

  @Override
  public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters,
                                                       double curvatureRadPerMeter,
                                                       double velocityMetersPerSecond) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(velocityMetersPerSecond, 0,
                                                                   velocityMetersPerSecond
                                                                       * curvatureRadPerMeter));

    var x = new MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(wheelSpeeds.leftMetersPerSecond,
        wheelSpeeds.rightMetersPerSecond);

    Matrix<N2, N1> xDot;
    Matrix<N2, N1> u;

    // dx/dt for minimum u
    u = new MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(-m_maxVoltage, -m_maxVoltage);
    xDot = m_system.getA().times(x).plus(m_system.getB().times(u));
    double minAccel = (xDot.get(0, 0) + xDot.get(1, 0)) / 2.0;

    // dx/dt for maximum u
    u = new MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(m_maxVoltage, m_maxVoltage);
    xDot = m_system.getA().times(x).plus(m_system.getB().times(u));
    double maxAccel = (xDot.get(0, 0) + xDot.get(1, 0)) / 2.0;

    // When turning about a point inside of the wheelbase (i.e. radius less than half
    // the trackwidth), the inner wheel's direction changes, but the magnitude remains
    // the same.  The formula above changes sign for the inner wheel when this happens.
    // We can accurately account for this by simply negating the inner wheel.

    if ((m_kinematics.trackWidthMeters / 2) > (1 / Math.abs(curvatureRadPerMeter))) {
      if (velocityMetersPerSecond > 0) {
        minAccel = -minAccel;
      } else if (velocityMetersPerSecond < 0) {
        maxAccel = -maxAccel;
      }
    }

    return new MinMax(minAccel, maxAccel);
  }
}
