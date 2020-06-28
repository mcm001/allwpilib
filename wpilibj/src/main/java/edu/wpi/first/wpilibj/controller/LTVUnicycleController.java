/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.Vector;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N3;

/**
 * A Linear Time-Varying Cascaded Unicycle Controller for differential drive
 * robots. Similar to RAMSETE, this controller combines feedback and feedforward
 * to output ChassisSpeeds to guide a robot along a trajectory. However, this
 * controller utilizes tolerances grounded in reality to pick gains rather than
 * magical Beta and Zeta gains.
 */

@SuppressWarnings("MemberName")
public class LTVUnicycleController {
  private final Matrix<N3, N2> m_B;

  private final Matrix<N3, N1> m_qElms;
  private final Matrix<N2, N1> m_rElms;

  private final double m_dt;

  Pose2d m_poseError;
  Pose2d m_poseTolerance;

  /**
   * Construct a LTV Unicycle Controller.
   *
   * @param qElms     The maximum desired error tolerance for the robot's state, in
   *                  the form [X, Y, Heading]^T. Units are meters and radians.
   * @param rElms     The maximum desired control effort by the feedback controller,
   *                  in the form [vMax, wMax]^T. Units are meters per second and
   *                  radians per second. Note that this is not the maximum speed of
   *                  the robot, but rather the maximum effort the feedback controller
   *                  should apply on top of the trajectory feedforward.
   * @param dtSeconds The nominal dt of this controller. With command based this is 0.020.
   */
  @SuppressWarnings({"ParameterName", "LocalVariableName"})
  public LTVUnicycleController(Vector<N3> qElms, Vector<N2> rElms, double dtSeconds) {
    this(qElms, 1.0, rElms, dtSeconds);
  }

  /**
   * Construct a LTV Unicycle Controller.
   *
   * @param qElms     The maximum desired error tolerance for the robot's state, in
   *                  the form [X, Y, Heading]^T. Units are meters and radians.
   * @param rho       A weighting factor that balances control effort and state excursion.
   *                  Greater values penalize state excursion more heavily.
   *                  1 is a good starting value.
   * @param rElms     The maximum desired control effort by the feedback controller,
   *                  in the form [vMax, wMax]^T. Units are meters per second and
   *                  radians per second. Note that this is not the maximum speed of
   *                  the robot, but rather the maximum effort the feedback controller
   *                  should apply on top of the trajectory feedforward.
   * @param dtSeconds The nominal dt of this controller. With command based this is 0.020.
   */
  @SuppressWarnings({"ParameterName", "LocalVariableName"})
  public LTVUnicycleController(
        Matrix<N3, N1> qElms,
        double rho,
        Matrix<N2, N1> rElms,
        double dtSeconds) {
    m_dt = dtSeconds;
    m_B = new MatBuilder<>(Nat.N3(), Nat.N2()).fill(1, 0, 0, 0, 0, 1);
    
    m_qElms = qElms.times(rho);
    m_rElms = rElms;
  }

  /**
   * Returns if the controller is at the reference pose on the trajectory.
   * Note that this is different than if the robot has traversed the entire
   * trajectory. The tolerance is set by the {@link #setTolerance(Pose2d)}
   * method.
   *
   * @return If the robot is within the specified tolerance of the
   */
  public boolean atReference() {
    var translationError = m_poseError.getTranslation();
    var rotationError = m_poseError.getRotation();
    var tolTranslate = m_poseTolerance.getTranslation();
    var tolRotate = m_poseTolerance.getRotation();
    return Math.abs(translationError.getX()) < tolTranslate.getX()
      && Math.abs(translationError.getY()) < tolTranslate.getY()
      && Math.abs(rotationError.getRadians()) < tolRotate.getRadians();
  }

  /**
   * Set the tolerance for if the robot is {@link #atReference()} or not.
   *
   * @param poseTolerance The new pose tolerance.
   */
  public void setTolerance(final Pose2d poseTolerance) {
    this.m_poseTolerance = poseTolerance;
  }

  /**
   * Returns the next output of the controller.
   *
   * <p>The reference pose, linear velocity, and angular velocity should come
   * from a {@link Trajectory}.
   *
   * @param currentPose                       The current position of the robot.
   * @param currentLinearVelocityMetersPerSec The current linear velocity of the robot.
   *                                          this can be determined by averaging the
   *                                          measured left and right wheel velocities.
   * @param poseRef                           The desired pose of the robot.
   * @param linearVelocityRefMetersPerSec     The desired linear velocity of the robot.
   * @param angularVelocityRefRadPerSec       The desired angular velocity of the robot.
   * @return The next calculated output.
   */
  @SuppressWarnings("LocalVariableName")
  public ChassisSpeeds calculate(Pose2d currentPose,
                                 double currentLinearVelocityMetersPerSec,
                                 Pose2d poseRef,
                                 double linearVelocityRefMetersPerSec,
                                 double angularVelocityRefRadPerSec) {
    m_poseError = poseRef.relativeTo(currentPose);

    if (currentLinearVelocityMetersPerSec < 1e-9) {
      currentLinearVelocityMetersPerSec = 1e-9;
    }

    var A = new MatBuilder<>(Nat.N3(), Nat.N3())
            .fill(0, 0, 0, 0, 0, currentLinearVelocityMetersPerSec, 0, 0, 0);
    
    var K = new LinearQuadraticRegulator<N3, N2, N2>(A, m_B, m_qElms, m_rElms, m_dt).getK();

    var error = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
          m_poseError.getTranslation().getX(),
          m_poseError.getTranslation().getY(),
          m_poseError.getRotation().getRadians());

    var u = K.times(error);

    return new ChassisSpeeds(
      linearVelocityRefMetersPerSec + u.get(0, 0),
      0.0,
      angularVelocityRefRadPerSec + u.get(1, 0));
  }

  /**
   * Returns the next output of the controller.
   *
   * <p>The reference pose and desired state should come from a {@link Trajectory}.
   *
   * @param currentPose  The current pose.
   * @param desiredState The desired pose, linear velocity, and angular velocity
   *                     from a trajectory.
   * @return The calculated {@link ChassisSpeeds}.
   */
  public ChassisSpeeds calculate(Pose2d currentPose,
      double currentLinearVelocityMetersPerSec,
      Trajectory.State desiredState) {
    return calculate(currentPose, currentLinearVelocityMetersPerSec,
            desiredState.poseMeters,
            desiredState.velocityMetersPerSecond,
            desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter);
  }

}
