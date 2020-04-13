package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpiutil.math.*;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveMotorVoltages;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.NumericalJacobian;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N10;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N4;
import edu.wpi.first.wpiutil.math.numbers.N5;
import edu.wpi.first.wpiutil.math.numbers.N7;

/**
 * A Linear Time-Varying Differential Drive Controller for differential drive
 * robots. Similar to RAMSETE, this controller combines feedback and feedforward
 * to output Differeint to guide a robot along a trajectory. However, this
 * controller utilizes tolerances grounded in reality to pick gains rather than
 * magical Beta and Zeta gains.
 */
@SuppressWarnings({"ParameterName", "LocalVariableName", "MemberName"})
public class LTVDiffDriveController {
  private final double m_rb;
  private final LinearSystem<N2, N2, N2> m_plant;

  private Matrix<N5, N1> m_nextR;
  private Matrix<N2, N1> m_cappedU;

  private Matrix<N5, N2> m_B;
  private Matrix<N2, N5> m_K0;
  private Matrix<N2, N5> m_K1;

  private Matrix<N5, N1> m_stateError;

  private Pose2d m_poseTolerance;
  private double m_velocityTolerance;

  private DifferentialDriveKinematics m_kinematics;

  public LTVDiffDriveController(LinearSystem<N2, N2, N2> plant,
          Matrix<N5, N1> controllerQ,
          Matrix<N2, N1> controllerR,
          DifferentialDriveKinematics kinematics,
          double dtSeconds) {
    this.m_plant = plant;
    this.m_kinematics = kinematics;
    this.m_rb = kinematics.trackWidthMeters / 2.0;
    
    reset();

    var x0 = MatrixUtils.zeros(Nat.N10());
    x0.set(State.kLeftVelocity.value, 0, 1e-9);
    x0.set(State.kRightVelocity.value, 0, 1e-9);

    var x1 = MatrixUtils.zeros(Nat.N10());
    x1.set(State.kLeftVelocity.value, 0, 1);
    x1.set(State.kRightVelocity.value, 0, 1);

    var u0 = MatrixUtils.zeros(Nat.N2());

    var a0 = NumericalJacobian.numericalJacobianX(Nat.N10(), Nat.N10(), this::getDynamics, x0, u0)
            .block(Nat.N5(), Nat.N5(), new SimpleMatrixUtils.Pair<>(0, 0));
    var a1 = NumericalJacobian.numericalJacobianX(Nat.N10(), Nat.N10(), this::getDynamics, x1, u0)
            .block(Nat.N5(), Nat.N5(), new SimpleMatrixUtils.Pair<>(0, 0));

    m_B = NumericalJacobian.numericalJacobianU(Nat.N10(), Nat.N2(),
            this::getDynamics, x0, u0).block(Nat.N5(), Nat.N2(), new SimpleMatrixUtils.Pair<>(0, 0));

    m_K0 = new LinearQuadraticRegulator<N5, N2, N3>(a0, m_B,
            controllerQ, controllerR, dtSeconds).getK();
    m_K1 = new LinearQuadraticRegulator<N5, N2, N3>(a1, m_B,
            controllerQ, controllerR, dtSeconds).getK();
  }

  @SuppressWarnings("JavadocMethod")
  protected Matrix<N2, N1> getController(Matrix<N5, N1> x, Matrix<N5, N1> r) {
    // This implements the linear time-varying differential drive controller in
    // theorem 8.5.3 of https://tavsys.net/controls-in-frc.
    double kx = m_K0.get(0, 0);
    double ky0 = m_K0.get(0, 1);
    double kvpos0 = m_K0.get(0, 3);
    double kvneg0 = m_K0.get(1, 3);
    double ky1 = m_K1.get(0, 1);
    double ktheta1 = m_K1.get(0, 2);
    double kvpos1 = m_K1.get(0, 3);

    double v = (x.get(State.kLeftVelocity.value, 0) + x.get(State.kRightVelocity.value, 0)) / 2.0;
    double sqrtAbsV = Math.sqrt(Math.abs(v));

    var K = new Matrix<N2, N5>(new SimpleMatrix(2, 5));
    K.set(0, 0, kx);
    K.set(0, 1, (ky0 + (ky1 - ky0) * sqrtAbsV) * Math.signum(v));
    K.set(0, 2, ktheta1 * sqrtAbsV);
    K.set(0, 3, kvpos0 + (kvpos1 - kvpos0) * sqrtAbsV);
    K.set(0, 4, kvneg0 - (kvpos1 - kvpos0) * sqrtAbsV);
    K.set(1, 0, kx);
    K.set(1, 1, -K.get(0, 1));
    K.set(1, 2, -K.get(0, 2));
    K.set(1, 3, K.get(0, 4));
    K.set(1, 4, K.get(0, 3));

    @SuppressWarnings("VariableDeclarationUsageDistance")
    var inRobotFrame = new Matrix<N5, N5>(SimpleMatrix.identity(5));
    K.set(0, 0, Math.cos(x.get(2, 0)));
    K.set(0, 1, Math.sin(x.get(2, 0)));
    K.set(1, 0, -Math.sin(x.get(2, 0)));
    K.set(1, 1, Math.cos(x.get(2, 0)));

    Matrix<N5, N1> error = r.minus(x);

    error.set(State.kHeading.value, 0, normalizeAngle(error.get(State.kHeading.value, 0)));

    return K.times(inRobotFrame).times(error);
  }

  @SuppressWarnings("JavadocMethod")
  protected Matrix<N10, N1> getDynamics(Matrix<N10, N1> x, Matrix<N2, N1> u) {
    Matrix<N4, N2> B = new Matrix<>(new SimpleMatrix(4, 2));
    B.getStorage().insertIntoThis(0, 0, m_plant.getB().getStorage());
    B.getStorage().insertIntoThis(2, 0, new SimpleMatrix(2, 2));

    Matrix<N4, N7> A = new Matrix<>(new SimpleMatrix(4, 7));
    A.getStorage().insertIntoThis(0, 0, m_plant.getA().getStorage());

    A.getStorage().insertIntoThis(2, 0, SimpleMatrix.identity(2));
    A.getStorage().insertIntoThis(0, 2, new SimpleMatrix(4, 2));
    A.getStorage().insertIntoThis(0, 4, B.getStorage());
    A.getStorage().setColumn(6, 0, 0, 0, 1, -1);

    var v = (x.get(State.kLeftVelocity.value, 0) + x.get(State.kRightVelocity.value, 0)) / 2.0;

    var result = new Matrix<N10, N1>(new SimpleMatrix(10, 1));
    result.set(0, 0, v * Math.cos(x.get(State.kHeading.value, 0)));
    result.set(1, 0, v * Math.sin(x.get(State.kHeading.value, 0)));
    result.set(2, 0, (x.get(State.kRightVelocity.value, 0)
            - x.get(State.kLeftVelocity.value, 0)) / (2.0 * m_rb));

    result.getStorage().insertIntoThis(3, 0, A.times(new Matrix<N7, N1>(
            x.getStorage().extractMatrix(3, 10, 0, 1))).plus(B.times(u)).getStorage());
    result.getStorage().insertIntoThis(7, 0, new SimpleMatrix(3, 1));
    return result;
  }

  /**
  * Returns if the controller is at the reference pose on the trajectory.
  * Note that this is different than if the robot has traversed the entire
  * trajectory. The tolerance is set by the {@link #setTolerance(Pose2d, double)}
  * method.
  *
  * @return If the robot is within the specified tolerance of the
  */
  public boolean atReference() {
    var tolTranslate = m_poseTolerance.getTranslation();
    var tolRotate = m_poseTolerance.getRotation();
    return Math.abs(m_stateError.get(0, 0)) < tolTranslate.getX()
        && Math.abs(m_stateError.get(1, 0)) < tolTranslate.getY()
        && Math.abs(m_stateError.get(2, 0)) < tolRotate.getRadians()
        && Math.abs(m_stateError.get(3, 0)) < m_velocityTolerance
        && Math.abs(m_stateError.get(4, 0)) < m_velocityTolerance;
  }

  /**
   * Set the tolerance for if the robot is {@link #atReference()} or not.
   *
   * @param poseTolerance The new pose tolerance.
   * @param velocityTolerance The velocity tolerance.
   */
  public void setTolerance(final Pose2d poseTolerance, final double velocityTolerance) {
    this.m_poseTolerance = poseTolerance;
    this.m_velocityTolerance = velocityTolerance;
  }

  /**
   * Returns the current controller reference in the form
   * [X, Y, Heading, LeftVelocity, RightVelocity, LeftPosition].
   *
   * @return Matrix [N10, N1] The reference.
   */
  public Matrix<N5, N1> getReferences() {
    return m_nextR;
  }

  /**
   * Returns the inputs of the controller in the form [LeftVoltage, RightVoltage].
   *
   * @return Matrix [N2, N1] The inputs.
   */
  public Matrix<N2, N1> getInputs() {
    return m_cappedU;
  }

  /**
   * Returns the uncapped control input after updating the controller with the given
   *  reference and current states.
   *
   * @param currentState  The current state of the robot as a vector.
   * @param stateRef      The reference state vector.
   * @return The control input as a {@link DifferentialDriveMotorVoltages}.
   */
  public DifferentialDriveMotorVoltages calculate(
          Matrix<N5, N1> currentState,
          Matrix<N5, N1> stateRef) {
    m_nextR = stateRef;
    m_stateError = m_nextR.minus(currentState);

    m_cappedU = getController(currentState, m_nextR);

    return new DifferentialDriveMotorVoltages(
            m_cappedU.get(Input.kLeftVoltage.value, 0),
            m_cappedU.get(Input.kRightVoltage.value, 0));
  }

  /**
  * Returns the next output of the controller.
  *
  * <p>The desired state should come from a {@link Trajectory}.
  *
  * @param currentState  The current state of the robot as a vector.
  * @param desiredState  The desired pose, linear velocity, and angular velocity
  *                      from a trajectory.
  */
  public DifferentialDriveMotorVoltages calculate(Matrix<N5, N1> currentState,
                                                  Trajectory.State desiredState) {
    var wheelVelocities = m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(desiredState.velocityMetersPerSecond,
              0,
              desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter));
    
    Matrix<N5, N1> stateRef = new MatBuilder<>(Nat.N5(), Nat.N1()).fill(
            desiredState.poseMeters.getTranslation().getX(),
            desiredState.poseMeters.getTranslation().getY(),
            desiredState.poseMeters.getRotation().getRadians(),
            wheelVelocities.leftMetersPerSecond,
            wheelVelocities.rightMetersPerSecond);
    
    return calculate(currentState, stateRef);
  }

  /**
  * Resets any internal state.
  */
  public void reset() {
    m_nextR = MatrixUtils.zeros(Nat.N5(), Nat.N1());
    m_cappedU = MatrixUtils.zeros(Nat.N2(), Nat.N1());
  }

  private double normalizeAngle(double angle) {
    final int n_pi_pos = (int) ((angle + Math.PI) / 2.0 / Math.PI);
    angle -= n_pi_pos * 2.0 * Math.PI;

    final int n_pi_neg = (int) ((angle - Math.PI) / 2.0 / Math.PI);
    angle -= n_pi_neg * 2.0 * Math.PI;

    return angle;
  }

  private enum State {
    kX(0),
    kY(1),
    kHeading(2),
    kLeftVelocity(3),
    kRightVelocity(4),
    kLeftPosition(5),
    kRightPosition(6),
    kLeftVoltageError(7),
    kRightVoltageError(8),
    kAngularVelocityError(9);

    private final int value;

    State(int i) {
      this.value = i;
    }
  }

  private enum Input {
    kLeftVoltage(0), kRightVoltage(1);

    private final int value;

    Input(int i) {
      this.value = i;
    }
  }
}
