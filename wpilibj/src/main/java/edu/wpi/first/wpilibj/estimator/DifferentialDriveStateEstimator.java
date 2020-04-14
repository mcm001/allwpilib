/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.estimator;

import java.util.function.BiConsumer;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.MatrixUtils;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N10;
import edu.wpi.first.wpiutil.math.numbers.N2;
import edu.wpi.first.wpiutil.math.numbers.N3;
import edu.wpi.first.wpiutil.math.numbers.N4;
import edu.wpi.first.wpiutil.math.numbers.N7;

/**
 * This class wraps an Unscented Kalman Filter to fuse latency-compensated global
 * measurements(ex. vision) with differential drive encoder
 * measurements. It will correct for noisy global measurements and encoder drift.
 *
 * <p>This class is indented to be paired with
 * {@link edu.wpi.first.wpilibj.controller.LTVDiffDriveController} as it provides a
 * 10-state estimate. This can then be trimmed into 5-state using{@link Matrix#block}
 * with the operation
 * ```10-stateEstimate.block(Nat.N5(), Nat.N1(), new SimpleMatrixUtils.Pair(0, 0))```
 * then passed into the controller as the current state estimate.
 *
 * <p>{@link DifferentialDriveStateEstimator#update} should be called every robot
 * loop (if your robot loops are faster than the default then you should change
 * the {@link DifferentialDriveStateEstimator#DifferentialDriveStateEstimator(LinearSystem,
 * Matrix, Matrix, Matrix, Matrix, DifferentialDriveKinematics, double) nominal delta time}.)
 * {@link DifferentialDriveStateEstimator#applyPastGlobalMeasurement} can be called as
 * infrequently as you want.
 *
 * <p>Our state-space system is:
 *
 * <p>x = [[x, y, theta, vel_l, vel_r, dist_l, dist_r, voltError_l, voltError_r, angularVelError]]^T
 * in the field coordinate system (dist_* are wheel distances.)
 *
 * <p>u = [[voltage_l, voltage_r]]^T This is typically the control input of the last timestep
 * from a {@link edu.wpi.first.wpilibj.controller.LTVDiffDriveController}.
 *
 * <p>y = [[x, y, theta]]^T from vision, or y = [[dist_l, dist_r, theta]] from encoders and gyro.
 */
@SuppressWarnings({"ParameterName", "LocalVariableName", "MemberName"})
public class DifferentialDriveStateEstimator {
  private final UnscentedKalmanFilter<N10, N2, N3> m_observer;
  private final KalmanFilterLatencyCompensator<N10, N2, N3> m_latencyCompensator;

  private final BiConsumer<Matrix<N2, N1>, Matrix<N3, N1>> m_globalCorrect;

  private final double m_rb;
  private final LinearSystem<N2, N2, N2> m_plant;

  private final double m_nominalDt; // Seconds
  private double m_prevTimeSeconds = -1.0;


  private Matrix<N3, N1> m_localY;
  private Matrix<N3, N1> m_globalY;

  /**
   * Constructs a DifferentialDriveStateEstimator.
   *
   * @param plant                    A {@link LinearSystem} representing a differential drivetrain.
   * @param initialState             The starting state estimate.
   * @param stateStdDevs             Standard deviations of model states. Increase these numbers to
   *                                 trust your wheel and gyro velocities less.
   * @param localMeasurementStdDevs  Standard deviations of the encoder and gyro measurements.
   *                                 Increase these numbers to trust encoder distances and gyro
   *                                 angle less.
   * @param globalMeasurementStdDevs Standard deviations of the global(vision) measurements.
   *                                 Increase these numbers to global(vision) measurements less.
   * @param kinematics               A {@link DifferentialDriveKinematics} object representing the
   *                                 differential drivetrain's kinematics.
   */
  public DifferentialDriveStateEstimator(LinearSystem<N2, N2, N2> plant,
                                        Matrix<N10, N1> initialState,
                                        Matrix<N10, N1> stateStdDevs,
                                        Matrix<N3, N1> localMeasurementStdDevs,
                                        Matrix<N3, N1> globalMeasurementStdDevs,
                                        DifferentialDriveKinematics kinematics
  ) {
    this(plant, initialState, stateStdDevs,
            localMeasurementStdDevs, globalMeasurementStdDevs, kinematics, 0.02);
  }

  /**
   * Constructs a DifferentialDriveStateEstimator.
   *
   * @param plant                    A {@link LinearSystem} representing a differential drivetrain.
   * @param initialState             The starting state estimate.
   * @param stateStdDevs             Standard deviations of model states. Increase these numbers to
   *                                 trust your wheel and gyro velocities less.
   * @param localMeasurementStdDevs  Standard deviations of the encoder and gyro measurements.
   *                                 Increase these numbers to trust encoder distances and gyro
   *                                 angle less.
   * @param globalMeasurementStdDevs Standard deviations of the global(vision) measurements.
   *                                 Increase these numbers to global(vision) measurements less.
   * @param kinematics               A {@link DifferentialDriveKinematics} object representing the
   *                                 differential drivetrain's kinematics.
   * @param nominalDtSeconds         The time in seconds between each robot loop.
   */
  public DifferentialDriveStateEstimator(
          LinearSystem<N2, N2, N2> plant,
          Matrix<N10, N1> initialState,
          Matrix<N10, N1> stateStdDevs,
          Matrix<N3, N1> localMeasurementStdDevs,
          Matrix<N3, N1> globalMeasurementStdDevs,
          DifferentialDriveKinematics kinematics,
          double nominalDtSeconds
  ) {
    m_nominalDt = nominalDtSeconds;
    m_plant = plant;
    m_rb = kinematics.trackWidthMeters / 2.0;

    m_localY = MatrixUtils.zeros(Nat.N3());
    m_globalY = MatrixUtils.zeros(Nat.N3());

    var globalContR = StateSpaceUtil.makeCovMatrix(Nat.N3(), globalMeasurementStdDevs);
    var globalDiscR = StateSpaceUtil.discretizeR(globalContR, m_nominalDt);

    m_observer = new UnscentedKalmanFilter<>(
      Nat.N10(), Nat.N2(), Nat.N3(),
      this::getDynamics,
      this::getLocalMeasurementModel,
      stateStdDevs,
      localMeasurementStdDevs,
      nominalDtSeconds
   );

    m_globalCorrect = (u, y) -> m_observer.correct(
      Nat.N3(), u, y,
      this::getGlobalMeasurementModel,
      globalDiscR
    );
    m_latencyCompensator = new KalmanFilterLatencyCompensator<>();

    reset(initialState);
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

  @SuppressWarnings("JavadocMethod")
  public Matrix<N3, N1> getLocalMeasurementModel(Matrix<N10, N1> x, Matrix<N2, N1> u) {
    return new MatBuilder<>(Nat.N3(), Nat.N1()).fill(x.get(State.kHeading.value, 0),
            x.get(State.kLeftPosition.value, 0), x.get(State.kRightPosition.value, 0));
  }

  @SuppressWarnings("JavadocMethod")
  public Matrix<N3, N1> getGlobalMeasurementModel(Matrix<N10, N1> x, Matrix<N2, N1> u) {
    return new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
        x.get(State.kX.value, 0),
        x.get(State.kY.value, 0),
        x.get(State.kHeading.value, 0)
    );
  }

  /**
   * Set global measurements.
   *
   * @param robotPoseMeters  The measured robot pose.
   * @param timestampSeconds The timestamp of the global measurement in seconds. Note that if
   *                         you don't use your own time source by calling
   *                         {@link DifferentialDriveStateEstimator#updateWithTime} then you
   *                         must use a timestamp with an epoch since FPGA startup
   *                         (i.e. the epoch of this timestamp is the same epoch as
   *                         {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp
   *                         Timer.getFPGATimestamp}.) This means that you should
   *                         use Timer.getFPGATimestamp as your time source in
   *                         this case.
   */
  public void applyPastGlobalMeasurement(Pose2d robotPoseMeters,
                                         double timestampSeconds) {
    m_globalY.set(GlobalOutput.kX.value, 0, robotPoseMeters.getTranslation().getX());
    m_globalY.set(GlobalOutput.kY.value, 0, robotPoseMeters.getTranslation().getY());
    m_globalY.set(GlobalOutput.kHeading.value, 0, robotPoseMeters.getRotation().getRadians());

    m_latencyCompensator.applyPastGlobalMeasurement(
            Nat.N3(),
            m_observer, m_nominalDt,
            m_globalY,
            m_globalCorrect,
            timestampSeconds
    );
  }

  /**
   * Gets the state of the robot at the current time as estimated by the Unscented Kalman Filter.
   *
   * @return The robot state estimate.
   */
  public Matrix<N10, N1> getEstimatedState() {
    return m_observer.getXhat();
  }

  /**
   * Updates the the Unscented Kalman Filter using wheel encoder information, robot heading and the
   * previous control input. The control input can be obtained from a
   * {@link edu.wpi.first.wpilibj.controller.LTVDiffDriveController}.
   * Note that this should be called every loop.
   *
   * @param headingRadians The current heading of the robot in radians.
   * @param leftPosition The distance traveled by the left side of the robot in meters.
   * @param rightPosition The distance traveled by the right side of the robot in meters.
   * @param prevInput The control input from the last timestep.
   * @return The robot state estimate.
   */
  public Matrix<N10, N1> update(double headingRadians, double leftPosition,
                                double rightPosition,
                                Matrix<N2, N1> prevInput) {
    return updateWithTime(headingRadians,
            leftPosition,
            rightPosition,
            prevInput,
            Timer.getFPGATimestamp());
  }

  /**
   * Updates the the Unscented Kalman Filter using wheel encoder information, robot heading and the
   * previous control input. The control input can be obtained from a
   * {@link edu.wpi.first.wpilibj.controller.LTVDiffDriveController}.
   * Note that this should be called every loop.
   *
   * @param headingRadians The current heading of the robot in radians.
   * @param leftPosition The distance traveled by the left side of the robot in meters.
   * @param rightPosition The distance traveled by the right side of the robot in meters.
   * @param prevInput The control input from the last timestep.
   * @param currentTimeSeconds ime at which this method was called, in seconds.
   * @return The robot state estimate.
   */
  @SuppressWarnings("VariableDeclarationUsageDistance")
  public Matrix<N10, N1> updateWithTime(double headingRadians, double leftPosition,
                                        double rightPosition,
                                        Matrix<N2, N1> prevInput,
                                        double currentTimeSeconds) {
    double dt = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : m_nominalDt;
    m_prevTimeSeconds = currentTimeSeconds;

    m_localY.set(LocalOutput.kHeading.value, 0, headingRadians);
    m_localY.set(LocalOutput.kLeftPosition.value, 0, leftPosition);
    m_localY.set(LocalOutput.kRightPosition.value, 0, rightPosition);

    m_latencyCompensator.addObserverState(m_observer, prevInput, m_localY, currentTimeSeconds);

    m_observer.predict(prevInput, dt);
    m_observer.correct(prevInput, m_localY);

    return getEstimatedState();
  }

  /**
  * Resets any internal state with a given initial state.
  *
  * @param initialState Initial state for state estimate.
  */
  public void reset(Matrix<N10, N1> initialState) {
    m_observer.reset();

    m_observer.setXhat(initialState);
  }

  /**
   * Resets any internal state.
   */
  public void reset() {
    m_observer.reset();
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

  private enum LocalOutput {
    kHeading(0), kLeftPosition(1), kRightPosition(2);

    private final int value;

    LocalOutput(int i) {
      this.value = i;
    }
  }

  private enum GlobalOutput {
    kX(0),
    kY(1),
    kHeading(2);

    private final int value;

    GlobalOutput(int i) {
      this.value = i;
    }
  }
}