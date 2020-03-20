package edu.wpi.first.wpilibj.estimator;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.MatrixUtils;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;

public class SwerveDrivePoseEstimator {
  private final KalmanFilter<N3, N3, N3> m_observer;
  private final SwerveDriveKinematics m_kinematics;
  private final KalmanFilterLatencyCompensator<N3, N3, N3> m_latencyCompensator;

  private final double m_nominalDt; // Seconds

  private Rotation2d m_gyroOffset;
  private Rotation2d m_previousAngle;

  /**
   * Constructs a SwerveDrivePose estimator.
   *
   * @param gyroAngle          The current gyro angle.
   * @param initialPoseMeters  The starting pose estimate.
   * @param stateStdDevs       Standard deviations of model states. Increase these numbers to trust
   *                           your encoders less.
   * @param measurementStdDevs Standard deviations of the measurements. Increase these numbers to
   *                           trust vision less.
   */
  public SwerveDrivePoseEstimator(
          Rotation2d gyroAngle, Pose2d initialPoseMeters, SwerveDriveKinematics kinematics,
          Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> measurementStdDevs
  ) {
    this(gyroAngle, initialPoseMeters, kinematics, stateStdDevs, measurementStdDevs, 0.01);
  }

  /**
   * Constructs a SwerveDrivePose estimator.
   *
   * @param gyroAngle          The current gyro angle.
   * @param initialPoseMeters  The starting pose estimate.
   * @param stateStdDevs       Standard deviations of model states. Increase these numbers to trust
   *                           your encoders less.
   * @param measurementStdDevs Standard deviations of the measurements. Increase these numbers to
   *                           trust vision less.
   * @param nominalDtSeconds   The time in seconds between each robot loop.
   */
  public SwerveDrivePoseEstimator(
          Rotation2d gyroAngle, Pose2d initialPoseMeters, SwerveDriveKinematics kinematics,
          Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> measurementStdDevs, double nominalDtSeconds
  ) {
    m_nominalDt = nominalDtSeconds;

    var observerSystem = new LinearSystem<>(
            Nat.N3(), Nat.N3(), Nat.N3(),
            MatrixUtils.zeros(Nat.N3(), Nat.N3()), // A
            new MatBuilder<>(Nat.N3(), Nat.N3()).fill( // B
                    1, 0, 0,
                    0, 1, 0,
                    0, 0, 1
            ),
            new MatBuilder<>(Nat.N3(), Nat.N3()).fill( // C
                    1, 0, 0,
                    0, 1, 0,
                    0, 0, 1
            ),
            MatrixUtils.zeros(Nat.N3(), Nat.N3()), // D
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill( // uMin
                    Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY
            ),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill( // uMax
                    Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY
            )
    );
    m_observer = new KalmanFilter<>(Nat.N3(), Nat.N3(), Nat.N3(), observerSystem, stateStdDevs,
            measurementStdDevs, nominalDtSeconds);
    m_kinematics = kinematics;
    m_latencyCompensator = new KalmanFilterLatencyCompensator<>();

    m_gyroOffset = initialPoseMeters.getRotation().minus(gyroAngle);
    m_previousAngle = initialPoseMeters.getRotation();
    m_observer.setXhat(poseToVector(initialPoseMeters));
  }

  /**
   * Add a vision measurement to the Extended Kalman Filter. This will correct the
   * odometry pose estimate while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are
   * calling {@link DifferentialDrivePoseEstimator#update} every loop.
   *
   * @param visionRobotPose  The pose of the robot as measured by the vision
   *                         camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds
   *                         since FPGA startup (i.e. the epoch of this timestamp
   *                         is the same epoch as
   *                         {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp
   *                         Timer.getFPGATimestamp}.) This means that you should
   *                         use Timer.getFPGATimestamp as your time source in
   *                         this case.
   */
  public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds) {
    m_latencyCompensator.applyPastMeasurement(
            m_observer, m_nominalDt,
            poseToVector(visionRobotPose), timestampSeconds
    );
  }

  /**
   * Updates the the Kalman Filter using only wheel encoder information.
   * Note that this should be called every loop (and the correct loop period must
   * be passed into the constructor of this class.)
   *
   * @param gyroAngle           The current gyro angle.
   * @param wheelStates         Velocities and rotations of the swerve modules.
   * @return The estimated pose of the robot.
   */
  @SuppressWarnings("LocalVariableName")
  public Pose2d update(Rotation2d gyroAngle, SwerveModuleState... wheelStates) {
    var angle = gyroAngle.plus(m_gyroOffset);
    var omega = angle.minus(m_previousAngle).getRadians() / m_nominalDt;

    var chassisSpeeds = m_kinematics.toChassisSpeeds(wheelStates);
    var fieldRelativeVelocities = new Pose2d(0, 0, angle).exp(
            new Twist2d(chassisSpeeds.vxMetersPerSecond,
                    chassisSpeeds.vyMetersPerSecond,
                    omega));

    var u = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
            fieldRelativeVelocities.getTranslation().getX(),
            fieldRelativeVelocities.getTranslation().getY(),
            omega
    );
    m_previousAngle = angle;

    m_latencyCompensator.addObserverState(m_observer, u);
    m_observer.predict(u, m_nominalDt);

    return new Pose2d(
            m_observer.getXhat(0),
            m_observer.getXhat(1),
            new Rotation2d(m_observer.getXhat(2))
    );
  }

  // TODO: Deduplicate
  private Matrix<N3, N1> poseToVector(Pose2d pose) {
    return new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
            pose.getTranslation().getX(),
            pose.getTranslation().getY(),
            pose.getRotation().getRadians()
    );
  }
}
