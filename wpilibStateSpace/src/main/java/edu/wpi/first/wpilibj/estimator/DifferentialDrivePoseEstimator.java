package edu.wpi.first.wpilibj.estimator;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;

/**
 * This class wraps an Extended Kalman Filter to fuse latency-compensated vision
 * measurements with differential drive encoder measurements. It will correct
 * for noisy vision measurements and encoder drift. It is intended to be an easy
 * drop-in for
 * {@link edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry}; in fact,
 * if you never call {@link DifferentialDrivePoseEstimator#addVisionMeasurement}
 * and only call {@link DifferentialDrivePoseEstimator#update} then this will
 * behave exactly the same as DifferentialDriveOdometry.
 * <p>
 * {@link DifferentialDrivePoseEstimator#update} should be called every robot
 * loop (if your robot loops are faster than the default then you should change
 * the
 * {@link DifferentialDrivePoseEstimator#DifferentialDrivePoseEstimator(Matrix, Matrix, double)
 * nominal delta time}.)
 * {@link DifferentialDrivePoseEstimator#addVisionMeasurement} can be called as
 * infrequently as you want; if you never call it then this class will behave
 * exactly like regular encoder odometry.
 * <p>
 * Our state-space system is:
 * <p>
 * x = [[x, y, dtheta]]^T in the field coordinate system.
 * <p>
 * u = [[d_l, d_r, dtheta]]^T -- these aren't technically system inputs, but
 * they make things considerably easier.
 * <p>
 * y = [[x, y, theta]]^T
 */
public class DifferentialDrivePoseEstimator {
    private final ExtendedKalmanFilter<N3, N3, N3> mObserver;
    private final KalmanFilterLatencyCompensator<N3, N3, N3> mLatencyCompensator;
    /** Seconds */
    private final double mNominalDt;

    private Rotation2d mGyroOffset;
    private Rotation2d mPreviousAngle;

    public DifferentialDrivePoseEstimator(Rotation2d gyroAngle, Pose2d initialPoseMeters, Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> measurementStdDevs) {
        this(gyroAngle, initialPoseMeters, stateStdDevs, measurementStdDevs, 0.01);
    }

    public DifferentialDrivePoseEstimator(Rotation2d gyroAngle, Pose2d initialPoseMeters, Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> measurementStdDevs, double nominalDtSeconds) {
        mNominalDt = nominalDtSeconds;

        mObserver = new ExtendedKalmanFilter<N3, N3, N3>(Nat.N3(), Nat.N3(), Nat.N3(), this::f, (x, u) -> x,
                stateStdDevs, measurementStdDevs, false, mNominalDt);
        mLatencyCompensator = new KalmanFilterLatencyCompensator<>();

        mGyroOffset = initialPoseMeters.getRotation().minus(gyroAngle);
        mPreviousAngle = initialPoseMeters.getRotation();
        mObserver.setXhat(poseToVector(initialPoseMeters));
    }

    private final Matrix<N3, N1> f(Matrix<N3, N1> x, Matrix<N3, N1> u) {
        // Diff drive forward kinematics:
        // v_c = (v_l + v_r) / 2
        double dx = (u.get(0, 0) + u.get(1, 0)) / 2;
        var newPose = new Pose2d(x.get(0, 0), x.get(1, 0), new Rotation2d(x.get(2, 0)))
                .exp(new Twist2d(dx, 0.0, u.get(2, 0)));

        return new MatBuilder<>(Nat.N3(), Nat.N1()).fill(newPose.getTranslation().getX(),
                newPose.getTranslation().getY(), x.get(2, 0) + u.get(2, 0));
    }

    /**
     * Add a vision measurement to the Extended Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     * <p>
     * This method can be called as infrequently as you want, as long as you are
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
        mLatencyCompensator.applyPastMeasurement(mObserver, mNominalDt, poseToVector(visionRobotPose),
                timestampSeconds);
    }

    /**
     * Update the the Extended Kalman Filter using only wheel encoder information.
     * Note that this should be called every loop (and the correct loop period must
     * be passed into the constructor of this class.)
     * 
     * @param gyroAngle           The current gyro angle.
     * @param leftDistanceMeters  The distance the left wheel has travelled since
     *                            the last loop iteration.
     * @param rightDistanceMeters The distance the left wheel has travelled since
     *                            the last loop iteration.
     * @return The estimated pose of the robot.
     */
    public Pose2d update(Rotation2d gyroAngle, double leftDistanceMeters, double rightDistanceMeters) {
        var angle = gyroAngle.plus(mGyroOffset);
        var u = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(leftDistanceMeters, rightDistanceMeters,
                angle.minus(mPreviousAngle).getRadians());
        mPreviousAngle = angle;

        mLatencyCompensator.addObserverState(mObserver, u);
        mObserver.predict(u, mNominalDt);

        return new Pose2d(mObserver.getXhat(0), mObserver.getXhat(1), new Rotation2d(mObserver.getXhat(2)));
    }

    private Matrix<N3, N1> poseToVector(Pose2d pose) {
        return new MatBuilder<>(Nat.N3(), Nat.N1()).fill(pose.getTranslation().getX(), pose.getTranslation().getY(),
                pose.getRotation().getRadians());
    }
}
