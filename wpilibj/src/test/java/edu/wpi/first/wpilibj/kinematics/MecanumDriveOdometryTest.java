package edu.wpi.first.wpilibj.kinematics;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class MecanumDriveOdometryTest {

  private final Translation2d m_fl = new Translation2d(12, 12);
  private final Translation2d m_fr = new Translation2d(12, -12);
  private final Translation2d m_bl = new Translation2d(-12, 12);
  private final Translation2d m_br = new Translation2d(-12, -12);

  private final MecanumDriveKinematics m_kinematics =
      new MecanumDriveKinematics(m_fl, m_fr, m_bl, m_br);

  private final MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics);

  @Test
  void testMultipleConsecutiveUpdates() {
      var wheelSpeeds = new MecanumDriveWheelSpeeds(3.536, 3.536, 3.536, 3.536);

      m_odometry.updateWithTime(0.0, 0.0, wheelSpeeds);
      var secondPose = m_odometry.updateWithTime(0.0, 0.0, wheelSpeeds);

      assertAll(
        () -> assertEquals(secondPose.getTranslation().getX(), 0.0, 0.01),
        () -> assertEquals(secondPose.getTranslation().getY(), 0.0, 0.01),
        () -> assertEquals(secondPose.getRotation().getDegrees(), 0.0, 0.01)
      );
    }

  @Test
  void testTwoIterations() {
    // 5 units/sec  in the x axis (forward)
    final var wheelSpeeds = new MecanumDriveWheelSpeeds(3.536, 3.536, 3.536, 3.536);

    m_odometry.updateWithTime(0.0, 0.0, new MecanumDriveWheelSpeeds());
    var pose = m_odometry.updateWithTime(0.10, wheelSpeeds);

    assertAll(
      () -> assertEquals(5.0 / 10.0, pose.getTranslation().getX(), 0.01),
      () -> assertEquals(0, pose.getTranslation().getY(), 0.01),
      () -> assertEquals(0.0, pose.getRotation().getDegrees(), 0.01)
    );
  }

  @Test
  void test90degreeTurn() {
    // This is a 90 degree turn about the point between front left and rear left wheels
    // fl -13.328649 fr 39.985946 rl -13.328649 rr 39.985946
    final var wheelSpeeds = new MecanumDriveWheelSpeeds(-13.328, 39.986, -13.329, 39.986);

    m_odometry.updateWithTime(0.0, 0.0, new MecanumDriveWheelSpeeds());
    final var pose = m_odometry.updateWithTime(1.0, /*Math.toRadians(90),*/ wheelSpeeds);

    assertAll(
      () -> assertEquals(12.0, pose.getTranslation().getX(), 0.01),
      () -> assertEquals(12.0, pose.getTranslation().getY(), 0.01),
      () -> assertEquals(90.0, pose.getRotation().getDegrees(), 0.01)
    );
  }

}
