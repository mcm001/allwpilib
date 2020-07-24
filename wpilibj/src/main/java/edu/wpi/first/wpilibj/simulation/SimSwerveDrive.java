/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.simulation;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.MatrixUtils;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;
import org.ejml.MatrixDimensionException;

import java.util.Arrays;

public class SimSwerveDrive {
  private final SwerveDriveKinematics m_kinematics;
  private final double m_chassisMassKg;
  private final double m_chassisMoiKgMetersSquared;
  private final SimSwerveModule[] m_modules;
  private double[] m_moduleVoltages;
  private Rotation2d[] m_moduleAngles;

  public SimSwerveDrive(double chassisMassKg, double chassisMoiKgMetersSquared, SimSwerveModule... modules) {
    this.m_chassisMassKg = chassisMassKg;
    this.m_chassisMoiKgMetersSquared = chassisMoiKgMetersSquared;
    this.m_modules = modules.clone();

    this.m_kinematics = new SwerveDriveKinematics(
        Arrays.stream(modules).map(SimSwerveModule::getPosition).toArray(Translation2d[]::new));

    m_moduleVoltages = new double[m_modules.length];
    m_moduleAngles = new Rotation2d[m_modules.length];
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  @SuppressWarnings("MemberName")
  /*
    Chassis state vector. States are [Vx, Vy, Omega].
   */
  protected Matrix<N3, N1> m_x = VecBuilder.fill(0, 0, 0);

  protected LinearSystem<N3, N3, N3> m_velocitySystem = new LinearSystem<>(
      MatrixUtils.zeros(Nat.N3(), Nat.N3()), MatrixUtils.eye(Nat.N3()), MatrixUtils.eye(Nat.N3()),
      MatrixUtils.zeros(Nat.N3(), Nat.N3()));

  public ChassisSpeeds getSpeeds() {
    return new ChassisSpeeds(m_x.get(0, 0),
        m_x.get(1, 0),
        m_x.get(2, 0));
  }

  public SwerveModuleState[] getModuleStates() {
    return m_kinematics.toSwerveModuleStates(getSpeeds());
  }

  public SimSwerveModule[] getModules() {
    return m_modules.clone();
  }

  public void setModuleVoltages(double... voltages) {
    if (voltages.length != m_modules.length) {
      throw new MatrixDimensionException(String.format("Got the wrong number of inputs. Got %s voltage inputs, but I have %s wheels!", voltages.length, m_modules.length));
    }

    m_moduleVoltages = voltages.clone();
  }

  public void setModuleVoltage(int module, double voltage) {
    m_moduleVoltages[module] = voltage;
  }

  public void setModuleAngles(Rotation2d... angles) {
    if (angles.length != m_modules.length) {
      throw new MatrixDimensionException(String.format("Got the wrong number of angles. Got %s angles, but I have %s wheels!", angles.length, m_modules.length));
    }

    m_moduleAngles = angles.clone();
  }

  public void setModuleAngle(int module, Rotation2d angle) {
    m_moduleAngles[module] = angle;
  }

  public void update(double dtSeconds) {
    var netTorque = 0.0;
    var netForce = new Translation2d();

    var wheelSpeeds = m_kinematics.toSwerveModuleStates(getSpeeds());

    for (int i = 0; i < m_modules.length; i++) {
      var module = m_modules[i];
      // Sum forces and torques so we can use them as inputs to our linear system
      // Torque = R_robot2wheel x F, and the cross product of two vectors whos Z is zero is
      // A x B = (A_x B_y - A_y B_x)

      var force = module.getModuleForceNewtons(wheelSpeeds[i].speedMetersPerSecond, m_moduleVoltages[i], m_moduleAngles[i]);
      netForce = netForce.plus(force);
      var moduleTorque = module.getPosition().getX() * force.getY() - module.getPosition().getY() * force.getX();
      netTorque += moduleTorque;
    }

    var netLinearAcceleration = netForce.div(m_chassisMassKg);
    var netAngularAcceleration = netTorque / m_chassisMoiKgMetersSquared;

    //    System.out.println(String.format("%s, %s, %s", netLinearAcceleration.getX(), netLinearAcceleration.getY(), netAngularAcceleration));

    m_x = m_velocitySystem.calculateX(m_x, VecBuilder.fill(netLinearAcceleration.getX(), netLinearAcceleration.getY(),
        netAngularAcceleration), dtSeconds);
  }

  public void resetChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    this.m_x = VecBuilder.fill(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);
  }

  public static class SimSwerveModule {
    private final DCMotor m_driveMotor;
    private final double m_driveMotorGearing;
    private final double m_driveWheelRadiusMeters;
    private final Translation2d m_position;

    /**
     * Construct a SimSwerveModule. The base unit of this model must be radians. This model
     * can be created with {@link edu.wpi.first.wpilibj.system.plant.LinearSystemId#createSingleJointedArmSystem(DCMotor, double, double)}
     * or {@link edu.wpi.first.wpilibj.system.plant.LinearSystemId#identifyPositionSystem(double, double)}.
     *
     * @param driveMotor             The drive motor as a DCMotor.
     * @param driveMotorGearing      The gearing between the drive motor and wheel. Usually greater than one.
     * @param driveWheelRadiusMeters The radius of the wheel in meters.
     * @param positionMeters         The locations of this wheel relative to the physical center of the
     *                               robot.
     */
    public SimSwerveModule(DCMotor driveMotor, double driveMotorGearing,
                           double driveWheelRadiusMeters, Translation2d positionMeters) {
      this.m_driveMotor = driveMotor;
      this.m_driveMotorGearing = driveMotorGearing;
      this.m_driveWheelRadiusMeters = driveWheelRadiusMeters;
      this.m_position = positionMeters;
    }

    @SuppressWarnings("LocalVariableName")
    public Translation2d getModuleForceNewtons(double wheelVelocityMps, double wheelVoltage, Rotation2d moduleHeading) {
      // By the elevator equations of motion presented in Controls Engineering in FRC,
      // F_m = (G Kt)/(R r) Voltage - (G^2 Kt)/(R r^2 Kv) velocity
      var G = m_driveMotorGearing;
      var r = m_driveWheelRadiusMeters;
      var m = m_driveMotor;
      var forceNewtons = G * m.m_KtNMPerAmp / (m.m_rOhms * r) * wheelVoltage - (G * G * m.m_KtNMPerAmp)
          / (m.m_rOhms * r * r * m.m_KvRadPerSecPerVolt) * wheelVelocityMps;

      return new Translation2d(moduleHeading.getCos() * forceNewtons,
          moduleHeading.getSin() * forceNewtons);
    }

    public Translation2d getPosition() {
      return m_position;
    }
  }
}
