package edu.wpi.first.wpilibj.simulation;

import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.RungeKutta;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

public class SimSingleJointedArm extends SimLinearSystem<N2, N1, N1> {

  @SuppressWarnings("MemberName")
  private final double m_r;
  private final double m_armMass;
  private final DCMotor m_motor;
  private final double m_gearing;

  /**
   * Create a LinearSystemSimulator. This simulator uses an {@link LinearSystem}
   * to simulate the state of the system. In simulationPeriodic, users should
   * first set inputs from motors, update the simulation and write simulated
   * outputs to sensors.
   *
   * @param armPlant           The arm system being controlled.
   * @param addNoise           If we should add noise to the measurement
   *                           vector.
   * @param measurementStdDevs Standard deviations of measurements. Can be null
   *                           if addNoise is false.
   * @param armMassKg          The mass of the arm.
   * @param armLengthMeters    The distance from the pivot of the arm to its center
   *                           of rotation. This number is not the same as the overall length of the arm.
   * @param motor              The DCMotor used to drive the arm.
   * @param gearing            The gearing between the motor and the output, written as
   *                           output over input. In most cases this should be greater than one.
   */
  public SimSingleJointedArm(LinearSystem<N2, N1, N1> armPlant, boolean addNoise, Matrix<N1, N1> measurementStdDevs,
                             double armMassKg, double armLengthMeters, DCMotor motor, double gearing) {
    super(armPlant, addNoise, measurementStdDevs);
    this.m_armMass = armMassKg;
    this.m_r = armLengthMeters;
    this.m_motor = motor;
    this.m_gearing = gearing;
  }

  @Override
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are output over input, so a reduction of 2:1 means the motor is spinning
    // 2x faster than the flywheel
    return m_u.get(0, 0) / m_motor.m_rOhms
      - getVelocityRadPerSec() * m_gearing
      / (m_motor.m_KvRadPerSecPerVolt * m_motor.m_rOhms);
  }

  public double getPositionRadians() {
    return m_trueXhat.get(0, 0);
  }

  public double getVelocityRadPerSec() {
    return m_trueXhat.get(1, 0);
  }

  @Override
  protected Matrix<N2, N1> updateXhat(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
    /*
    Horizontal case:
    Torque = f * r = I * alpha
    alpha = f * r / I
    since f=mg,
    alpha = m g r / I

    Multiply RHS by cos(theta) to account for the arm angle
     */
    return RungeKutta.rungeKutta((x, u_) -> (m_plant.getA().times(x)).plus(m_plant.getB().times(u_))
        .plus(VecBuilder.fill(0, m_armMass * m_r * -9.8 / (m_armMass * m_r * m_r))),
      currentXhat, u, dtSeconds);
  }
}
