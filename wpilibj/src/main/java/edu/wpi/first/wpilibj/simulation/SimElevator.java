/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.simulation;

import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.RungeKutta;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

public class SimElevator extends SimLinearSystem<N2, N1, N1> {

  private final DCMotor m_motor;
  private final double m_drumRadiusMeters;
  private final double m_gearing;

  /**
   * Create a LinearSystemSimulator. This simulator uses an {@link LinearSystem} to simulate
   * the state of the system. In simulationPeriodic, users should first set inputs from motors, update
   * the simulation and write simulated outputs to sensors.
   *
   * @param elevatorPlant        The elevator system being controlled. This system can be created
   *                             with {@link edu.wpi.first.wpilibj.system.plant.LinearSystemId#createElevatorSystem(DCMotor, double, double, double)}
   *                             or {@link edu.wpi.first.wpilibj.system.plant.LinearSystemId#identifyPositionSystem(double, double)}.
   * @param addNoise             If we should add noise to the measurement vector
   * @param m_measurementStdDevs Standard deviations of measurements. Can be null if addNoise is false.
   */
  public SimElevator(LinearSystem<N2, N1, N1> elevatorPlant, boolean addNoise,
                     Matrix<N1, N1> m_measurementStdDevs, DCMotor elevatorGearbox,
                     double gearingReduction, double drumRadiusMeters) {
    super(elevatorPlant, addNoise, m_measurementStdDevs);
    this.m_motor = elevatorGearbox;
    this.m_gearing = gearingReduction;
    this.m_drumRadiusMeters = drumRadiusMeters;
  }

  @Override
  protected Matrix<N2, N1> updateXhat(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
    return RungeKutta.rungeKutta((x, u_) -> (m_plant.getA().times(x)).plus(m_plant.getB().times(u_)).plus(VecBuilder.fill(0, -9.8)),
          currentXhat, u, dtSeconds);
  }

  public double getElevatorPositionMeters() {
    return getOutput(0);
  }

  public double getElevatorVelocityMetersPerSecond() {
    return m_trueXhat.get(0, 1);
  }

  @Override
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
    // spinning 10x faster than the output
    // v = r w, so w = v/r
    double velocityMps = m_trueXhat.get(0, 1);
    double motorVelocityRadPerSec = velocityMps / m_drumRadiusMeters * m_gearing;
    return m_u.get(0, 0) / m_motor.m_rOhms
          - motorVelocityRadPerSec / (m_motor.m_KvRadPerSecPerVolt * m_motor.m_rOhms);
  }
}
