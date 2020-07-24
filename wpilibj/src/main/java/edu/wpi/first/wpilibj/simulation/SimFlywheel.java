/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.simulation;

import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.numbers.N1;

public class SimFlywheel extends SimLinearSystem<N1, N1, N1> {

  private final DCMotor m_motor;
  private final double m_gearing;

  /**
   * Create a LinearSystemSimulator. This simulator uses an {@link LinearSystem} to simulate
   * the state of the system. In simulationPeriodic, users should first set inputs from motors, update
   * the simulation and write simulated outputs to sensors.
   *
   * @param flywheelPlant      The flywheel system being controlled. This system can be created
   *                           using {@link edu.wpi.first.wpilibj.system.plant.LinearSystemId#createFlywheelSystem(DCMotor, double, double)}
   *                           or {@link edu.wpi.first.wpilibj.system.plant.LinearSystemId#identifyVelocitySystem(double, double)}
   * @param addNoise           If we should add noise to the measurement vector
   * @param measurementStdDevs Standard deviations of measurements. Can be null if addNoise is false.
   */
  public SimFlywheel(LinearSystem<N1, N1, N1> flywheelPlant, boolean addNoise, Matrix<N1, N1> measurementStdDevs,
                     DCMotor flywheelGearbox, double gearing) {
    super(flywheelPlant, addNoise, measurementStdDevs);
    this.m_motor = flywheelGearbox;
    this.m_gearing = gearing;
  }

  public double getFlywheelVelocityRadPerSec() {
    return getOutput(0);
  }

  public double getFlywheelVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(getOutput(0));
  }

  @Override
  public double getCurrentDrawAmps() {
    // I = V / R - omega / (Kv * R)
    // Reductions are output over input, so a reduction of 2:1 means the motor is spinning
    // 2x faster than the flywheel
    return m_u.get(0, 0) / m_motor.m_rOhms
      - getFlywheelVelocityRadPerSec() * m_gearing
      / (m_motor.m_KvRadPerSecPerVolt * m_motor.m_rOhms);
  }
}
