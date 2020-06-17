/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.simulation;

import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.RungeKutta;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

public class SimSingleJointedArm extends SimLinearSystem<N2, N1, N1> {

  @SuppressWarnings("MemberName")
  private final double m_r;
  private final double m_armMass;

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

  /**
   * Create a LinearSystemSimulator. This simulator uses an {@link LinearSystem} to simulate
   * the state of the system. In simulationPeriodic, users should first set inputs from motors, update
   * the simulation and write simulated outputs to sensors.
   *
   * @param armSystem            The arm system being controlled.
   * @param addNoise             If we should add noise to the measurement vector
   * @param m_measurementStdDevs Standard deviations of measurements. Can be null if addNoise is false.
   * @param armMassKg            The mass of the arm.
   * @param armLengthMeters      The distance from the pivot of the arm to its center of rotation.
   *                             This number is not the same as the overall length of the arm.
   */
  public SimSingleJointedArm(LinearSystem<N2, N1, N1> armSystem, boolean addNoise, Matrix<N1, N1> m_measurementStdDevs,
                             double armMassKg, double armLengthMeters) {
    super(armSystem, addNoise, m_measurementStdDevs);
    this.m_armMass = armMassKg;
    this.m_r = armLengthMeters;
  }
}
