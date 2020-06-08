/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

import static org.junit.jupiter.api.Assertions.assertEquals;

class NonlinearPlantInversionFeedforwardTest {
  @SuppressWarnings("LocalVariableName")
  @Test
  void testCalculate() {
    NonlinearPlantInversionFeedforward<N2, N1, N1> feedforward =
            new NonlinearPlantInversionFeedforward<N2, N1, N1>(
                    Nat.N2(),
                    Nat.N1(),
                    this::getDynamics,
                    0.02);

    assertEquals(48.0, feedforward.calculate(
         new MatBuilder<>(Nat.N2(), Nat.N1()).fill(2, 2),
         new MatBuilder<>(Nat.N2(), Nat.N1()).fill(3, 3)).get(0, 0),
         1e-6);
  }

  @SuppressWarnings("LocalVariableName")
  @Test
  void testCalculateState() {
    Matrix<N2, N1> B = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(0, 1);

    NonlinearPlantInversionFeedforward<N2, N1, N1> feedforward =
            new NonlinearPlantInversionFeedforward<N2, N1, N1>(
                    Nat.N2(),
                    Nat.N1(),
                    this::getStateDynamics,
                    B,
                    0.02);

    assertEquals(48.0, feedforward.calculate(
            new MatBuilder<>(Nat.N2(), Nat.N1()).fill(2, 2),
            new MatBuilder<>(Nat.N2(), Nat.N1()).fill(3, 3)).get(0, 0),
            1e-6);
  }

  @SuppressWarnings("ParameterName")
  protected Matrix<N2, N1> getDynamics(Matrix<N2, N1> x, Matrix<N1, N1> u) {
    var result = new Matrix<N2, N1>(new SimpleMatrix(2, 1));

    result = new MatBuilder<>(Nat.N2(), Nat.N2()).fill(1.000, 0, 0, 1.000).times(x)
            .plus(new MatBuilder<>(Nat.N2(), Nat.N1()).fill(0, 1).times(u));

    return result;
  }

  @SuppressWarnings("ParameterName")
  protected Matrix<N2, N1> getStateDynamics(Matrix<N2, N1> x) {
    var result = new Matrix<N2, N1>(new SimpleMatrix(2, 1));

    result = new MatBuilder<>(Nat.N2(), Nat.N2()).fill(1.000, 0, 0, 1.000).times(x);

    return result;
  }
}
