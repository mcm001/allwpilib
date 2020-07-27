/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;

public class MerweScaledSigmaPointsTest {
  @Test
  public void testZeroMeanPoints() {
    var merweScaledSigmaPoints = new MerweScaledSigmaPoints<>(Nat.N2());
    var points = merweScaledSigmaPoints.sigmaPoints(VecBuilder.fill(0, 0),
          Matrix.mat(Nat.N2(), Nat.N2()).fill(1, 0, 0, 1));

    assertEquals(points, Matrix.mat(Nat.N2(), Nat.N5()).fill(
            0.0, 0.00173205, 0.0, -0.00173205, 0.0,
            0.0, 0.0, 0.00173205, 0.0, -0.00173205
    ));
  }

  @Test
  public void testNonzeroMeanPoints() {
    var merweScaledSigmaPoints = new MerweScaledSigmaPoints<>(Nat.N2());
    var points = merweScaledSigmaPoints.sigmaPoints(VecBuilder.fill(1, 2),
          Matrix.mat(Nat.N2(), Nat.N2()).fill(1, 0, 0, 10));

    assertEquals(points, Matrix.mat(Nat.N2(), Nat.N5()).fill(
            1.0, 1.00173205, 1.0, 0.99826795, 1.0,
            2.0, 2.0, 2.00547723, 2.0, 1.99452277
    ));
  }
}
