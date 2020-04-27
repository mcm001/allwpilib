/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.system;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Nat;

import static org.ejml.EjmlUnitTests.assertEquals;

class LinearSystemTest {
  @Test
  public void testDrivetrainVelcitySystem() {
    var model = LinearSystem.createDrivetrainVelocitySystem(
            DCMotor.getNEO(4), 70, 0.05, 0.4, 6.0, 6, 12
    );
    assertEquals(model.getA().getStorage().getDDRM(), new MatBuilder<>(Nat.N2(),
            Nat.N2()).fill(-10.14132, 3.06598, 3.06598, -10.14132).getStorage().getDDRM(), 0.001);

    assertEquals(model.getB().getStorage().getDDRM(), new MatBuilder<>(Nat.N2(),
            Nat.N2()).fill(4.2590, -1.28762, -1.2876, 4.2590).getStorage().getDDRM(), 0.001);

    assertEquals(model.getC().getStorage().getDDRM(), new MatBuilder<>(Nat.N2(),
            Nat.N2()).fill(1.0, 0.0, 0.0, 1.0).getStorage().getDDRM(), 0.001);

    assertEquals(model.getD().getStorage().getDDRM(), new MatBuilder<>(Nat.N2(),
            Nat.N2()).fill(0.0, 0.0, 0.0, 0.0).getStorage().getDDRM(), 0.001);

    Assertions.assertEquals(12.0, model.getUMax(0), 0.001);
    Assertions.assertEquals(-12.0, model.getUMin(0), 0.001);
  }

  @Test
  public void testElevatorSystem() {

    var model = LinearSystem.createElevatorSystem(DCMotor.getNEO(2), 5, 0.05, 12, 12);
    assertEquals(model.getA().getStorage().getDDRM(), new MatBuilder<>(Nat.N2(),
            Nat.N2()).fill(0, 1, 0, -99.05473).getStorage().getDDRM(), 0.001);

    assertEquals(model.getB().getStorage().getDDRM(), new MatBuilder<>(Nat.N2(),
            Nat.N1()).fill(0, 20.8).getStorage().getDDRM(), 0.001);

    assertEquals(model.getC().getStorage().getDDRM(), new MatBuilder<>(Nat.N1(),
            Nat.N2()).fill(1, 0).getStorage().getDDRM(), 0.001);

    assertEquals(model.getD().getStorage().getDDRM(), new MatBuilder<>(Nat.N1(),
            Nat.N1()).fill(0).getStorage().getDDRM(), 0.001);

    Assertions.assertEquals(12.0, model.getUMax(0), 0.001);
    Assertions.assertEquals(-12.0, model.getUMin(0), 0.001);
  }

  @Test
  public void testFlywheelSystem() {
    var model = LinearSystem.createFlywheelSystem(DCMotor.getNEO(2), 0.00032, 1.0,  12);
    assertEquals(model.getA().getStorage().getDDRM(), new MatBuilder<>(Nat.N1(),
            Nat.N1()).fill(-26.87032).getStorage().getDDRM(), 0.001);

    assertEquals(model.getB().getStorage().getDDRM(), new MatBuilder<>(Nat.N1(),
            Nat.N1()).fill(1354.166667).getStorage().getDDRM(), 0.001);

    assertEquals(model.getC().getStorage().getDDRM(), new MatBuilder<>(Nat.N1(),
            Nat.N1()).fill(1).getStorage().getDDRM(), 0.001);

    assertEquals(model.getD().getStorage().getDDRM(), new MatBuilder<>(Nat.N1(),
            Nat.N1()).fill(0).getStorage().getDDRM(), 0.001);

    Assertions.assertEquals(12.0, model.getUMax(0), 0.001);
    Assertions.assertEquals(-12.0, model.getUMin(0), 0.001);
  }

  @Test
  public void testIdentifyPositionSystem() {
    // By controls engineering in frc,
    // x-dot = [0 1 | 0 -kv/ka] x = [0 | 1/ka] u
    var kv = 1.0;
    var ka = 0.5;
    var model = LinearSystem.identifyPositionSystem(kv, ka, 12.0);

    Assertions.assertEquals(12.0, model.getUMax(0), 0.001);
    Assertions.assertEquals(-12.0, model.getUMin(0), 0.001);

    assertEquals(model.getA().getStorage().getDDRM(), new SimpleMatrix(2, 2, true,
          new double[] { 0, 1, 0, -kv / ka }).getDDRM());
    assertEquals(model.getB().getStorage().getDDRM(), new SimpleMatrix(2, 1, true,
          new double[] { 0, 1 / ka }).getDDRM());
  }

  @Test
  public void testIdentifyVelocitySystem() {
    // By controls engineering in frc,
    // V = kv * velocity + ka * acceleration
    // x-dot =  -kv/ka * v + 1/ka \cdot V
    var kv = 1.0;
    var ka = 0.5;
    var model = LinearSystem.identifyVelocitySystem(kv, ka, 12.0);

    Assertions.assertEquals(12.0, model.getUMax(0), 0.001);
    Assertions.assertEquals(-12.0, model.getUMin(0), 0.001);

    assertEquals(model.getA().getStorage().getDDRM(), new SimpleMatrix(1, 1, true,
          new double[] { -kv / ka }).getDDRM());
    assertEquals(model.getB().getStorage().getDDRM(), new SimpleMatrix(1, 1, true,
          new double[] { 1 / ka }).getDDRM());
  }
}
