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
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.VecBuilder;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class SimSwerveDriveTest {
  private final SimSwerveDrive m_sim = new SimSwerveDrive(50, 2.0,
      //      new SimSwerveDrive.SimSwerveModule(DCMotor.getNEO(1), 8.0,
      //          Units.inchesToMeters(2.0), new Translation2d(1, 1)),
      //      new SimSwerveDrive.SimSwerveModule(DCMotor.getNEO(1), 8.0,
      //          Units.inchesToMeters(2.0), new Translation2d(1, -1)),
      new SimSwerveDrive.SimSwerveModule(DCMotor.getNEO(1), 8.0,
          Units.inchesToMeters(2.0), new Translation2d(-1, 1)),
      new SimSwerveDrive.SimSwerveModule(DCMotor.getNEO(1), 8.0,
          Units.inchesToMeters(2.0), new Translation2d(-1, -1)));

  @Test
  public void testForceCalculation() {
    var module = m_sim.getModules()[0];
    var force = module.getModuleForceNewtons(0, 12, Rotation2d.fromDegrees(-45));
    // f = a when mass = 1kg
    // xdot = [vel, accel] = 0x + B u
    var system = LinearSystemId.createElevatorSystem(DCMotor.getNEO(1), 1.0, Units.inchesToMeters(2.0), 8);
    var refForce = system.getB().times(VecBuilder.fill(12));

    assertEquals(refForce.get(1, 0) * 1 / Math.sqrt(2), force.getX(), 1e-4);
    assertEquals(-refForce.get(1, 0) * 1 / Math.sqrt(2), force.getY(), 1e-4);
  }

  @Test
  public void testSimulationDrivingForward() {

    // v = r = sqrt(2)

    var desiredSpeeds =
        m_sim.getKinematics().toSwerveModuleStates(new ChassisSpeeds(1, 2, 0));

    var chassisSpeedList = new ArrayList<ChassisSpeeds>();

    for (int i = 0; i < 200; i++) {
      m_sim.setModuleVoltages(2, 2); //2, 2, 2);

      m_sim.setModuleAngles(desiredSpeeds[0].angle,
          desiredSpeeds[1].angle); //, desiredSpeeds[2].angle//,
      //          desiredSpeeds[3].angle
      //);

      m_sim.update(0.020);

      chassisSpeedList.add(m_sim.getSpeeds());
    }

    assertEquals(m_sim.getModules().length, 2);

    //    XYChart chart = new XYChartBuilder().build();
    //    chart.addSeries("vx, mps", chassisSpeedList.stream().map(it -> it.vxMetersPerSecond).collect(Collectors.toList()));
    //    chart.addSeries("vy. mps", chassisSpeedList.stream().map(it -> it.vyMetersPerSecond).collect(Collectors.toList()));
    //    chart.addSeries("omega, rad/s", chassisSpeedList.stream().map(it -> it.omegaRadiansPerSecond).collect(Collectors.toList()));

    //    try {
    //      new SwingWrapper<>(chart).displayChart();
    //      Thread.sleep(10000000);
    //    } catch (InterruptedException e) {
    //      e.printStackTrace()
    //    }
  }

}
