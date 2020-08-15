/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.simulation;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.VecBuilder;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class SimElevatorTest {
  @Test
  @SuppressWarnings("LocalVariableName")
  public void testStateSpaceSimWithElevator() {
    var plant = LinearSystemId.createElevatorSystem(
        DCMotor.getVex775Pro(4),
        8.0,
        0.75 * 25.4 / 1000.0,
        14.67);

//    var controller = new PIDController(10, 0, 0);
    var controller = new LinearQuadraticRegulator<>(plant, VecBuilder.fill(0.01, 1.0),
      VecBuilder.fill(12.0), 0.020);

    var sim = new SimElevator(plant, true, VecBuilder.fill(0.01), DCMotor.getVex775Pro(4),
      14.67, 0.75 * 25.4 / 1000.0);

    var motor = new PWMVictorSPX(0);
    var simPos = 0.0;
    var simVel = 0.0;

    for (int i = 0; i < 100; i++) {

      double nextVoltage = controller.calculate(VecBuilder.fill(simPos, simVel), VecBuilder.fill(2.0, 0.0))
        .get(0, 0);

      motor.setVoltage(nextVoltage);
      RoboRioSim.setVInVoltage(SimBattery.calculateLoadedBatteryVoltage(sim.getCurrentDrawAmps()));

      // ------ SimulationPeriodic() happens after user code -------

      double currentBatteryVoltage = RobotController.getBatteryVoltage();
      var u = VecBuilder.fill(motor.get() * currentBatteryVoltage);
      sim.setInput(u);
      sim.update(0.020);
      simPos = sim.getElevatorPositionMeters();
      simVel = sim.getElevatorVelocityMetersPerSecond();

       System.out.println(motor.get() * currentBatteryVoltage + ", " + simPos);
    }

    assertEquals(controller.getR(0), sim.getOutput(0), 0.2);
  }
}
