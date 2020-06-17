/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.simulation;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
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

    var controller = new PIDController(10, 0, 0);

    var sim = new SimLinearSystem<>(plant, true, VecBuilder.fill(0.01));

    var motor = new PWMVictorSPX(0);
    double simPos = 0.0;

    for (int i = 0; i < 100; i++) {
      controller.setSetpoint(2.0);

      double nextVoltage = controller.calculate(simPos);

      double currentBatteryVoltage = RobotController.getBatteryVoltage();
      motor.set(nextVoltage / currentBatteryVoltage);

      // ------ SimulationPeriodic() happens after user code -------

      var u = VecBuilder.fill(motor.get() * currentBatteryVoltage);
      sim.setInput(u);
      sim.update(0.020);
      var y = sim.getOutput();
      simPos = y.get(0, 0);

      // System.out.println(motor.get() * 12.0 + ", " + simPos );
    }

    assertEquals(controller.getSetpoint(), sim.getOutput(0), 0.2);
  }
}
