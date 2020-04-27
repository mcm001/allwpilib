package edu.wpi.first.wpilibj.simulation;

import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.VecBuilder;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class ArmSimTest {

    SingleJointedArmSim sim = new SingleJointedArmSim(
        DCMotor.getVex775Pro(2),
        100,
        10 / 2.2,
        Units.inchesToMeters(19.0/2.0),
        -Math.PI,
        0.0, VecBuilder.fill(0.0)
    );

    @Test
    public void testArmDisabled() {
        sim.resetState(VecBuilder.fill(0.0, 0.0));

        for (int i = 0; i < 12 / 0.02; i++) {
            sim.setInput(0.0);
            sim.update(0.020);
        }

        // the arm should swing down
        assertEquals(-Math.PI / 2.0, sim.getArmAngleRads(), 0.1);
    }
}
