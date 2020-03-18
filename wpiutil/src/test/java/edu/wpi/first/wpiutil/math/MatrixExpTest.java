package edu.wpi.first.wpiutil.math;

import edu.wpi.first.wpiutil.math.numbers.N2;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;


public class MatrixExpTest {

  @Test
  public void testMatrixExp() {

    Matrix<N2, N2> mat = new MatBuilder<>(Nat.N2(), Nat.N2()).fill(1.2, 5.6, 3.0, 4.0);
    var exped = StateSpaceUtils.exp(mat);

    assertEquals(346.557, exped.get(0, 0), 0.1);
    assertEquals(661.735, exped.get(0, 1), 0.1);
    assertEquals(354.501, exped.get(1, 0), 0.1);
    assertEquals(677.425, exped.get(1, 1), 0.1);
  }

}
