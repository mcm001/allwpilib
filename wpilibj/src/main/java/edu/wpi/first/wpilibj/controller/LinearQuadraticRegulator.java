package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj.math.StateSpaceUtils;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.Drake;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.SimpleMatrixUtils;
import edu.wpi.first.wpiutil.math.numbers.N1;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;

/**
 * Contains the controller coefficients and logic for a linear-quadratic
 * regulator (LQR).
 * LQRs use the control law u = K(r - x). The feedforward uses an inverted plant
 * and is calculated as u_ff = B<sup>+</sup> (r_k+1 - A r_k), were B<sup>+</sup>
 * is the pseudoinverse of B.
 *
 * <p>For more on the underlying math, read
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 */
public class LinearQuadraticRegulator<S extends Num, I extends Num,
        O extends Num> {

  @SuppressWarnings("MemberName")
  private final Matrix<S, S> m_A;

  @SuppressWarnings("MemberName")
  private final Matrix<S, I> m_B;

  /**
   * If the controller is enabled. Defaults to false.
   */
  private boolean m_enabled;

  /**
   * The current reference state.
   */
  @SuppressWarnings("MemberName")
  private Matrix<S, N1> m_r;

  /**
   * The computed and capped controller output.
   */
  @SuppressWarnings("MemberName")
  private Matrix<I, N1> m_u;

  /**
   * The computed feedforward.
   */
  private Matrix<I, N1> m_uff;

  // Controller gain.
  @SuppressWarnings("MemberName")
  private Matrix<I, S> m_K;

  private Matrix<S, I> m_discB;
  private Matrix<S, S> m_discA;

  /**
   * Constructs a controller with the given coefficients and plant.
   *
   * @param states    a Nat representing the number of states.
   * @param inputs    a Nat representing the number of inputs.
   * @param plant     The plant being controlled.
   * @param qelms     The maximum desired error tolerance for each state.
   * @param relms     The maximum desired control effort for each input.
   * @param dtSeconds Discretization timestep.
   */
  public LinearQuadraticRegulator(
          Nat<S> states, Nat<I> inputs,
          LinearSystem<S, I, O> plant,
          Matrix<S, N1> qelms,
          Matrix<I, N1> relms,
          double dtSeconds
  ) {
    this(states, inputs, plant.getA(), plant.getB(), qelms, relms, dtSeconds);
  }

  /**
   * Constructs a controller with the given coefficients and plant.
   *
   * @param states    a Nat representing the number of states.
   * @param inputs    a Nat representing the number of inputs.
   * @param A         Continuous system matrix of the plant being controlled.
   * @param B         Continuous input matrix of the plant being controlled.
   * @param qelms     The maximum desired error tolerance for each state.
   * @param relms     The maximum desired control effort for each input.
   * @param dtSeconds Discretization timestep.
   */
  @SuppressWarnings("ParameterName")
  public LinearQuadraticRegulator(
          Nat<S> states, Nat<I> inputs,
          Matrix<S, S> A, Matrix<S, I> B,
          Matrix<S, N1> qelms, Matrix<I, N1> relms,
          double dtSeconds
  ) {
    this.m_A = A;
    this.m_B = B;

    var size = states.getNum() + inputs.getNum();
    @SuppressWarnings("LocalVariableName")
    var Mcont = new SimpleMatrix(0, 0);
    var scaledA = m_A.times(dtSeconds);
    var scaledB = m_B.times(dtSeconds);
    Mcont = Mcont.concatColumns(scaledA.getStorage());
    Mcont = Mcont.concatColumns(scaledB.getStorage());
    // so our Mcont is now states x (states + inputs)
    // and we want (states + inputs) x (states + inputs)
    // so we want to add (inputs) many rows onto the bottom
    Mcont = Mcont.concatRows(new SimpleMatrix(inputs.getNum(), size));

    // calculate discrete A and B matrices
    SimpleMatrix Mstate = StateSpaceUtils.exp(Mcont);

    var discA = new SimpleMatrix(states.getNum(), states.getNum());
    var discB = new SimpleMatrix(states.getNum(), inputs.getNum());
    CommonOps_DDRM.extract(Mstate.getDDRM(), 0, 0, discA.getDDRM());
    CommonOps_DDRM.extract(Mstate.getDDRM(), 0, states.getNum(), discB.getDDRM());

    // make the cost matrices
    @SuppressWarnings("LocalVariableName")
    var Q = StateSpaceUtils.makeCostMatrix(states, qelms);
    var R = StateSpaceUtils.makeCostMatrix(inputs, relms);

    this.m_discB = new Matrix<>(discB);
    this.m_discA = new Matrix<>(discA);

    var S = Drake.discreteAlgebraicRiccatiEquation(discA, discB, Q.getStorage(), R.getStorage());

    m_K = new Matrix<>((discB.transpose().mult(S).mult(discB).plus(R.getStorage())).invert()
            .mult(discB.transpose()).mult(S).mult(discA)); // TODO (HIGH) SWITCH ALGORITHMS

    reset();
  }

  /**
   * Constructs a controller with the given coefficients and plant.
   *
   * @param states    a Nat representing the number of states.
   * @param inputs    a Nat representing the number of inputs.
   * @param A         Continuous system matrix of the plant being controlled.
   * @param B         Continuous input matrix of the plant being controlled.
   * @param k         the controller matrix K to use.
   * @param dtSeconds Discretization timestep.
   */
  @SuppressWarnings("ParameterName")
  public LinearQuadraticRegulator(
          Nat<S> states, Nat<I> inputs,
          Matrix<S, S> A, Matrix<S, I> B,
          Matrix<I, S> k,
          double dtSeconds
  ) {
    this.m_A = A;
    this.m_B = B;

    Matrix<S, S> discA = new Matrix<>(new SimpleMatrix(states.getNum(), states.getNum()));
    Matrix<S, I> discB = new Matrix<>(new SimpleMatrix(states.getNum(), inputs.getNum()));
    StateSpaceUtils.discretizeAB(states, inputs, A, B, dtSeconds, discA, discB);
    this.m_discA = discA;
    this.m_discB = discB;

    m_K = k;

    this.m_r = new Matrix<>(new SimpleMatrix(states.getNum(), 1));
    this.m_u = new Matrix<>(new SimpleMatrix(inputs.getNum(), 1));
  }

  /**
   * Returns the control input vector u.
   */
  public Matrix<I, N1> getU() {
    return m_u;
  }

  /**
   * Returns the feedforward component of the control input vector u.
   */
  public Matrix<I, N1> getUff() {
    return m_uff;
  }

  /**
   * Returns an element of the feedforward component of the control input vector
   * u.
   *
   * @param row Row of u.
   */
  public double getUff(int row) {
    return m_uff.get(row, 0);
  }

  /**
   * Returns the reference vector r.
   */
  public Matrix<S, N1> getR() {
    return m_r;
  }

  /**
   * Returns an element of the reference vector r.
   *
   * @param row Row of r.
   */
  public double getR(int row) {
    return m_r.get(row, 0);
  }

  /**
   * Enables the controller.
   */
  public void enable() {
    m_enabled = true;
  }

  /**
   * Disables controller and zeros controller output U.
   */
  public void disable() {
    m_enabled = false;
    m_u.getStorage().fill(0.0);
  }

  public boolean isEnabled() {
    return m_enabled;
  }

  /**
   * Returns the controller matrix K.
   *
   * @return the controller matrix K.
   */
  public Matrix<I, S> getK() {
    return m_K;
  }

  /**
   * Resets the controller.
   */
  public void reset() {
    m_r.getStorage().fill(0.0);
    m_u.getStorage().fill(0.0);
    m_uff.getStorage().fill(0.0);
  }

  /**
   * Update controller without setting a new reference.
   *
   * @param x The current state x.
   */
  @SuppressWarnings("ParameterName")
  public void update(Matrix<S, N1> x) {
    if (m_enabled) {
      m_uff = new Matrix<>(SimpleMatrixUtils.householderQrDecompose(m_discB.getStorage())
              .solve((m_r.minus(m_discA.times(m_r))).getStorage()));
      m_u = m_K.times(m_r.minus(x)).plus(m_uff);
    }
  }

  /**
   * Set a new reference and update the controller.
   *
   * @param x     The current state x.
   * @param nextR the next reference vector r.
   */
  @SuppressWarnings("ParameterName")
  public void update(Matrix<S, N1> x, Matrix<S, N1> nextR) {
    if (m_enabled) {
      Matrix<I, N1> feedBack = m_K.times(m_r.minus(x));
      m_uff = new Matrix<>(SimpleMatrixUtils.householderQrDecompose(m_discB.getStorage())
              .solve((nextR.minus(m_discA.times(m_r))).getStorage()));

      m_u = feedBack.plus(m_uff);
      m_r = nextR;
    }
  }

}