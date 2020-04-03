/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.math.StateSpaceUtil;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.Drake;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.numbers.N1;

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
   * @param plant     The plant being controlled.
   * @param qelms     The maximum desired error tolerance for each state.
   * @param relms     The maximum desired control effort for each input.
   * @param dtSeconds Discretization timestep.
   */
  public LinearQuadraticRegulator(
          LinearSystem<S, I, O> plant,
          Matrix<S, N1> qelms,
          Matrix<I, N1> relms,
          double dtSeconds
  ) {
    this(plant.getA(), plant.getB(), qelms, relms, dtSeconds);
  }

  /**
   * Constructs a controller with the given coefficients and plant.
   *
   * @param A         Continuous system matrix of the plant being controlled.
   * @param B         Continuous input matrix of the plant being controlled.
   * @param qelms     The maximum desired error tolerance for each state.
   * @param relms     The maximum desired control effort for each input.
   * @param dtSeconds Discretization timestep.
   */
  @SuppressWarnings({"ParameterName", "LocalVariableName"})
  public LinearQuadraticRegulator(Matrix<S, S> A, Matrix<S, I> B,
                                  Matrix<S, N1> qelms, Matrix<I, N1> relms,
                                  double dtSeconds
  ) {
    this.m_A = A;
    this.m_B = B;

    var discABPair = StateSpaceUtil.discretizeAB(m_A, m_B, dtSeconds);

    // make the cost matrices
    var Q = StateSpaceUtil.makeCostMatrix(qelms);
    var R = StateSpaceUtil.makeCostMatrix(relms);

    this.m_discA = discABPair.getFirst();
    this.m_discB = discABPair.getSecond();

    var S = Drake.discreteAlgebraicRiccatiEquation(m_discA, m_discB, Q, R);

    var temp = m_discB.getStorage().transpose().mult(S).mult(m_discB.getStorage())
            .plus(R.getStorage());
    m_K = new Matrix<>(temp.solve(m_discB.getStorage().transpose().mult(S)
            .mult(m_discA.getStorage()))); // Eigen: m_k = temp.llt().solve(toSolve)

    initializeRandU(B.getNumRows(), B.getNumCols());
    reset();
  }

  /**
   * Constructs a controller with the given coefficients and plant.
   *
   * @param A         Continuous system matrix of the plant being controlled.
   * @param B         Continuous input matrix of the plant being controlled.
   * @param k         the controller matrix K to use.
   * @param dtSeconds Discretization timestep.
   */
  @SuppressWarnings("ParameterName")
  public LinearQuadraticRegulator(
          Matrix<S, S> A, Matrix<S, I> B,
          Matrix<I, S> k,
          double dtSeconds
  ) {
    this.m_A = A;
    this.m_B = B;

    var discABpair = StateSpaceUtil.discretizeAB(A, B, dtSeconds);
    this.m_discA = discABpair.getFirst();
    this.m_discB = discABpair.getSecond();

    m_K = k;

    initializeRandU(m_B.getNumRows(), m_B.getNumCols());
    reset();
  }

  private void initializeRandU(int states, int inputs) {
    m_r = new Matrix<>(new SimpleMatrix(states, 1));
    m_u = new Matrix<>(new SimpleMatrix(inputs, 1));
    m_uff = new Matrix<>(new SimpleMatrix(inputs, 1));
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
      m_uff = new Matrix<>(m_discB.getStorage()
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
    update(x);
    m_r = nextR;
  }
}
