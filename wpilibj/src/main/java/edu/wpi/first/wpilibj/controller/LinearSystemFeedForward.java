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
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.numbers.N1;

/**
 * Constructs a plant inversion model-based feedforward from a {@link LinearSystem}.
 *
 * <p>The feedforward is calculated as u_ff = B<sup>+</sup> (r_k+1 - A r_k), were B<sup>+</sup>
 * is the pseudoinverse of B.
 *
 * <p>For more on the underlying math, read
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 */
public class LinearSystemFeedForward<S extends Num, I extends Num,
        O extends Num> {
  /**
   * The current reference state.
   */
  @SuppressWarnings("MemberName")
  private Matrix<S, N1> m_r;

  /**
   * The computed feedforward.
   */
  private Matrix<I, N1> m_uff;

  private Matrix<S, I> m_discB;
  private Matrix<S, S> m_discA;

  /**
   * Constructs a feedforward with the given plant.
   *
   * @param plant     The plant being controlled.
   * @param dtSeconds Discretization timestep.
   */
  public LinearSystemFeedForward(
          LinearSystem<S, I, O> plant,
          double dtSeconds
  ) {
    this(plant.getA(), plant.getB(), dtSeconds);
  }

  /**
   * Constructs a feedforward with the given coefficients.
   *
   * @param A         Continuous system matrix of the plant being controlled.
   * @param B         Continuous input matrix of the plant being controlled.
   * @param dtSeconds Discretization timestep.
   */
  @SuppressWarnings({"ParameterName", "LocalVariableName"})
  public LinearSystemFeedForward(Matrix<S, S> A, Matrix<S, I> B,
                                  double dtSeconds) {

    var discABPair = StateSpaceUtil.discretizeAB(A, B, dtSeconds);
    this.m_discA = discABPair.getFirst();
    this.m_discB = discABPair.getSecond();
    
    m_r = new Matrix<S, N1>(new SimpleMatrix(B.getNumRows(), 1));
    m_uff = new Matrix<I, N1>(new SimpleMatrix(B.getNumCols(), 1));

    reset();
  }

  /**
   * Returns the previously calculated feedforward as an input vector.
   */
  public Matrix<I, N1> getUff() {
    return m_uff;
  }

  /**
   * Returns an element of the previously calculated feedforward.
   *
   * @param row Row of u.
   */
  public double getUff(int row) {
    return m_uff.get(row, 0);
  }

  /**
   * Returns the current reference vector r.
   */
  public Matrix<S, N1> getR() {
    return m_r;
  }

  /**
   * Returns an element of the current reference vector r.
   *
   * @param row Row of r.
   */
  public double getR(int row) {
    return m_r.get(row, 0);
  }

  /**
   * Resets the feedforward.
   */
  public void reset() {
    m_r.getStorage().fill(0.0);
    m_uff.getStorage().fill(0.0);
  }

  /**
   * Resets the feedforward with a specified initial state vector.
   */
  public void reset(Matrix<S, N1> initialState) {
    m_r = initialState;
    m_uff.getStorage().fill(0.0);
  }

  /**
   * Calculate the feedforward with only the future reference. This
   * uses the internally stored previous reference.
   *
   * @param nextR The future reference state of time k + dt.
   */
  public Matrix<I, N1> calculate(Matrix<S, N1> nextR) {
    m_uff = new Matrix<>(m_discB.getStorage()
              .solve((nextR.minus(m_discA.times(m_r))).getStorage()));
    m_r = nextR;

    return m_uff;
  }

  /**
   * Calculate the feedforward with current anf future reference vectors.
   * 
   * @param r The current reference state of time k.
   * @param nextR The future reference state of time k + dt.
   */
  @SuppressWarnings("ParameterName")
  public Matrix<I, N1> calculate(Matrix<S, N1> r, Matrix<S, N1> nextR) {
    m_r = r;
    m_uff = new Matrix<>(m_discB.getStorage()
            .solve((nextR.minus(m_discA.times(m_r))).getStorage()));
    m_r = nextR;
    return m_uff;
  }
}
