/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import java.util.function.BiFunction;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.math.Discretization;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.NumericalJacobian;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.MatrixUtils;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.numbers.N1;

/**
 * Constructs a plant inversion model-based feedforward from a {@link LinearSystem}.
 *
 * <p>The feedforward is calculated as u_ff = B<sup>+</sup> (r_k+1 - A r_k), were B<sup>+</sup>
 * is the pseudoinverse of B.
 *
 * <p>The feedforward has an overload for model dynamics and calculates B
 * through a {@link edu.wpi.first.wpilibj.system.NumericalJacobian}.
 * With the dynamics, the feedforward is calculated as
 * u_ff = B<sup>+</sup> (rDot - f(x)), were B<sup>+</sup> is the pseudoinverse of B.
 *
 * <p>For more on the underlying math, read
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 */
@SuppressWarnings({"ParameterName", "LocalVariableName", "MemberName"})
public class PlantInversionFeedforward<S extends Num, I extends Num,
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

  @SuppressWarnings("MemberName")
  private Matrix<S, I> m_B;

  @SuppressWarnings("MemberName")
  private Matrix<S, S> m_A;

  private Nat<I> m_inputs;

  private double m_dt;

  /**
   * The model dynamics, if the overload is used.
   */
  private BiFunction<Matrix<S, N1>, Matrix<I, N1>, Matrix<S, N1>> m_f;

  /**
   * Constructs a feedforward with the given plant.
   *
   * @param plant     The plant being controlled.
   * @param dtSeconds Discretization timestep.
   */
  public PlantInversionFeedforward(
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
  public PlantInversionFeedforward(Matrix<S, S> A, Matrix<S, I> B,
                                   double dtSeconds) {
    this.m_dt = dtSeconds;

    var discABPair = Discretization.discretizeAB(A, B, dtSeconds);
    this.m_A = discABPair.getFirst();
    this.m_B = discABPair.getSecond();

    m_r = new Matrix<S, N1>(new SimpleMatrix(B.getNumRows(), 1));
    m_uff = new Matrix<I, N1>(new SimpleMatrix(B.getNumCols(), 1));

    reset();
  }

  /**
   * Constructs a feedforward with given model dynamics.
   *
   * @param states    A {@link Nat} representing the number of states.
   * @param inputs    A {@link Nat} representing the number of inputs.
   * @param f         A vector-valued function of x, the state, and
   *                  u, the input, that returns the derivative of
   *                  the state vector.
   * @param dtSeconds The timestep between calls of calculate().
   */
  public PlantInversionFeedforward(
        Nat<S> states,
        Nat<I> inputs,
        BiFunction<Matrix<S, N1>, Matrix<I, N1>, Matrix<S, N1>> f,
        double dtSeconds) {
    this.m_dt = dtSeconds;
    this.m_f = f;
    this.m_inputs = inputs;


    this.m_B = NumericalJacobian.numericalJacobianU(states, inputs,
            m_f, MatrixUtils.zeros(states), MatrixUtils.zeros(inputs));

    m_r = new Matrix<S, N1>(new SimpleMatrix(states.getNum(), 1));
    m_uff = new Matrix<I, N1>(new SimpleMatrix(inputs.getNum(), 1));

    reset();
  }


  /**
   * Returns the previously calculated feedforward as an input vector.
   *
   * @return The calculated feedforward.
   */
  public Matrix<I, N1> getUff() {
    return m_uff;
  }

  /**
   * Returns an element of the previously calculated feedforward.
   *
   * @param row Row of uff.
   *
   * @return The row of the calculated feedforward.
   */
  public double getUff(int row) {
    return m_uff.get(row, 0);
  }

  /**
   * Returns the current reference vector r.
   *
   * @return The current reference vector.
   */
  public Matrix<S, N1> getR() {
    return m_r;
  }

  /**
   * Returns an element of the current reference vector r.
   *
   * @param row Row of r.
   *
   * @return The row of the current reference vector.
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
   *
   * @param initialState The initial state vector.
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
   *
   * @return The calculated feedforward.
   */
  public Matrix<I, N1> calculate(Matrix<S, N1> nextR) {
    return calculate(m_r, nextR);
  }

  /**
   * Calculate the feedforward with current anf future reference vectors.
   *
   * @param r The current reference state of time k.
   * @param nextR The future reference state of time k + dt.
   *
   * @return The calculated feedforward.
   */
  @SuppressWarnings({"ParameterName", "LocalVariableName"})
  public Matrix<I, N1> calculate(Matrix<S, N1> r, Matrix<S, N1> nextR) {
    if (m_f == null) {
      m_uff = new Matrix<>(m_B.getStorage()
              .solve((nextR.minus(m_A.times(r))).getStorage()));
    } else {
      var rDot = (nextR.minus(r)).div(m_dt);

      m_uff = new Matrix<>(m_B.getStorage()
              .solve(rDot.minus(m_f.apply(m_r, MatrixUtils.zeros(m_inputs))).getStorage()));
    }
    m_r = nextR;
    return m_uff;
  }
}
