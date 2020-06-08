/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.controller;

import java.util.function.BiFunction;
import java.util.function.Function;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.system.NumericalJacobian;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.MatrixUtils;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.numbers.N1;

/**
 * Constructs a control-affine plant inversion model-based feedforward from 
 * given model dynamics.
 *
 * <p>If given the vector valued function as f(x, u) where x is the state
 * vector and u is the input vector, the B matrix(continuous input matrix)
 * is calculated through a {@link edu.wpi.first.wpilibj.system.NumericalJacobian}.
 * In this case f has to be control-affine (of the form f(x) + Bu).
 *
 * <p>The feedforward is calculated as
 * u_ff = B<sup>+</sup> (rDot - f(x)), were B<sup>+</sup> is the pseudoinverse
 * of B.
 *
 * <p>This feedforward does not account for a dynamic B matrix, B is either
 * determined or supplied when the feedforward is created and remains constant.
 *
 * <p>For more on the underlying math, read
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 */
@SuppressWarnings({"ParameterName", "LocalVariableName", "MemberName"})
public class ControlAffinePlantInversionFeedforward<S extends Num, I extends Num,
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
  private final Matrix<S, I> m_B;

  private final Nat<I> m_inputs;

  private final double m_dt;

  /**
   * The model dynamics.
   */
  private final BiFunction<Matrix<S, N1>, Matrix<I, N1>, Matrix<S, N1>> m_f;

  /**
   * Constructs a feedforward with given model dynamics as a function
   * of state and input.
   *
   * @param states    A {@link Nat} representing the number of states.
   * @param inputs    A {@link Nat} representing the number of inputs.
   * @param f         A vector-valued function of x, the state, and
   *                  u, the input, that returns the derivative of
   *                  the state vector. HAS to be control-affine
   *                  (of the form f(x) + Bu).
   * @param dtSeconds The timestep between calls of calculate().
   */
  public ControlAffinePlantInversionFeedforward(
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

    reset(m_r);
  }

  /**
   * Constructs a feedforward with given model dynamics as a function of state,
   * and the plant's B(continuous input matrix) matrix.
   *
   * @param states    A {@link Nat} representing the number of states.
   * @param inputs    A {@link Nat} representing the number of inputs.
   * @param f         A vector-valued function of x, the state,
   *                  that returns the derivative of the state vector.
   * @param B         Continuous input matrix of the plant being controlled.
   * @param dtSeconds The timestep between calls of calculate().
   */
  public ControlAffinePlantInversionFeedforward(
        Nat<S> states,
        Nat<I> inputs,
        Function<Matrix<S, N1>, Matrix<S, N1>> f,
        Matrix<S, I> B,
        double dtSeconds) {
    this.m_dt = dtSeconds;
    this.m_inputs = inputs;

    this.m_f = (x, u) -> f.apply(x);
    this.m_B = B;

    m_r = new Matrix<S, N1>(new SimpleMatrix(states.getNum(), 1));
    m_uff = new Matrix<I, N1>(new SimpleMatrix(inputs.getNum(), 1));

    reset(m_r);
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
   * uses the internally stored current reference.
   *
   * @param nextR The reference state of the future timestep(k + dt).
   *
   * @return The calculated feedforward.
   */
  public Matrix<I, N1> calculate(Matrix<S, N1> nextR) {
    return calculate(m_r, nextR);
  }

  /**
   * Calculate the feedforward with current and future reference vectors.
   *
   * @param r     The reference state of the current timestep(k).
   * @param nextR The reference state of the future timestep(k + dt).
   *
   * @return The calculated feedforward.
   */
  @SuppressWarnings({"ParameterName", "LocalVariableName"})
  public Matrix<I, N1> calculate(Matrix<S, N1> r, Matrix<S, N1> nextR) {
    var rDot = (nextR.minus(r)).div(m_dt);

    m_uff = new Matrix<>(m_B.getStorage()
            .solve(rDot.minus(m_f.apply(r, MatrixUtils.zeros(m_inputs))).getStorage()));

    m_r = nextR;
    return m_uff;
  }
}
