/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.estimator;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.SimpleMatrixUtils;
import edu.wpi.first.wpiutil.math.numbers.N1;

/**
 * Generates sigma points and weights according to Van der Merwe's 2004
 * dissertation[1] for the UnscentedKalmanFilter class.
 *
 * <p>It parametrizes the sigma points using alpha, beta, kappa terms, and is the
 * version seen in most publications. Unless you know better, this should be
 * your default choice.
 *
 * <p>States is the dimensionality of the state. 2*States+1 weights will be
 * generated.
 *
 * <p>[1] R. Van der Merwe "Sigma-Point Kalman Filters for Probabilitic
 * Inference in Dynamic State-Space Models" (Doctoral dissertation)
 */
public class MerweScaledSigmaPoints<S extends Num> {

  private final double m_alpha;
  private final int m_kappa;
  private final Nat<S> m_states;
  private Matrix m_wm;
  private Matrix m_wc;

  /**
   * Constructs a generator for Van der Merwe scaled sigma points.
   *
   * @param states an instance of Num that represents the number of states.
   * @param alpha  Determines the spread of the sigma points around the mean.
   *               Usually a small positive value (1e-3).
   * @param beta   Incorporates prior knowledge of the distribution of the mean.
   *               For Gaussian distributions, beta = 2 is optimal.
   * @param kappa  Secondary scaling parameter usually set to 0 or 3 - States.
   */
  public MerweScaledSigmaPoints(Nat<S> states, double alpha, double beta, int kappa) {
    this.m_states = states;
    this.m_alpha = alpha;
    this.m_kappa = kappa;

    computeWeights(beta);
  }

  /**
   * Constructs a generator for Van der Merwe scaled sigma points with default values for alpha,
   * beta, and kappa.
   *
   * @param states an instance of Num that represents the number of states.
   */
  public MerweScaledSigmaPoints(Nat<S> states) {
    this(states, 1e-3, 2, 3 - states.getNum());
  }

  /**
   * Returns number of sigma points for each variable in the state x.
   *
   * @return The number of sigma points for each variable in the state x.
   */
  public int getNumSigmas() {
    return 2 * m_states.getNum() + 1;
  }

  /**
   * Computes the sigma points for an unscented Kalman filter given the mean
   * (x) and covariance(P) of the filter.
   *
   * @param x An array of the means.
   * @param P Covariance of the filter.
   * @return Two dimensional array of sigma points. Each column contains all of
   *         the sigmas for one dimension in the problem space. Ordered by
   *         Xi_0, Xi_{1..n}, Xi_{n+1..2n}.
   */
  @SuppressWarnings({"ParameterName", "LocalVariableName"})
  public Matrix sigmaPoints(
          Matrix<S, N1> x,
          Matrix<S, S> P) {
    double lambda = Math.pow(m_alpha, 2) * (m_states.getNum() + m_kappa) - m_states.getNum();

    var intermediate = P.times(lambda + m_states.getNum()).getStorage();
    var U = SimpleMatrixUtils.lltDecompose(intermediate); // Upper triangular

    // 2 * states + 1 by states
    SimpleMatrix sigmas = new SimpleMatrix(2 * m_states.getNum() + 1, m_states.getNum());
    for (int i = 0; i < m_states.getNum(); i++) {
      sigmas.set(0, i, x.get(i, 0));
    }
    for (int k = 0; k < m_states.getNum(); k++) {
      var xT = x.transpose().getStorage();
      var xPlusU = xT.plus(U.extractVector(true, k));
      var xMinusU = xT.minus(U.extractVector(true, k));
      for (int i = 0; i < m_states.getNum(); i++) {
        sigmas.set(k + 1, i, xPlusU.get(0, i));
        sigmas.set(m_states.getNum() + k + 1, i, xMinusU.get(0, i));
      }
    }

    return new Matrix<>(sigmas);
  }

  /**
   * Computes the weights for the scaled unscented Kalman filter.
   *
   * @param beta Incorporates prior knowledge of the distribution of the mean.
   */
  @SuppressWarnings("LocalVariableName")
  private void computeWeights(double beta) {
    var wCBacking = new double[2 * m_states.getNum() + 1];
    var wMBacking = new double[2 * m_states.getNum() + 1];

    double lambda = Math.pow(m_alpha, 2) * (m_states.getNum() + m_kappa) - m_states.getNum();
    double c = 0.5 / (m_states.getNum() + lambda);

    var wM = new SimpleMatrix(1, 2 * m_states.getNum() + 1, true, wCBacking);
    var wC = new SimpleMatrix(1, 2 * m_states.getNum() + 1, true, wMBacking);
    wM.fill(c);
    wC.fill(c);

    wM.set(0, 0, lambda / (m_states.getNum() + lambda));
    wC.set(0, 0, lambda / (m_states.getNum() + lambda) + (1 - Math.pow(m_alpha, 2) + beta));

    this.m_wm = new Matrix(wM);
    this.m_wc = new Matrix(wC);
  }

  /**
   * Returns the weight for each sigma point for the mean.
   *
   * @return the weight for each sigma point for the mean.
   */
  public Matrix getWm() {
    return m_wm;
  }

  /**
   * Returns an element of the weight for each sigma point for the mean.
   *
   * @param element Element of vector to return.
   * @return the element i's weight for the mean.
   */
  public double getWm(int element) {
    return m_wm.get(0, element);
  }

  /**
   * Returns the weight for each sigma point for the covariance.
   *
   * @return the weight for each sigma point for the covariance.
   */
  public Matrix getWc() {
    return m_wc;
  }

  /**
   * Returns an element of the weight for each sigma point for the covariance.
   *
   * @param element Element of vector to return.
   * @return The element I's weight for the covariance.
   */
  public double getWc(int element) {
    return m_wc.get(0, element);
  }
}
