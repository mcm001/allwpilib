/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.math;

import java.util.Random;

import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.qr.QRDecompositionHouseholder_DDRM;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpiutil.WPIUtilJNI;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.SimpleMatrixUtils;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N3;

@SuppressWarnings({"PMD.TooManyMethods", "ParameterName"})
public final class StateSpaceUtil {
  private StateSpaceUtil() {
    // Utility class
  }

  /**
   * Creates a covariance matrix from the given vector for use with Kalman
   * filters.
   *
   * <p>Each element is squared and placed on the covariance matrix diagonal.
   *
   * @param <S>     Num representing the states of the system.
   * @param states  A Nat representing the states of the system.
   * @param stdDevs For a Q matrix, its elements are the standard deviations of
   *                each state from how the model behaves. For an R matrix, its
   *                elements are the standard deviations for each output
   *                measurement.
   * @return Process noise or measurement noise covariance matrix.
   */
  public static <S extends Num> Matrix<S, S> makeCovMatrix(
          Nat<S> states, Matrix<S, N1> stdDevs
  ) {
    var result = new Matrix<S, S>(new SimpleMatrix(states.getNum(), states.getNum()));
    for (int i = 0; i < states.getNum(); i++) {
      result.set(i, i, Math.pow(stdDevs.get(i, 0), 2));
    }
    return result;
  }

  /**
   * Creates a vector of normally distributed white noise with the given noise
   * intensities for each element.
   *
   * @param <N>     Num representing the dimensionality of  the noise vector to create.
   * @param rows    A Nat representing the dimensionality of  the noise vector to create.
   * @param stdDevs A matrix whose elements are the standard deviations of each
   *                element of the noise vector.
   * @return White noise vector.
   */
  public static <N extends Num> Matrix<N, N1> makeWhiteNoiseVector(
          Nat<N> rows, Matrix<N, N1> stdDevs
  ) {
    var rand = new Random();

    Matrix<N, N1> result = new Matrix<>(new SimpleMatrix(rows.getNum(), 1));
    for (int i = 0; i < rows.getNum(); i++) {
      result.set(i, 0, rand.nextGaussian() * stdDevs.get(i, 0));
    }
    return result;
  }

  /**
   * Returns a discretized version of the provided continuous process noise
   * covariance matrix.
   *
   * @param <S>       A Num representing the number of states.
   * @param states    A Nat representing the number of states.
   * @param A         The system matrix A.
   * @param Q         Continuous process noise covariance matrix.
   * @param dtSeconds Discretization timestep.
   * @return The discretized process noise covariance matrix.
   */
  @SuppressWarnings("ParameterName")
  public static <S extends Num> Matrix<S, S> discretizeProcessNoiseCov(
          Nat<S> states, Matrix<S, S> A, Matrix<S, S> Q, double dtSeconds) {

    var gain = new SimpleMatrix(0, 0);

    // Set up the matrix M = [[-A, Q], [0, A.T]]
    gain = gain.concatColumns(
            (A.times(-1)).getStorage().concatRows(new SimpleMatrix(states.getNum(),
                    states.getNum())), Q.getStorage().concatRows(A.transpose().getStorage())
    );

    var phi = SimpleMatrixUtils.expm(gain.scale(dtSeconds));

    // Phi12 = phi[0:States,        States:2*States]
    // Phi22 = phi[States:2*States, States:2*States]
    Matrix<S, S> phi12 = new Matrix<>(new SimpleMatrix(states.getNum(), states.getNum()));
    Matrix<S, S> phi22 = new Matrix<>(new SimpleMatrix(states.getNum(), states.getNum()));
    CommonOps_DDRM.extract(
            phi.getDDRM(), 0, states.getNum(), states.getNum(), states.getNum(),
            phi12.getStorage().getDDRM()
    );
    CommonOps_DDRM.extract(
            phi.getDDRM(), states.getNum(), states.getNum(), states.getNum(), states.getNum(),
            phi22.getStorage().getDDRM()
    );

    return phi22.transpose().times(phi12);
  }

  /**
   * Discretizes the given continuous A matrix.
   *
   * @param <S>       Num representing the number of states.
   * @param contA     Continuous system matrix.
   * @param dtSeconds Discretization timestep.
   * @return the discrete matrix system.
   */
  public static <S extends Num> Matrix<S, S> discretizeA(Matrix<S, S> contA, double dtSeconds) {
    return exp(contA.times(dtSeconds));
  }

  /**
   * Discretizes the given continuous A and Q matrices.
   *
   * <p>Rather than solving a 2N x 2N matrix exponential like in DiscretizeQ() (which
   * is expensive), we take advantage of the structure of the block matrix of A
   * and Q.
   *
   * <p>The exponential of A*t, which is only N x N, is relatively cheap.
   * 2) The upper-right quarter of the 2N x 2N matrix, which we can approximate
   * using a taylor series to several terms and still be substantially cheaper
   * than taking the big exponential.
   *
   * @param <S>       Nat representing the number of states.
   * @param contA     Continuous system matrix.
   * @param contQ     Continuous process noise covariance matrix.
   * @param dtSeconds Discretization timestep.
   * @return a pair representing the discrete system matrix and process noise covariance matrix.
   */
  @SuppressWarnings("LocalVariableName")
  public static <S extends Num> SimpleMatrixUtils.Pair<Matrix<S, S>,
          Matrix<S, S>> discretizeAQTaylor(Matrix<S, S> contA, Matrix<S, S> contQ,
                                           double dtSeconds) {
    Matrix<S, S> Q = (contQ.plus(contQ.transpose())).div(2.0);


    Matrix<S, S> lastTerm = Q.copy();
    double lastCoeff = dtSeconds;

    // A^T^n
    Matrix<S, S> Atn = contA.transpose();
    Matrix<S, S> phi12 = lastTerm.times(lastCoeff);

    // i = 6 i.e. 6th order should be enough precision
    for (int i = 2; i < 6; ++i) {
      lastTerm = contA.times(-1).times(lastTerm).plus(Q.times(Atn));
      lastCoeff *= dtSeconds / ((double) i);

      phi12 = phi12.plus(lastTerm.times(lastCoeff));

      Atn = Atn.times(contA.transpose());
    }

    var discA = discretizeA(contA, dtSeconds);
    Q = discA.times(phi12);

    // Make Q symmetric if it isn't already
    var discQ = Q.plus(Q.transpose()).div(2.0);

    return new SimpleMatrixUtils.Pair<>(discA, discQ);
  }

  /**
   * Returns a discretized version of the provided continuous measurement noise
   * covariance matrix. Note that dt=0.0 divides R by zero.
   *
   * @param <O>       Nat representing the number of outputs.
   * @param R         Continuous measurement noise covariance matrix.
   * @param dtSeconds Discretization timestep.
   * @return Discretized version of the provided continuous measurement noise covariance matrix.
   */
  public static <O extends Num> Matrix<O, O> discretizeR(Matrix<O, O> R, double dtSeconds) {
    return R.div(dtSeconds);
  }

  /**
   * Returns a discretized version of the provided continuous measurement noise
   * covariance matrix.
   *
   * @param <O>       Num representing the size of R.
   * @param R         Continuous measurement noise covariance matrix.
   * @param dtSeconds Discretization timestep.
   * @return A discretized version of the provided continuous measurement noise covariance matrix.
   */
  public static <O extends Num> Matrix<O, O> discretizeMeasurementNoiseCov(
          Matrix<O, O> R, double dtSeconds) {
    return R.div(dtSeconds);
  }

  /**
   * Creates a cost matrix from the given vector for use with LQR.
   *
   * <p>The cost matrix is constructed using Bryson's rule. The inverse square of
   * each element in the input is taken and placed on the cost matrix diagonal.
   *
   * @param <S>   Nat representing the states of the system.
   * @param costs An array. For a Q matrix, its elements are the maximum allowed
   *              excursions of the states from the reference. For an R matrix,
   *              its elements are the maximum allowed excursions of the control
   *              inputs from no actuation.
   * @return State excursion or control effort cost matrix.
   */
  public static <S extends Num> Matrix<S, S> makeCostMatrix(Matrix<S, N1> costs) {
    var result = new SimpleMatrix(costs.getNumRows(), costs.getNumRows());
    result.fill(0.0);

    for (int i = 0; i < costs.getNumRows(); i++) {
      result.set(i, i, 1.0 / (Math.pow(costs.get(i, 0), 2)));
    }

    return new Matrix<>(result);
  }

  /**
   * Discretizes the given continuous A and B matrices.
   *
   * @param <S>       Nat representing the states of the system.
   * @param <I>       Nat representing the inputs to the system.
   * @param contA     Continuous system matrix.
   * @param contB     Continuous input matrix.
   * @param dtSeconds Discretization timestep.
   * @return a Pair representing discA and diskB.
   */
  @SuppressWarnings("LocalVariableName")
  public static <S extends Num, I extends Num> SimpleMatrixUtils.Pair<Matrix<S, S>,
          Matrix<S, I>> discretizeAB(Matrix<S, S> contA,
                                     Matrix<S, I> contB,
                                     double dtSeconds) {

    SimpleMatrix Mcont = new SimpleMatrix(0, 0);
    var scaledA = contA.times(dtSeconds);
    var scaledB = contB.times(dtSeconds);
    Mcont = Mcont.concatColumns(scaledA.getStorage());
    Mcont = Mcont.concatColumns(scaledB.getStorage());
    // so our Mcont is now states x (states + inputs)
    // and we want (states + inputs) x (states + inputs)
    // so we want to add (inputs) many rows onto the bottom
    Mcont = Mcont.concatRows(new SimpleMatrix(contB.getNumCols(),
            contB.getNumRows() + contB.getNumCols()));
    var Mdisc = exp(Mcont);

    var discA = new Matrix<S, S>(new SimpleMatrix(contB.getNumRows(), contB.getNumRows()));
    var discB = new Matrix<S, I>(new SimpleMatrix(contB.getNumRows(), contB.getNumCols()));
    CommonOps_DDRM.extract(Mdisc.getDDRM(), 0, 0, discA.getStorage().getDDRM());
    CommonOps_DDRM.extract(Mdisc.getDDRM(), 0, contB.getNumRows(), discB.getStorage().getDDRM());

    return new SimpleMatrixUtils.Pair<>(discA, discB);
  }

  /**
   * Returns true if (A, B) is a stabilizable pair.
   *
   * <p>(A,B) is stabilizable if and only if the uncontrollable eigenvalues of A, if
   * any, have absolute values less than one, where an eigenvalue is
   * uncontrollable if rank(lambda * I - A, B) %3C n where n is number of states.
   *
   * @param <S> Num representing the size of A.
   * @param <I> Num representing the columns of B.
   * @param A   System matrix.
   * @param B   Input matrix.
   * @return If the system is stabilizable.
   */
  public static <S extends Num, I extends Num> boolean isStabilizable(
          Matrix<S, S> A, Matrix<S, I> B
  ) {
    return WPIUtilJNI.isStabilizable(A.getNumRows(), B.getNumCols(),
            A.getStorage().getDDRM().getData(), B.getStorage().getDDRM().getData());
  }

  /**
   * Computes the matrix exponential using Eigen's solver.
   *
   * @param A the matrix to exponentiate.
   * @return the exponential of A.
   */
  public static SimpleMatrix exp(
          SimpleMatrix A
  ) {
    SimpleMatrix toReturn = new SimpleMatrix(A.numRows(), A.numRows());
    WPIUtilJNI.exp(A.getDDRM().getData(), A.numRows(), toReturn.getDDRM().getData());
    return toReturn;
  }

  /**
   * Computes the matrix exponential using Eigen's solver.
   *
   * @param A   the matrix to exponentiate.
   * @param <N> the size of the matrix A.
   * @return the exponential of A.
   */
  public static <N extends Num> Matrix<N, N> exp(Matrix<N, N> A) {
    Matrix<N, N> toReturn = new Matrix<>(new SimpleMatrix(A.getNumRows(), A.getNumCols()));
    WPIUtilJNI.exp(A.getStorage().getDDRM().getData(), A.getNumRows(),
            toReturn.getStorage().getDDRM().getData());
    return toReturn;
  }

  /**
   * Convert a {@link Pose2d} to a vector of [x, y, theta], where theta is in radians.
   *
   * @param pose A pose to convert to a vector.
   * @return The given pose in vector form, with the third element, theta, in radians.
   */
  public static Matrix<N3, N1> poseToVector(Pose2d pose) {
    return VecBuilder.fill(
            pose.getTranslation().getX(),
            pose.getTranslation().getY(),
            pose.getRotation().getRadians()
    );
  }

  /**
   * Decompose a given matrix by QR decomposition with the Householder QR decomposition algorithm.
   * This will throw a RuntimeException if src is not full rank.
   *
   * @param src The matrix to decompose.
   * @return the decomposed matrix.
   */
  public static SimpleMatrix householderQrDecompose(SimpleMatrix src) {
    var temp = src.copy();

    var decomposer = new QRDecompositionHouseholder_DDRM();
    if (!(decomposer.decompose(temp.getDDRM()))) {
        throw new RuntimeException("Householder decomposition failed!");
    }

    return temp;
  }
}
