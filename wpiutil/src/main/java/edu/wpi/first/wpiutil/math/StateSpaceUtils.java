package edu.wpi.first.wpilibj.math;

import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.SimpleMatrixUtils;
import edu.wpi.first.wpiutil.math.numbers.N1;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.decomposition.qr.QRDecompositionHouseholder_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.simple.SimpleMatrix;

import java.util.Random;

public class StateSpaceUtils {

    /**
     * Creates a covariance matrix from the given vector for use with Kalman
     * filters.
     * <p>
     * Each element is squared and placed on the covariance matrix diagonal.
     *
     * @param <States> Num representing the states of the system.
     * @param states   A Nat representing the states of the system.
     * @param stdDevs  For a Q matrix, its elements are the standard deviations of
     *                 each state from how the model behaves. For an R matrix, its
     *                 elements are the standard deviations for each output
     *                 measurement.
     * @return Process noise or measurement noise covariance matrix.
     */
    public static <States extends Num> Matrix<States, States> makeCovMatrix(
            Nat<States> states, Matrix<States, N1> stdDevs
    ) {
        var result = new Matrix<States, States>(new SimpleMatrix(states.getNum(), states.getNum()));
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
     * @param n       A Nat representing the dimensionality of  the noise vector to create.
     * @param stdDevs A matrix whose elements are the standard deviations of each
     *                element of the noise vector.
     * @return White noise vector.
     */
    public static <N extends Num> Matrix<N, N1> makeWhiteNoiseVector(
            Nat<N> n, Matrix<N, N1> stdDevs
    ) {
        var rand = new Random();

        Matrix<N, N1> result = new Matrix<>(new SimpleMatrix(n.getNum(), 1));
        for (int i = 0; i < n.getNum(); i++) {
            result.set(i, 0, rand.nextGaussian() * stdDevs.get(i, 0));
        }
        return result;
    }

    /**
     * Returns a discretized version of the provided continuous process noise
     * covariance matrix.
     *
     * @param <States>  A Num representing the number of states.
     * @param states    A Nat representing the number of states.
     * @param A         The system matrix A.
     * @param Q         Continuous process noise covariance matrix.
     * @param dtSeconds Discretization timestep.
     * @return The discretized process noise covariance matrix.
     */
    public static <States extends Num> Matrix<States, States> discretizeProcessNoiseCov(
            Nat<States> states, Matrix<States, States> A, Matrix<States, States> Q, double dtSeconds) {

        var Mgain = new SimpleMatrix(0, 0);

        // Set up the matrix M = [[-A, Q], [0, A.T]]
        Mgain = Mgain.concatColumns(
                (A.times(-1)).getStorage().concatRows(new SimpleMatrix(states.getNum(), states.getNum())),
                Q.getStorage().concatRows(A.transpose().getStorage())
        );

        var phi = SimpleMatrixUtils.expm(Mgain.scale(dtSeconds));

        // Phi12 = phi[0:States,        States:2*States]
        // Phi22 = phi[States:2*States, States:2*States]
        Matrix<States, States> phi12 = new Matrix<>(new SimpleMatrix(states.getNum(), states.getNum()));
        Matrix<States, States> phi22 = new Matrix<>(new SimpleMatrix(states.getNum(), states.getNum()));
        CommonOps_DDRM.extract(
                phi.getDDRM(), 0, states.getNum(), states.getNum(), states.getNum(), phi12.getStorage().getDDRM()
        );
        CommonOps_DDRM.extract(
                phi.getDDRM(), states.getNum(), states.getNum(), states.getNum(), states.getNum(), phi22.getStorage().getDDRM()
        );

        return phi22.transpose().times(phi12);
    }

    /**
     * Discretizes the given continuous A matrix.
     *
     * @param <States>  Num representing the number of states.
     * @param contA     Continuous system matrix.
     * @param dtSeconds Discretization timestep.
     * @return the discrete matrix system.
     */
    public static <States extends Num> Matrix<States, States> discretizeA(Matrix<States, States> contA, double dtSeconds) {
        return exp((contA.times(dtSeconds)));
    }

    /**
     * Discretizes the given continuous A and Q matrices.
     * <p>
     * Rather than solving a 2N x 2N matrix exponential like in DiscretizeQ() (which
     * is expensive), we take advantage of the structure of the block matrix of A
     * and Q.
     * <p>
     * The exponential of A*t, which is only N x N, is relatively cheap.
     * 2) The upper-right quarter of the 2N x 2N matrix, which we can approximate
     * using a taylor series to several terms and still be substantially cheaper
     * than taking the big exponential.
     *
     * @param <States>  Nat representing the number of states.
     * @param contA     Continuous system matrix.
     * @param contQ     Continuous process noise covariance matrix.
     * @param dtSeconds Discretization timestep.
     * @return a pair representing the discrete system matrix and process noise covariance matrix.
     */
    public static <States extends Num> Pair<Matrix<States, States>, Matrix<States, States>> discretizeAQTaylor(
            Matrix<States, States> contA, Matrix<States, States> contQ, double dtSeconds
    ) {

        Matrix<States, States> Q = (contQ.plus(contQ.transpose())).div(2.0);

        Matrix<States, States> lastTerm = Q.copy();
        double lastCoeff = dtSeconds;

        // A^T^n
        Matrix<States, States> Atn = contA.transpose();
        Matrix<States, States> phi12 = lastTerm.times(lastCoeff);

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
        var discQ = (Q.plus(Q.transpose()).div(2.0));

        return new Pair<>(discA, discQ);
    }

    /**
     * Returns a discretized version of the provided continuous measurement noise
     * covariance matrix.
     *
     * @param <Outputs> Nat representing the number of outputs.
     * @param R         Continuous measurement noise covariance matrix.
     * @param dtSeconds Discretization timestep.
     * @return Discretized version of the provided continuous measurement noise covariance matrix.
     */
    public static <Outputs extends Num> Matrix<Outputs, Outputs> discretizeR(Matrix<Outputs, Outputs> R, double dtSeconds) {
        return R.div(dtSeconds);
    }

    /**
     * Returns a discretized version of the provided continuous measurement noise
     * covariance matrix.
     *
     * @param <Outputs> Num representing the size of R.
     * @param R         Continuous measurement noise covariance matrix.
     * @param dtSeconds Discretization timestep.
     * @return A discretized version of the provided continuous measurement noise covariance matrix.
     */
    public static <Outputs extends Num> Matrix<Outputs, Outputs> discretizeMeasurementNoiseCov(
            Matrix<Outputs, Outputs> R, double dtSeconds) {
        return R.div(dtSeconds);
    }

    /**
     * Creates a cost matrix from the given vector for use with LQR.
     * <p>
     * The cost matrix is constructed using Bryson's rule. The inverse square of
     * each element in the input is taken and placed on the cost matrix diagonal.
     *
     * @param <States> Nat representing the states of the system.
     * @param states   a Nat representing the number of States in the system.
     * @param costs    An array. For a Q matrix, its elements are the maximum allowed
     *                 excursions of the states from the reference. For an R matrix,
     *                 its elements are the maximum allowed excursions of the control
     *                 inputs from no actuation.
     * @return State excursion or control effort cost matrix.
     */
    public static <States extends Num> Matrix<States, States> makeCostMatrix(Nat<States> states, Matrix<States, N1> costs) {
        var result = new SimpleMatrix(states.getNum(), states.getNum());
        result.fill(0.0);

        for (int i = 0; i < states.getNum(); i++) {
            result.set(i, i, 1.0 / (Math.pow(costs.get(i, 0), 2)));
        }

        return new Matrix<>(result);
    }

    /**
     * Discretizes the given continuous A and B matrices.
     *
     * @param <States>  Nat representing the states of the system.
     * @param <Inputs>  Nat representing the inputs to the system.
     * @param states    Num representing the states of the system.
     * @param inputs    Num representing the inputs to the system.
     * @param contA     Continuous system matrix.
     * @param contB     Continuous input matrix.
     * @param dtSeconds Discretization timestep.
     * @param discA     Storage for discrete system matrix.
     * @param discB     Storage for discrete input matrix.
     * @return a Pair representing discA and diskB.
     */
    public static <States extends Num, Inputs extends Num> Pair<Matrix<States, States>, Matrix<States, Inputs>>
    discretizeAB(Nat<States> states, Nat<Inputs> inputs,
                 Matrix<States, States> contA,
                 Matrix<States, Inputs> contB,
                 double dtSeconds,
                 Matrix<States, States> discA,
                 Matrix<States, Inputs> discB) {

        SimpleMatrix Mcont = new SimpleMatrix(0, 0);
        var scaledA = contA.times(dtSeconds);
        var scaledB = contB.times(dtSeconds);
        Mcont = Mcont.concatColumns(scaledA.getStorage());
        Mcont = Mcont.concatColumns(scaledB.getStorage());
        // so our Mcont is now states x (states + inputs)
        // and we want (states + inputs) x (states + inputs)
        // so we want to add (inputs) many rows onto the bottom
        Mcont = Mcont.concatRows(new SimpleMatrix(inputs.getNum(), states.getNum() + inputs.getNum()));

//        System.out.println(Mcont);

        // Discretize A and B with the given timestep
        var Mdisc = StateSpaceUtils.scipyExpm(Mcont); // TODO should we use the Eigen exp algorithm?

//        System.out.printf("Mdisc: \n%s", Mdisc);

//        discA.getStorage().set(Mdisc.extractMatrix(0, 0, states.getNum(), states.getNum()));
//        discB.getStorage().set(Mdisc.extractMatrix(0, states.getNum(), states.getNum(), inputs.getNum()));
        CommonOps_DDRM.extract(Mdisc.getDDRM(), 0, 0, discA.getStorage().getDDRM());
        CommonOps_DDRM.extract(Mdisc.getDDRM(), 0, states.getNum(), discB.getStorage().getDDRM());

        return new Pair<>(discA, discB);
    }

    /**
     * The identy of a square matrix
     *
     * @param rows the number of rows (and columns)
     * @return the identiy matrix, rows x rows.
     */
    private static SimpleMatrix eye(int rows) {
        return SimpleMatrix.identity(rows);
    }

    public static SimpleMatrix lltDecompose(SimpleMatrix src) {
        SimpleMatrix temp = src.copy();
        CholeskyDecomposition_F64<DMatrixRMaj> chol =
                DecompositionFactory_DDRM.chol(temp.numRows(), true);
        if (!chol.decompose(temp.getMatrix()))
            throw new RuntimeException("Cholesky failed!");

        return SimpleMatrix.wrap(chol.getT(null));
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
            throw new RuntimeException("householder decomposition failed!");
        }

        return temp;
    }

    /**
     * Calculate matrix exponential of a square matrix.
     * <p>
     * Implementation from jblas https://github.com/jblas-project/jblas
     * <p>
     * A scaled Pade approximation algorithm is used.
     * The algorithm has been directly translated from Golub and Van Loan "Matrix Computations",
     * algorithm 11.3.1. Special Horner techniques from 11.2 are also used to minimize the number
     * of matrix multiplications.
     *
     * @param A a square matrix.
     * @return the matrix exponential of A.
     */
    public static SimpleMatrix scipyExpm(SimpleMatrix A) {

        /*
         * Copyright (c) 2009, Mikio L. Braun
         * All rights reserved.
         *
         * Redistribution and use in source and binary forms, with or without
         * modification, are permitted provided that the following conditions are
         * met:
         *
         *     * Redistributions of source code must retain the above copyright
         *       notice, this list of conditions and the following disclaimer.
         *
         *     * Redistributions in binary form must reproduce the above
         *       copyright notice, this list of conditions and the following
         *       disclaimer in the documentation and/or other materials provided
         *       with the distribution.
         *
         *     * Neither the name of the Technische Universitat Berlin nor the
         *       names of its contributors may be used to endorse or promote
         *       products derived from this software without specific prior
         *       written permission.
         *
         * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
         * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
         * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
         * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
         * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
         * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
         * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
         * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
         * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
         * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
         * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
         */

        if (A.numRows() != A.numCols()) {
            throw new RuntimeException("A must be square to preform expm!");
        }

        // constants for pade approximation
        final double c0 = 1.0;
        final double c1 = 0.5;
        final double c2 = 0.12;
        final double c3 = 0.01833333333333333;
        final double c4 = 0.0019927536231884053;
        final double c5 = 1.630434782608695E-4;
        final double c6 = 1.0351966873706E-5;
        final double c7 = 5.175983436853E-7;
        final double c8 = 2.0431513566525E-8;
        final double c9 = 6.306022705717593E-10;
        final double c10 = 1.4837700484041396E-11;
        final double c11 = 2.5291534915979653E-13;
        final double c12 = 2.8101705462199615E-15;
        final double c13 = 1.5440497506703084E-17;

        int j = Math.max(0, 1 + (int) Math.floor(Math.log(A.elementMaxAbs()) / Math.log(2)));
        SimpleMatrix As = A.divide(Math.pow(2, j)); // scaled version of A
        int n = A.numRows();

        // calculate D and N using special Horner techniques
        SimpleMatrix As_2 = As.mult(As);
        SimpleMatrix As_4 = As_2.mult(As_2);
        SimpleMatrix As_6 = As_4.mult(As_2);
        // U = c0*I + c2*A^2 + c4*A^4 + (c6*I + c8*A^2 + c10*A^4 + c12*A^6)*A^6
        SimpleMatrix U = eye(n).scale(c0).plus(As_2.scale(c2)).plus(As_4.scale(c4)).plus(
                eye(n).scale(c6).plus(As_2.scale(c8)).plus(As_4.scale(c10)).plus(As_6.scale(c12)).mult(As_6));
        // V = c1*I + c3*A^2 + c5*A^4 + (c7*I + c9*A^2 + c11*A^4 + c13*A^6)*A^6
        SimpleMatrix V = eye(n).scale(c1).plus(As_2.scale(c3)).plus(As_4.scale(c5)).plus(
                eye(n).scale(c7).plus(As_2.scale(c9)).plus(As_4.scale(c11)).plus(As_6.scale(c13)).mult(As_6));

        SimpleMatrix AV = As.mult(V);
        SimpleMatrix N = U.plus(AV);
        SimpleMatrix D = U.minus(AV);

        // solve DF = N for F
        SimpleMatrix F = D.solve(N);

        // now square j times
        for (int k = 0; k < j; k++) {
            F.mult(F);
        }

        return F;

    }

    /**
     * Computes the matrix exponential using Eigen's solver.
     *
     * @param A   the matrix to exponentiate.
     * @param <N> the size of the matrix A.
     * @return the exponential of A.
     */
    public static <N extends Num> Matrix<N, N> exp(
            Matrix<N, N> A
    ) {
        Matrix<N, N> toReturn = new Matrix<>(new SimpleMatrix(A.getNumRows(), A.getNumCols()));
        StateSpaceUtilsJNI.exp(A.getStorage().getDDRM().getData(), A.getNumRows(), toReturn.getStorage().getDDRM().getData());
        return toReturn;
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
        StateSpaceUtilsJNI.exp(A.getDDRM().getData(), A.numRows(), toReturn.getDDRM().getData());
        return toReturn;
    }

    /**
     * Returns true if (A, B) is a stabilizable pair.
     * <p>
     * (A,B) is stabilizable if and only if the uncontrollable eigenvalues of A, if
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
        return StateSpaceUtilsJNI.isStabilizable(A.getNumRows(), B.getNumCols(),
                A.getStorage().getDDRM().getData(), B.getStorage().getDDRM().getData());
    }

    /**
     * Returns true if (A, B) is a stabilizable pair.
     * <p>
     * (A,B) is stabilizable if and only if the uncontrollable eigenvalues of A, if
     * any, have absolute values less than one, where an eigenvalue is
     * uncontrollable if rank(lambda * I - A, B) %3C n where n is number of states.
     *
     * @param A System matrix.
     * @return If the system is stabilizable or not.
     */
    public static SimpleMatrix isStabilizable(
            SimpleMatrix A
    ) {
        SimpleMatrix toReturn = new SimpleMatrix(A.numRows(), A.numRows());
        StateSpaceUtilsJNI.exp(A.getDDRM().getData(), A.numRows(), toReturn.getDDRM().getData());
        return toReturn;
    }

    public static class Pair<A, B> {
        private final A m_first;
        private final B m_second;

        Pair(A first, B second) {
            m_first = first;
            m_second = second;
        }

        public A getFirst() {
            return m_first;
        }

        public B getSecond() {
            return m_second;
        }
    }

}
