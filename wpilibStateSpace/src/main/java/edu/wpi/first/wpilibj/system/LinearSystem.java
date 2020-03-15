package edu.wpi.first.wpilibj.system;

import edu.wpi.first.wpilibj.math.StateSpaceUtils;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpiutil.math.*;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import org.ejml.simple.SimpleMatrix;

public class LinearSystem<States extends Num, Inputs extends Num,
        Outputs extends Num> {

    /**
     * Continuous system matrix.
     */
    private final Matrix<States, States> m_A;

    /**
     * Continuous input matrix.
     */
    private final Matrix<States, Inputs> m_B;

    /**
     * Output matrix.
     */
    private final Matrix<Outputs, States> m_C;

    /**
     * Feedthrough matrix.
     */
    private final Matrix<Outputs, Inputs> m_D;

    /**
     * Minimum allowable input vector.
     */
    private final Matrix<Inputs, N1> m_uMin;

    /**
     * Maximum allowable input vector.
     */
    private final Matrix<Inputs, N1> m_uMax;
    /**
     * The states of the system represented as a Nat.
     */
    private final Nat<States> states;
    /**
     * The inputs of the system represented as a Nat.
     */
    private final Nat<Inputs> inputs;
    /**
     * The outputs of the system represented as a Nat.
     */
    private final Nat<Outputs> outputs;
    /**
     * State vector.
     */
    private Matrix<States, N1> m_x;
    /**
     * Output vector.
     */
    private Matrix<Outputs, N1> m_y;
    /**
     * Delayed u since predict and correct steps are run in reverse.
     */
    private Matrix<Inputs, N1> m_delayedU;

    /**
     * @param states  A Nat representing the states of the system.
     * @param inputs  A Nat representing the inputs to the system.
     * @param outputs A Nat representing the outputs of the system.
     * @param A       The system matrix A.
     * @param B       The input matrix B.
     * @param C       The output matrix C.
     * @param D       The feedthrough matrix D.
     * @param uMin    The minimum control input. Inputs with elements smaller than this will be clamped to the minimum input.
     * @param uMax    The maximum control input. Inputs with elements smaller than this will be clamped to the maximum input.
     */
    public LinearSystem(Nat<States> states, Nat<Inputs> inputs, Nat<Outputs> outputs,
                        Matrix<States, States> A, Matrix<States, Inputs> B,
                        Matrix<Outputs, States> C, Matrix<Outputs, Inputs> D,
                        Matrix<Inputs, N1> uMin, Matrix<Inputs, N1> uMax) {

        this.states = states;
        this.inputs = inputs;
        this.outputs = outputs;

        this.m_A = A;
        this.m_B = B;
        this.m_C = C;
        this.m_D = D;
        this.m_uMin = uMin;
        this.m_uMax = uMax;

        this.m_x = MatrixUtils.zeros(states);
        this.m_y = MatrixUtils.zeros(outputs);
        this.m_delayedU = MatrixUtils.zeros(inputs);

        reset();
    }

    /**
     * @param motor        The motor (or gearbox) attached to the arm.
     * @param massKg       The mass of the elevator carriage, in kilograms.
     * @param radiusMeters The radius of thd driving drum of the elevator, in meters.
     * @param G            The reduction between motor and drum, as a ratio of output to input.
     * @param maxVoltage   The max voltage that can be applied. Inputs greater than this will
     *                     be clamped to it.
     * @return A LinearSystem representing the given characterized constants.
     */
    public static LinearSystem<N2, N1, N1> createElevatorSystem(DCMotor motor, double massKg, double radiusMeters, double G, double maxVoltage) {
        return new LinearSystem<>(Nat.N2(), Nat.N1(), Nat.N1(),
                new MatBuilder<>(Nat.N2(), Nat.N2()).fill(0, 1,
                        0, -Math.pow(G, 2) * motor.KtNMPerAmp /
                                (motor.Rohms * radiusMeters * radiusMeters * massKg * motor.KvRadPerSecPerVolt)),
                new MatBuilder<>(Nat.N2(), Nat.N1()).fill(
                        0, (G * motor.KtNMPerAmp / (motor.Rohms * radiusMeters * massKg))),
                new MatBuilder<>(Nat.N1(), Nat.N2()).fill(1, 0),
                MatrixUtils.zeros(Nat.N1()),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(-maxVoltage),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(maxVoltage));
    }

    /**
     * @param motor            The motor (or gearbox) attached to the arm.
     * @param jKgSquaredMeters The momoent of inertia J of the flywheel.
     * @param G                The reduction between motor and drum, as a ratio of output to input.
     * @param maxVoltage       The max voltage that can be applied. Inputs greater than this will
     *                         be clamped to it.
     * @return A LinearSystem representing the given characterized constants.
     */
    public static LinearSystem<N1, N1, N1> createFlywheelSystem(DCMotor motor, double jKgSquaredMeters, double G, double maxVoltage) {
        return new LinearSystem<>(Nat.N1(), Nat.N1(), Nat.N1(),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                        -G * G * motor.KtNMPerAmp /
                                (motor.KvRadPerSecPerVolt * motor.Rohms * jKgSquaredMeters)),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(G * motor.KtNMPerAmp / (motor.Rohms * jKgSquaredMeters)),
                MatrixUtils.eye(Nat.N1()),
                MatrixUtils.zeros(Nat.N1()),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(-maxVoltage),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(maxVoltage));
    }

    /**
     * @param motor            The motor (or gearbox) attached to the arm.
     * @param jKgSquaredMeters The momoent of inertia J of the arm.
     * @param G                the gearing between the motor and arm, in output over input. Most of the time this
     *                         will be greater than 1.
     * @param maxVoltage       The max voltage that can be applied. Inputs greater than this will
     *                         be clamped to it.
     * @return A LinearSystem representing the given characterized constants.
     */
    public static LinearSystem<N2, N1, N1> createSingleJointedArmSystem(DCMotor motor, double jKgSquaredMeters, double G, double maxVoltage) {
        return new LinearSystem<>(Nat.N2(), Nat.N1(), Nat.N1(),
                new MatBuilder<>(Nat.N2(), Nat.N2()).fill(0, 1,
                        0, -Math.pow(G, 2) * motor.KtNMPerAmp /
                                (motor.KvRadPerSecPerVolt * motor.Rohms * jKgSquaredMeters)),
                new MatBuilder<>(Nat.N2(), Nat.N1()).fill(0, (G * motor.KtNMPerAmp / (motor.Rohms * jKgSquaredMeters))),
                new MatBuilder<>(Nat.N1(), Nat.N2()).fill(1, 0),
                MatrixUtils.zeros(Nat.N1()),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(-maxVoltage),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(maxVoltage));
    }

    /**
     * Identify a velocity system from it's kV (volts/(unit/sec)) and kA (volts/(unit/sec^2).
     * These constants cam be found using frc-characterization.
     *
     * @param kV         The velocity gain, in volts per (units per second)
     * @param kA         The acceleration gain, in volts per (units per second squared)
     * @param maxVoltage The max voltage that can be applied. Inputs greater than this will
     *                   be clamped to it.
     * @return A LinearSystem representing the given characterized constants.
     * @see <a href="https://github.com/wpilibsuite/frc-characterization"> https://github.com/wpilibsuite/frc-characterization</a>
     */
    public static LinearSystem<N1, N1, N1> identifyVelocitySystem(double kV, double kA, double maxVoltage) {
        return new LinearSystem<>(Nat.N1(), Nat.N1(), Nat.N1(),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(-kV / kA),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(1.0 / kA),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(1.0),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.0),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(-maxVoltage),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(maxVoltage));
    }

    /**
     * Identify a position system from it's kV (volts/(unit/sec)) and kA (volts/(unit/sec^2).
     * These constants cam be found using frc-characterization.
     *
     * @param kV         The velocity gain, in volts per (units per second)
     * @param kA         The acceleration gain, in volts per (units per second squared)
     * @param maxVoltage The max voltage that can be applied. Control inputs above this will be clamped to it.
     * @return A LinearSystem representing the given characterized constants.
     * @see <a href="https://github.com/wpilibsuite/frc-characterization"> https://github.com/wpilibsuite/frc-characterization</a>
     */
    public static LinearSystem<N2, N1, N1> identifyPositionSystem(double kV, double kA, double maxVoltage) {
        return new LinearSystem<>(Nat.N2(), Nat.N1(), Nat.N1(),
                new MatBuilder<>(Nat.N2(), Nat.N2()).fill(0.0, 1.0, 0.0, -kV / kA),
                new MatBuilder<>(Nat.N2(), Nat.N1()).fill(0.0, 1.0 / kA),
                new MatBuilder<>(Nat.N1(), Nat.N2()).fill(1.0, 0.0),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.0),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(-maxVoltage),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(maxVoltage));
    }

    /**
     * Identify a standard differential drive drivetrain, given the drivetrain's
     * kV and kA in both linear (volts/(meter/sec) and volts/(meter/sec^2)) and
     * angular (volts/(radian/sec) and volts/(radian/sec^2)) cases. This can be
     * found using frc-characterization.
     *
     * @param kVLinear   The linear velocity gain, volts per (meter per second).
     * @param kALinear   The linear acceleration gain, volts per (meter per second squared).
     * @param kVAngular  The angular velocity gain, volts per (radians per second).
     * @param kAAngular  The angular acceleration gain, volts per (radians per second squared).
     * @param maxVoltage The max voltage that can be applied. Control inputs above this will be clamped to it.
     * @return A LinearSystem representing the given characterized constants.
     * @see <a href="https://github.com/wpilibsuite/frc-characterization"> https://github.com/wpilibsuite/frc-characterization</a>
     */
    public static LinearSystem<N2, N2, N2> identifyDrivetrainSystem(
            double kVLinear, double kALinear, double kVAngular, double kAAngular, double maxVoltage) {

        final double c = 0.5 / (kALinear * kAAngular);
        final double A1 = c * (-kALinear * kVAngular - kVLinear * kAAngular);
        final double A2 = c * (kALinear * kVAngular - kVLinear * kAAngular);
        final double B1 = c * (kALinear + kAAngular);
        final double B2 = c * (kAAngular - kALinear);

        return new LinearSystem<>(Nat.N2(), Nat.N2(), Nat.N2(),
                new MatBuilder<>(Nat.N2(), Nat.N2()).fill(A1, A2, A2, A1),
                new MatBuilder<>(Nat.N2(), Nat.N2()).fill(B1, B2, B2, B1),
                new MatBuilder<>(Nat.N2(), Nat.N2()).fill(1, 0, 0, 1),
                new MatBuilder<>(Nat.N2(), Nat.N2()).fill(0, 0, 0, 0),
                new MatBuilder<>(Nat.N2(), Nat.N1()).fill(-maxVoltage, -maxVoltage),
                new MatBuilder<>(Nat.N2(), Nat.N1()).fill(maxVoltage, maxVoltage));
    }

    /**
     * Resets the plant.
     */
    public void reset() {
        m_x = MatrixUtils.zeros(states);
        m_y = MatrixUtils.zeros(outputs);
        m_delayedU = MatrixUtils.zeros(inputs);
    }

    /**
     * Returns the system matrix A.
     *
     * @return the system matrix A.
     */
    public Matrix<States, States> getA() {
        return m_A;
    }

    /**
     * Returns an element of the system matrix A.
     *
     * @param i Row of A.
     * @param j Column of A.
     * @return the system matrix A at (i, j).
     */
    public double getA(int i, int j) {
        return m_A.get(i, j);
    }

    /**
     * Returns the input matrix B.
     *
     * @return the input matrix B.
     */
    public Matrix<States, Inputs> getB() {
        return m_B;
    }

    /**
     * Returns an element of the input matrix B.
     *
     * @param i Row of B.
     * @param j Column of B.
     * @return The value of the input matrix B at (i, j).
     */
    public double getB(int i, int j) {
        return m_B.get(i, j);
    }

    /**
     * Returns the output matrix C.
     *
     * @return Output matrix C.
     */
    public Matrix<Outputs, States> getC() {
        return m_C;
    }

    /**
     * Returns an element of the output matrix C.
     *
     * @param i Row of C.
     * @param j Column of C.
     * @return the double value of C at the given position.
     */
    public double getC(int i, int j) {
        return m_C.get(i, j);
    }

    /**
     * Returns the feedthrough matrix D.
     *
     * @return the feedthrough matrix D.
     */
    public Matrix<Outputs, Inputs> getD() {
        return m_D;
    }

    /**
     * Returns an element of the feedthrough matrix D.
     *
     * @param i Row of D.
     * @param j Column of D.
     * @return The feedthrough matrix D at (i, j).
     */
    public double getD(int i, int j) {
        return m_D.get(i, j);
    }

    /**
     * Returns the minimum control input vector u.
     *
     * @return The minimum control input vector u.
     */
    public Matrix<Inputs, N1> getUMin() {
        return m_uMin;
    }

    /**
     * Returns an element of the minimum control input vector u.
     *
     * @param i Row of u.
     * @return The i-th element of the minimum control input vector u.
     */
    public double getUMin(int i) {
        return m_uMin.get(i, 1);
    }

    /**
     * Returns the maximum control input vector u.
     *
     * @return The maximum control input vector u.
     */
    public Matrix<Inputs, N1> getUMax() {
        return m_uMax;
    }

    /**
     * Returns an element of the maximum control input vector u.
     *
     * @param i Row of u.
     * @return The i=th element of the maximum control input vector u.
     */
    public double getUMax(int i) {
        return m_uMax.get(i, 1);
    }

    /**
     * Returns the current state x.
     *
     * @return The current state x.
     */
    public Matrix<States, N1> getX() {
        return m_x;
    }

    /**
     * Set the initial state x.
     *
     * @param x The initial state.
     */
    public void setX(Matrix<States, N1> x) {
        m_x = x;
    }

    /**
     * Returns an element of the current state x.
     *
     * @param i Row of x.
     * @return The i-th element of the current state x.
     */
    public double getX(int i) {
        return m_x.get(i, 0);
    }

    /**
     * Returns the current measurement vector y.
     *
     * @return the current measurement vector y.
     */
    public Matrix<Outputs, N1> getY() {
        return m_y;
    }

    /**
     * Set the current measurement y.
     *
     * @param y The current measurement.
     */
    public void setY(Matrix<Outputs, N1> y) {
        m_y = y;
    }

    /**
     * Returns an element of the current measurement vector y.
     *
     * @param i Row of y.
     * @return the output matrix Y at the given row i.
     */
    public double getY(int i) {
        return m_y.get(i, 0);
    }

    /**
     * Returns the control input vector u.
     *
     * @return the control input vector u.
     */
    public Matrix<Inputs, N1> getU() {
        return m_delayedU;
    }

    /**
     * Returns an element of the control input vector u.
     *
     * @param i Row of u.
     * @return The i-th element of control input vector u.
     */
    public double getU(int i) {
        return m_delayedU.get(i, 0);
    }

    /**
     * Set an element of the initial state x.
     *
     * @param i     Row of x.
     * @param value Value of element of x.
     */
    public void setX(int i, double value) {
        m_x.set(i, 0, value);
    }

    /**
     * Set an element of the current measurement y.
     *
     * @param i     Row of y.
     * @param value Value of element of y.
     */
    public void setY(int i, double value) {
        m_y.set(i, 0, value);
    }

    /**
     * Computes the new x and y given the control input.
     *
     * @param x         The current state.
     * @param u         The control input.
     * @param dtSeconds Timestep for model update.
     */
    public void update(Matrix<States, N1> x, Matrix<Inputs, N1> u, double dtSeconds) {
        m_x = calculateX(x, m_delayedU, dtSeconds);
        m_y = calculateY(m_x, m_delayedU);
        m_delayedU = u;
    }

    /**
     * Computes the new x given the old x and the control input.
     * <p>
     * This is used by state observers directly to run updates based on state
     * estimate.
     *
     * @param x         The current state.
     * @param u         The control input.
     * @param dtSeconds Timestep for model update.
     * @return the updated x.
     */
    public Matrix<States, N1> calculateX(Matrix<States, N1> x, Matrix<Inputs, N1> u, double dtSeconds) {
        Matrix<States, States> discA = new Matrix<>(new SimpleMatrix(states.getNum(), states.getNum()));
        Matrix<States, Inputs> discB = new Matrix<>(new SimpleMatrix(states.getNum(), inputs.getNum()));

        StateSpaceUtils.discretizeAB(states, inputs, m_A, m_B, dtSeconds, discA, discB);

        return (discA.times(x)).plus(discB.times(clampInput(u)));
    }

//    private void discretizeAB(Matrix<States, States> contA,
//                              Matrix<States, Inputs> contB,
//                              double dtSeconds,
//                              Matrix<States, States> discA,
//                              Matrix<States, Inputs> discB) {
//
//        SimpleMatrix Mcont = new SimpleMatrix(0, 0);
//        var scaledA = contA.times(dtSeconds);
//        var scaledB = contB.times(dtSeconds);
//        Mcont = Mcont.concatColumns(scaledA.getStorage());
//        Mcont = Mcont.concatColumns(scaledB.getStorage());
//        // so our Mcont is now states x (states + inputs)
//        // and we want (states + inputs) x (states + inputs)
//        // so we want to add (inputs) many rows onto the bottom
//        Mcont = Mcont.concatRows(new SimpleMatrix(inputs.getNum(), states.getNum() + inputs.getNum()));
//
////        System.out.println(Mcont);
//
//        // Discretize A and B with the given timestep
//        var Mdisc = StateSpaceUtils.scipyExpm(Mcont);
//
////        System.out.printf("Mdisc: \n%s", Mdisc);
//
////        discA.getStorage().set(Mdisc.extractMatrix(0, 0, states.getNum(), states.getNum()));
////        discB.getStorage().set(Mdisc.extractMatrix(0, states.getNum(), states.getNum(), inputs.getNum()));
//        CommonOps_DDRM.extract(Mdisc.getDDRM(), 0, 0, discA.getStorage().getDDRM());
//        CommonOps_DDRM.extract(Mdisc.getDDRM(), 0, states.getNum(), discB.getStorage().getDDRM());
//    }

    /**
     * Computes the new y given the control input.
     * <p>
     * This is used by state observers directly to run updates based on state
     * estimate.
     *
     * @param x The current state.
     * @param u The control input.
     * @return the updated output matrix Y.
     */
    public Matrix<Outputs, N1> calculateY(
            Matrix<States, N1> x,
            Matrix<Inputs, N1> u) {
        return m_C.times(x).plus(m_D.times(clampInput(u)));
    }

    public Matrix<Inputs, N1> clampInput(Matrix<Inputs, N1> u) {
        var result = new Matrix<Inputs, N1>(new SimpleMatrix(inputs.getNum(), 1));
        for (int i = 0; i < inputs.getNum(); i++) {
            result.set(i, 0, MathUtil.clamp(
                    u.get(i, 0),
                    m_uMin.get(i, 0),
                    m_uMax.get(i, 0)));
        }
        return result;
    }

    @Override
    public String toString() {
        return String.format("Linear System: A\n%s\n\nB:\n%s\n\nC:\n%s\n\nD:\n%s\n", m_A.getStorage(), m_B.getStorage(), m_C.getStorage(), m_D.getStorage());
    }
}
