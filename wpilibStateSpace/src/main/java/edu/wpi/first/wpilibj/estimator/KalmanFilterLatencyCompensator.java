package edu.wpi.first.wpilibj.estimator;

import java.util.Map;
import java.util.TreeMap;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Num;
import edu.wpi.first.wpiutil.math.numbers.N1;

class KalmanFilterLatencyCompensator<States extends Num, Inputs extends Num, Outputs extends Num>
{
    private final static int k_maxPastObserverStates = 300;

    private final TreeMap<Double, ObserverState> m_pastObserverStates;

    public KalmanFilterLatencyCompensator()
    {
        m_pastObserverStates = new TreeMap<>();
    }

    public void addObserverState(KalmanTypeFilter<States, Inputs, Outputs> observer, Matrix<Inputs, N1> u)
    {
        m_pastObserverStates.put(Timer.getFPGATimestamp(), new ObserverState(observer, u));
        
        if (m_pastObserverStates.size() > k_maxPastObserverStates) {
            m_pastObserverStates.remove(m_pastObserverStates.firstKey());
        }
    }

    public void applyPastMeasurement(
        KalmanTypeFilter<States, Inputs, Outputs> observer,
        double dtSeconds,
        Matrix<Outputs, N1> y,
        double timestampSeconds
    ) {
        var low = m_pastObserverStates.floorEntry(timestampSeconds);
        var high = m_pastObserverStates.ceilingEntry(timestampSeconds);

        // Find the entry closest in time to timestampSeconds
        Map.Entry<Double, ObserverState> closestEntry = null;
        if (low != null && high != null) {
            closestEntry = Math.abs(timestampSeconds - low.getKey()) < Math.abs(timestampSeconds - high.getKey()) ? low
                    : high;
        } else {
            closestEntry = low != null ? low : high;
        }
        if (closestEntry == null) {
            // State map was empty, which means that we got a past measurement right at startup
            // The only thing we can really do is ignore the measurement
            return;
        }

        var tailMap = m_pastObserverStates.tailMap(closestEntry.getKey(), true);
        for (var st : tailMap.values()) {
            if (y != null) {
                observer.setP(st.errorCovariances);
                observer.setXhat(st.xHat);
                // Note that we correct the observer with inputs closest in time to the measurement
                // This makes the assumption that the dt is small enough that the difference between the measurement time and the time that the inputs were captured at is very small
                observer.correct(st.inputs, y);
            }
            observer.predict(st.inputs, dtSeconds);

            y = null;
        }
    }

    /**
     * This class contains all the information about our observer 
     */
    public class ObserverState {
        public final Matrix<States, N1> xHat;
        public final Matrix<States, States> errorCovariances;
        public final Matrix<Inputs, N1> inputs;

        private ObserverState(KalmanTypeFilter<States, Inputs, Outputs> observer, Matrix<Inputs, N1> u) {
            this.xHat = observer.getXhat();
            this.errorCovariances = observer.getP();

            inputs = u;
        }
    }
}