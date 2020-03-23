#pragma once

#include <map>

#include <units/units.h>

namespace frc {
template <int States, int Inputs, int Outputs, typename KalmanTypeFilter>
class KalmanFilterLatencyCompensator {
 public:
  class ObserverState {
   public:
    const Eigen::Matrix<double, States, 1> xHat;
    const Eigen::Matrix<double, States, States> errorCovariances;
    const Eigen::Matrix<double, Inputs, 1> inputs;

    ObserverState(KalmanTypeFilter<States, Inputs, Outputs> observer,
                  Eigen::Matrix<double, Inputs, 1> u) {
      xHat = observer.XHat();
      errorCovariances = observer.P();
      inputs = u;
    }
  };

  void AddObserverState(KalmanTypeFilter<States, Inputs, Outputs> observer,
                        Eigen::Matrix<double, Inputs, 1> u,
                        units::second_t timestamp) {
    m_pastObserverStates.insert(std::pair<units::second_t, ObserverState>{
        timestamp, ObserverState{observer, u}});

    if (m_pastObserverStates.size() > kMaxPastObserverStates) {
      m_pastObserverStates.erase(m_pastObserverStates.begin());
    }
  }

  void ApplyPastMeasurement(KalmanTypeFilter<States, Inputs, Outputs>* observer,
                            units::second_t nominalDt,
                            Eigen::Matrix<double, Outputs, 1> y,
                            units::second_t timestamp){
      // TODO
  };

 private:
  constexpr unsigned int kMaxPastObserverStates = 300;
  std::map<units::second_t, ObserverState> m_pastObserverStates;
};
}  // namespace frc
