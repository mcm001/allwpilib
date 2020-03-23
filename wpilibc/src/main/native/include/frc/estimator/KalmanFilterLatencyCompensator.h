/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <map>
#include <utility>

#include <units/units.h>

namespace frc {
template <int States, int Inputs, int Outputs, typename KalmanTypeFilter>
class KalmanFilterLatencyCompensator {
 public:
  struct ObserverState {
    Eigen::Matrix<double, States, 1> xHat;
    Eigen::Matrix<double, States, States> errorCovariances;
    Eigen::Matrix<double, Inputs, 1> inputs;

    ObserverState(const KalmanTypeFilter& observer, const Eigen::Matrix<double, Inputs, 1>& u)
        : xHat(observer.Xhat()), errorCovariances(observer.P()), inputs(u) {}
  };

  void AddObserverState(KalmanTypeFilter observer,
                        Eigen::Matrix<double, Inputs, 1> u,
                        units::second_t timestamp) {
    m_pastObserverStates.insert(std::pair<units::second_t, ObserverState>{
        timestamp, ObserverState{observer, u}});

    if (m_pastObserverStates.size() > kMaxPastObserverStates) {
      m_pastObserverStates.erase(m_pastObserverStates.begin());
    }
  }

  void ApplyPastMeasurement(KalmanTypeFilter observer,
                            units::second_t nominalDt,
                            Eigen::Matrix<double, Outputs, 1> y,
                            units::second_t timestamp) {
    auto lowIterator = m_pastObserverStates.lower_bound(timestamp);
    auto highIterator = m_pastObserverStates.upper_bound(timestamp);

    typename std::map<units::second_t, ObserverState>::iterator closestEntry;

    // Find the entry which is closest in time to the timestamp.
    if (lowIterator != m_pastObserverStates.end() &&
        highIterator != m_pastObserverStates.end()) {
      closestEntry = units::math::abs(timestamp - lowIterator->first) <
                             units::math::abs(timestamp - highIterator->first)
                         ? lowIterator
                         : highIterator;
    } else if (lowIterator == m_pastObserverStates.end() &&
               highIterator == m_pastObserverStates.end()) {
      // State map was empty, which means that we got a measurement right at
      // startup. The only thing we can do is ignore the measurement.
      return;
    } else {
      closestEntry = lowIterator != m_pastObserverStates.end() ? lowIterator
                                                               : highIterator;
    }

    std::map<units::second_t, ObserverState> tailMap{
        closestEntry, m_pastObserverStates.end()};

    units::second_t lastTimestamp = tailMap.begin()->first - nominalDt;
    for (const auto& [key, st] : tailMap) {
      if (key == closestEntry->first) {
        observer.SetP(st.errorCovariances);
        observer.SetXhat(st.xHat);

        // Note that we correct the observer with inputs closest in time to the
        // measurement This makes the assumption that the dt is small enough
        // that the difference between the measurement time and the time that
        // the inputs were captured at is very small.
        observer.Correct(st.inputs, y);
      }
      observer.Predict(st.inputs, key - lastTimestamp);
      lastTimestamp = key;
    }
  }

 private:
  static constexpr uint32_t kMaxPastObserverStates = 300;
  std::map<units::second_t, ObserverState> m_pastObserverStates;
};
}  // namespace frc
