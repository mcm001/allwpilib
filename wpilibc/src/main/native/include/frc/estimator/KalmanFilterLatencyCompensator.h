/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <utility>
#include <vector>

#include <units/units.h>

namespace frc {
template <int States, int Inputs, int Outputs, typename KalmanFilterType>
class KalmanFilterLatencyCompensator {
 public:
  struct ObserverSnapshot {
    Eigen::Matrix<double, States, 1> xHat;
    Eigen::Matrix<double, States, States> errorCovariances;
    Eigen::Matrix<double, Inputs, 1> inputs;

    ObserverSnapshot(const KalmanFilterType& observer,
                     const Eigen::Matrix<double, Inputs, 1>& u)
        : xHat(observer.Xhat()), errorCovariances(observer.P()), inputs(u) {}
  };

  void AddObserverState(KalmanFilterType observer,
                        Eigen::Matrix<double, Inputs, 1> u,
                        units::second_t timestamp) {
    m_pastObserverSnapshots.emplace_back(timestamp,
                                         ObserverSnapshot{observer, u});

    if (m_pastObserverSnapshots.size() > kMaxPastObserverStates) {
      m_pastObserverSnapshots.erase(m_pastObserverSnapshots.begin());
    }
  }

  void ApplyPastMeasurement(KalmanFilterType observer,
                            units::second_t nominalDt,
                            Eigen::Matrix<double, Outputs, 1> y,
                            units::second_t timestamp) {
    if (m_pastObserverSnapshots.size() == 0) {
      // State map was empty, which means that we got a measurement right at
      // startup. The only thing we can do is ignore the measurement.
      return;
    }

    // We will perform a binary search to find the index of the element in the
    // vector that has a timestamp that is equal to or greater than the vision
    // measurement timestamp.

    // This index starts at one because we use the previous state later on, and
    // we always want to have a "previous state".
    int low = 1;
    int high = m_pastObserverSnapshots.size() - 1;

    while (low != high) {
      int mid = (low + high) / 2.0;
      if (m_pastObserverSnapshots[mid].first < timestamp) {
        // This index and everything under it are less than the requested
        // timestamp. Therefore, we can discard them.
        low = mid + 1;
      } else {
        // t is at least as large as the element at this index. This means that
        // anything after it cannot be what we are looking for.
        high = mid;
      }
    }

    // We are simply assigning this index to a new variable to avoid confusion
    // with variable names.
    int index = low;

    // High and Low should be the same. The sampled timestamp is greater than or
    // equal to the vision pose timestamp. We will now find the entry which is
    // closest in time to the requested timestamp.

    int indexOfClosestEntry =
        units::math::abs(timestamp - m_pastObserverSnapshots[index - 1].first) <
                units::math::abs(timestamp -
                                 m_pastObserverSnapshots[index].first)
            ? index - 1
            : index;
    std::pair<units::second_t, ObserverSnapshot> closestEntry =
        m_pastObserverSnapshots[indexOfClosestEntry];

    decltype(m_pastObserverSnapshots) snapshotsToUse{
        m_pastObserverSnapshots.begin() + indexOfClosestEntry,
        m_pastObserverSnapshots.end()};
    decltype(m_pastObserverSnapshots) newSnapshots;

    units::second_t lastTimestamp = snapshotsToUse.front().first - nominalDt;
    for (const auto& pair : snapshotsToUse) {
      observer.Predict(pair.second.inputs, pair.first - lastTimestamp);
      lastTimestamp = pair.first;

      if (pair.first == closestEntry.first) {
        observer.SetP(pair.second.errorCovariances);
        observer.SetXhat(pair.second.xHat);

        // Note that we correct the observer with inputs closest in time to the
        // measurement This makes the assumption that the dt is small enough
        // that the difference between the measurement time and the time that
        // the inputs were captured at is very small.
        observer.Correct(pair.second.inputs, y);
      }

      newSnapshots.emplace_back(pair.first,
                                ObserverSnapshot{observer, pair.second.inputs});
    }

    // Replace observer snapshots that haven't been corrected by a measurement
    // with ones that have been corrected.
    // TODO This might mess with our binary search.
    m_pastObserverSnapshots.insert(m_pastObserverSnapshots.end(),
                                   newSnapshots.begin(), newSnapshots.end());
  }

 private:
  static constexpr uint32_t kMaxPastObserverStates = 300;
  std::vector<std::pair<units::second_t, ObserverSnapshot>>
      m_pastObserverSnapshots;
};
}  // namespace frc
