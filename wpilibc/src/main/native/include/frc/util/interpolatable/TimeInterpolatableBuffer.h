#pragma once

#include <units/time.h>

#include <vector>
#include <array>
#include <wpi/MathExtras.h>

namespace frc {

template <typename T>
class TimeInterpolatableBuffer {
public:
  void addSample(units::second_t time, T sample) {
    // Add the new state into the vector.
    m_pastSnapshots.emplace_back(time, sample);

    // Remove the oldest snapshot if the vector exceeds our maximum size.
    if (m_pastSnapshots.size() > kMaxPastObserverStates) {
      m_pastSnapshots.erase(m_pastSnapshots.begin());
    }
  }

  void clear() {
    m_pastSnapshots.clear();
  }

  /**
   * Sample the buffer at the given time. If there are no elements in the buffer, calling this function results in undefined behavior.
   */
  T getSample(units::second_t time) {

    // We will perform a binary search to find the index of the element in the
    // vector that has a timestamp that is equal to or greater than the vision
    // measurement timestamp.

    if(m_pastSnapshots.size() < 2) return m_pastSnapshots[0].second;

    int low = 0;
    int high = m_pastSnapshots.size() - 1;

    while(low != high) {
      int mid = (low + high) / 2.0;
      if (m_pastSnapshots[mid].first < time) {
        // This index and everything under it are less than the requested
        // time. Therefore, we can discard them.
        low = mid + 1;
      } else {
        // t is at least as large as the element at this index. This means that
        // anything after it cannot be what we are looking for.
        high = mid;
      }
    }

    // Because low and high are now the same (both "high"), we decrement low
    low -= 1;

    std::pair<units::second_t, T> bottomBound = m_pastSnapshots[low];
    std::pair<units::second_t, T> topBound = m_pastSnapshots[high];

    return wpi::Lerp(bottomBound.second, topBound.second, (time - bottomBound.first) / (topBound.first - bottomBound.first));
  }

 private:
  static constexpr uint32_t kMaxPastObserverStates = 300;
  std::vector<std::pair<units::second_t, T>>
      m_pastSnapshots;
};

/**
 * Perform linear interpolation between two values.
 * @param startValue The value to start at.
 * @param endValue The value to end at.
 * @param t How far between the two values to interpolate. This is clamped to [0, 1].
 */
template <typename T>
const T Interpolate(const T& startValue, const T& endValue, double t) {
  return startValue + (endValue - startValue) * std::clamp(t, 0, 1);
}

}