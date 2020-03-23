#pragma once

#include <Eigen/Core>

namespace frc {
template <int States, int Inputs, int Outputs, typename KalmanTypeFilter>
class ObserverState {
 public:
  const Eigen::Matrix<double, States, 1> xHat;
  const Eigen::Matrix<double, States, States> errorCovariances;
  const Eigen::Matrix<double, Inputs, 1> inputs;

  ObserverState(KalmanTypeFilter<States, Inputs, Outputs> observer,
                Eigen::Matrix<double, Inputs, 1> u) {
    xHat = observer->XHat();
    errorCovariances = observer->P();
    inputs = u;
  }
};
}  // namespace frc
