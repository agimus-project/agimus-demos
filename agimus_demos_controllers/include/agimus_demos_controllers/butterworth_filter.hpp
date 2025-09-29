#ifndef AGIMUS_DEMOS_CONTROLLERS__BUTTERWORTH_FILTER_
#define AGIMUS_DEMOS_CONTROLLERS__BUTTERWORTH_FILTER_

#include <Eigen/Dense>
#include <array>

namespace agimus_demos_controllers {
class ButterworthFilter {
 public:
  ButterworthFilter(std::array<double, 2>& a, std::array<double, 3>& b) {
    raw_ = Eigen::Vector3d::Zero(3);
    filtered_ = Eigen::Vector2d::Zero(2);
    a_reverse_ =
        Eigen::Map<Eigen::Vector2d, Eigen::Unaligned>(a.data(), a.size());
    b_reverse_ =
        Eigen::Map<Eigen::Vector3d, Eigen::Unaligned>(b.data(), b.size());
    a_reverse_ = a_reverse_.reverse();
    b_reverse_ = b_reverse_.reverse();
  }

  double update(const double observation) {
    raw_[2] = observation;
    filtered_[1] = b_reverse_.dot(raw_) - a_reverse_.dot(filtered_);
    raw_.head<2>() = raw_.tail<2>();
    filtered_[0] = filtered_[1];

    return filtered_[1];
  }

 private:
  Eigen::Vector2d a_reverse_;
  Eigen::Vector3d b_reverse_;
  Eigen::Vector2d filtered_;
  Eigen::Vector3d raw_;
};
}  // namespace agimus_demos_controllers

#endif  // AGIMUS_DEMOS_CONTROLLERS__BUTTERWORTH_FILTER_
