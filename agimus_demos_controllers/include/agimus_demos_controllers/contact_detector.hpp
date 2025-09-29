#ifndef AGIMUS_DEMOS_CONTROLLERS__CONTACT_DETECTOR_
#define AGIMUS_DEMOS_CONTROLLERS__CONTACT_DETECTOR_

#include <pinocchio/spatial/force.hpp>

namespace agimus_demos_controllers {
class ContactDetector {
 public:
  ContactDetector() {}

  void update(const pinocchio::Force &force) {
    // Use lower threshold in case robot is in contact.
    // Otherwise require higher force to enter this state.
    const double thresh =
        last_in_contact_ ? lower_threshold_ : upper_threshold_;
    in_contact_ = (force.linear().cwiseProduct(mask_)).norm() > thresh;
  }

  bool in_contact() {
    // If state changes and minimum number
    // of samples since last change elapsed
    if (last_in_contact_ != in_contact_ &&
        samples_since_switch_ > hysteresis_samples_) {
      last_in_contact_ = in_contact_;
      samples_since_switch_ = 0;
    } else {
      samples_since_switch_++;
      // Prevent integer overflow
      if (samples_since_switch_ > 5 * hysteresis_samples_) {
        samples_since_switch_ = hysteresis_samples_;
      }
    }

    // Return latched version
    return last_in_contact_;
  }

  void set_hysteresis_samples(const unsigned int hysteresis_samples) {
    hysteresis_samples_ = hysteresis_samples;
  }

  unsigned int get_hysteresis_samples() const { return hysteresis_samples_; }

  void set_lower_threshold(const double lower_threshold) {
    lower_threshold_ = lower_threshold;
  }

  double get_lower_threshold() const { return lower_threshold_; }

  void set_upper_threshold(const double upper_threshold) {
    upper_threshold_ = upper_threshold;
  }

  double get_upper_threshold() const { return upper_threshold_; }

  void set_axis_mask(const std::string &axis_mask) {
    const std::string axies = "xyz";
    for (std::size_t i = 0; i < 3; i++) {
      mask_[i] = axis_mask.find(axies[i]) != std::string::npos ? 1.0 : 0.0;
    }
  }

 private:
  bool last_in_contact_ = false;
  bool in_contact_ = false;
  unsigned int samples_since_switch_ = 0;

  unsigned int hysteresis_samples_ =
      0;  // Minimum number of samples between contact state switch.
  double lower_threshold_ =
      0.0;  // Force threshold used to switch in contact -> not in contact.
  double upper_threshold_ =
      0.0;  // Force threshold used to switch not in contact -> in contact.
  Eigen::Vector3d mask_;
};
}  // namespace agimus_demos_controllers

#endif  // AGIMUS_DEMOS_CONTROLLERS__CONTACT_DETECTOR_
