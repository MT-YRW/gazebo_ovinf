#ifndef FILTER_PASS_THROUGH_HPP
#define FILTER_PASS_THROUGH_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <algorithm>

#include "filter/filter_base.hpp"

namespace ovinf {

template <typename T = float>
class PassThroughFilter : public FilterBase<T> {
 public:
  PassThroughFilter() = delete;
  PassThroughFilter(YAML::Node const &config) : FilterBase<T>(config) {
    lower_bound_ = config["lower_bound"].as<T>();
    upper_bound_ = config["upper_bound"].as<T>();
  }

  virtual T operator()(T const &input) final {
    return std::max(lower_bound_, std::min(upper_bound_, input));
  }

  virtual void Reset() final {};

 private:
  T lower_bound_;
  T upper_bound_;
};

}  // namespace ovinf

#endif  // !FILTER_PASS_THROUGH_HPP
