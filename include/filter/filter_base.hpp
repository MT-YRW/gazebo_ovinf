#ifndef FILTER_BASE_HPP
#define FILTER_BASE_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

namespace ovinf {

/**
 * @brief Filter base class.
 */
template <typename T = float>
class FilterBase {
 public:
  FilterBase() = delete;
  FilterBase(YAML::Node const &config) {}

  virtual T operator()(T const &input) { return input; }

  virtual void Reset() {};

 private:
};

}  // namespace ovinf

#endif  // !FILTER_BASE_HPP
