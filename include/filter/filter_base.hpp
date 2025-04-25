#ifndef FILTER_BASE_HPP
#define FILTER_BASE_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>

#include "utils/traits.h"

namespace ovinf {

/**
 * @brief Filter base class.
 */
template <typename T = float>
class FilterBase {
 public:
  FilterBase() = delete;
  constexpr FilterBase(YAML::Node const &config) {
    if constexpr (is_eigen_vector_v<T>) {
      // dimension_ is read from the YAML file
      dimension_ = -1;
    } else {
      this->dimension_ = 1;
    }
  }

  virtual T operator()(T const &input) { return input; }

  virtual void Reset() {};

 protected:
  /**
   * @brief Read from YAML vector or scalar
   *
   * @param[in] node Yaml node
   * @return value
   */
  static T ReadYamlParam(YAML::Node const &node) {
    if constexpr (is_eigen_vector_v<T>) {
      return Eigen::Map<T>(node.as<std::vector<typename T::Scalar>>().data(),
                           node.size());
    } else {
      return node.as<T>();
    }
  }

 protected:
  size_t dimension_ = 0;
};

}  // namespace ovinf

#endif  // !FILTER_BASE_HPP
