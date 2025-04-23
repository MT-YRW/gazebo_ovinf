#ifndef OVINF_FACTORY_HPP
#define OVINF_FACTORY_HPP

#include "ovinf.hpp"
#include "ovinf_humanoid.h"

namespace ovinf {

template <typename T = float>
ovinf::BasePolicy<T>::BasePolicyPtr PolicyFactory(const YAML::Node &config) {
  std::string policy_type = config["policy_type"].as<std::string>();
  if (policy_type == "Humanoid") {
    return std::make_shared<HumanoidPolicy>(config);
  } else {
    throw std::runtime_error("Unknown policy type: " + policy_type);
  }
}

}  // namespace ovinf
#endif  // !OVINF_FACTORY_HPP
