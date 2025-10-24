#ifndef OVINF_FACTORY_HPP
#define OVINF_FACTORY_HPP

#include "ovinf.hpp"
#include "ovinf_epsilon.h"
#include "ovinf_humanoid.h"
#include "ovinf_humanoid_stand.h"
#include "ovinf_locomotion.h"
#include "ovinf_perceptive.h"
#include "ovinf_perceptive_2.h"
#include "ovinf_perceptive_3.h"

namespace ovinf {

class PolicyFactory {
 public:
  template <typename T = float>
  static std::shared_ptr<ovinf::BasePolicy<T>> CreatePolicy(
      YAML::Node const &config) {
    std::string policy_type = config["policy_type"].as<std::string>();
    if (policy_type == "Humanoid") {
      return std::make_shared<HumanoidPolicy>(config);
    } else if (policy_type == "HumanoidStand") {
      return std::make_shared<HumanoidStandPolicy>(config);
    } else if (policy_type == "Epsilon") {
      return std::make_shared<EpsilonPolicy>(config);
    } else if (policy_type == "Perceptive") {
      return std::make_shared<PerceptivePolicy>(config);
    } else if (policy_type == "Perceptive2") {
      return std::make_shared<PerceptivePolicy2>(config);
    } else if (policy_type == "Perceptive3") {
      return std::make_shared<PerceptivePolicy3>(config);
    }else {
      throw std::invalid_argument("Unknown policy type: " + policy_type);
    }
  }
};

}  // namespace ovinf
#endif  // !OVINF_FACTORY_HPP
