#ifndef POLICY_CONTROLLER_HPP
#define POLICY_CONTROLLER_HPP

#include <chrono>

#include "controller/controller_base.hpp"
#include "filter/filter_mean.hpp"
#include "ovinf/ovinf_factory.hpp"

namespace ovinf {

class PolicyController : public ControllerBase<float> {
 public:
  using Ptr = std::shared_ptr<PolicyController>;

  PolicyController() = delete;
  ~PolicyController() = default;

  PolicyController(RobotBase<float>::RobotPtr robot, YAML::Node const& config)
      : ControllerBase<float>(robot, config),
        decimation_(config["decimation"].as<int>()) {
    p_gains_ = VectorT::Zero(robot_->joint_size_);
    d_gains_ = VectorT::Zero(robot_->joint_size_);

    for (auto const& pair : robot_->joint_names_) {
      p_gains_(pair.second) = config["p_gains"][pair.first].as<float>();
      d_gains_(pair.second) = config["d_gains"][pair.first].as<float>();
    }

    command_ = VectorT::Zero(3);
    counter_ = 0;

    inference_net_ = ovinf::PolicyFactory::CreatePolicy(config["inference"]);
  }

  virtual void WarmUp() final {
    inference_net_->WarmUp(
        {.command = command_,
         .ang_vel = robot_->observer_->AngularVelocity(),
         .proj_gravity = robot_->observer_->ProjGravity(),
         .joint_pos = robot_->observer_->JointActualPosition().segment(0, 12),
         .joint_vel = robot_->observer_->JointActualVelocity().segment(0, 12)});
    // inference_net_->PrintInfo();
  }

  virtual void Init() final {
    counter_ = 0;
    ready_ = true;
  }

  virtual void Step(bool set_target = true) final {
    if (!ready_) {
      std::cerr << "PolicyController not ready" << std::endl;
      return;
    }

    // TODO: Test vel filter
    // static MeanFilter<VectorT> vel_filter(YAML::Load(R"(
    //   type: "Mean"
    //   history_length: 3
    //   lower_bound: [-50.0, -50.0, -50.0, -50.0, -50.0, -50.0,
    //                 -50.0, -50.0, -50.0, -50.0, -50.0, -50.0]
    //   upper_bound: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0,
    //                 50.0, 50.0, 50.0, 50.0, 50.0, 50.0]
    // )"));
    //
    // if (counter_++ % decimation_ == 0) {
    //   auto err = inference_net_->InferUnsync(
    //       {.command = command_,
    //        .ang_vel = robot_->observer_->AngularVelocity(),
    //        .proj_gravity = robot_->observer_->ProjGravity(),
    //        .joint_pos = robot_->observer_->JointActualPosition().segment(0,
    //        12), .joint_vel = vel_filter.Filter(
    //            robot_->observer_->JointActualVelocity().segment(0, 12))});
    // }

    if (counter_++ % decimation_ == 0) {
      auto err = inference_net_->InferUnsync(
          {.command = command_,
           .ang_vel = robot_->observer_->AngularVelocity(),
           .proj_gravity = robot_->observer_->ProjGravity(),
           .joint_pos = robot_->observer_->JointActualPosition().segment(0, 12),
           .joint_vel =
               robot_->observer_->JointActualVelocity().segment(0, 12)});
    }

    if (set_target) {
      auto target_pos = inference_net_->GetResult();
      if (target_pos.has_value()) {
        for (size_t i = 0; i < 12; i++) {
          robot_->executor_->JointTargetPosition()[i] = target_pos.value()[i];
        }
        robot_->executor_->JointTargetPosition()[12] = 0.0;
        robot_->executor_->JointTargetPosition()[13] = 0.0;
      } else {
        // std::cout << "target pos is empty" << std::endl;
      }
      ComputeJointPd();
    }
  }

  virtual void Stop() final { ready_ = false; }

  VectorT& GetCommand() { return command_; }

 private:
  const int decimation_;

  VectorT command_;
  size_t counter_ = 0;
  ovinf::BasePolicy<float>::BasePolicyPtr inference_net_;
};

}  // namespace ovinf

#endif  // !POLICY_CONTROLLER_HPP
