/**
 * @file inference_humanoid.h
 * @brief ovinf for humanoid gym policy.
 * @author Dknt
 * @date 2025-4-18
 */

#ifndef OVINF_HUMANOID_H
#define OVINF_HUMANOID_H

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <iostream>
#include <map>
#include <openvino/openvino.hpp>
#include <optional>
#include <string>
#include <thread>

#include "atomicops.h"
#include "ovinf.hpp"
#include "readerwriterqueue.h"
#include "utils/history_buffer.hpp"
#include "utils/realtime_setting.hpp"

namespace ovinf {

class HumanoidPolicy : public BasePolicy<float> {
  using MatrixT = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
  using VectorT = Eigen::Matrix<float, Eigen::Dynamic, 1>;

 public:
  HumanoidPolicy() = delete;
  ~HumanoidPolicy();

  HumanoidPolicy(const YAML::Node &config);

  /**
   * @brief Policy warmup
   *
   * @param[in] obs_pack Proprioceptive observation
   * @param[in] num_itrations Warmup iterations
   * @return Is warmup done successfully.
   */
  virtual bool WarmUp(ProprioceptiveObservation<float> const &obs_pack,
                      size_t num_itrations = 10) final;

  /**
   * @brief Set observation, run inference.
   *
   * @param[in] obs_pack Proprioceptive observation
   * @return Is inference started immidiately.
   */
  virtual bool InferUnsync(
      ProprioceptiveObservation<float> const &obs_pack) final;

  /**
   * @brief Get resulting target_joint_pos
   *
   * @param[in] timeout Timeout in microseconds
   */
  virtual std::optional<VectorT> GetResult(const size_t timeout = 1000) final;

  virtual void PrintInfo() final;

 private:
  void WorkerThread();

 private:
  // Threading
  std::atomic<bool> inference_done_{false};
  std::atomic<bool> exiting_{false};
  std::thread worker_thread_;

  // OpenVINO inference
  ov::CompiledModel compiled_model_;
  ov::InferRequest infer_request_;
  ov::Output<const ov::Node> input_info_;

  // Infer data
  std::map<std::string, size_t> joint_names_;
  float cycle_time_;
  VectorT joint_default_position_;

  size_t single_obs_size_;
  size_t obs_buffer_size_;
  size_t action_size_;

  float action_scale_;
  float obs_scale_ang_vel_;
  float obs_scale_lin_vel_;
  float obs_scale_command_;
  float obs_scale_dof_pos_;
  float obs_scale_dof_vel_;
  float obs_scale_proj_gravity_;
  float clip_action_;

  // Buffer
  moodycamel::ReaderWriterQueue<VectorT> input_queue_;
  std::shared_ptr<HistoryBuffer<float>> obs_buffer_;
  VectorT last_action_;
  VectorT latest_target_;

  // Clock
  bool gait_start_ = false;
  std::chrono::steady_clock::time_point gait_start_time_;
  std::chrono::high_resolution_clock::time_point infer_start_time_;
  std::chrono::high_resolution_clock::time_point infer_end_time_;
};
}  // namespace ovinf

#endif  // !OVINF_HUMANOID_HPP
