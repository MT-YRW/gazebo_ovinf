#ifndef FILTER_MEAN_HPP
#define FILTER_MEAN_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <iostream>

#include "filter/filter_base.hpp"
#include "utils/history_buffer.hpp"

namespace ovinf {

// Scalar case
template <typename T = float>
class MeanFilter : public FilterBase<T> {
 public:
  MeanFilter() = delete;
  MeanFilter(YAML::Node const &config) : FilterBase<T>(config) {
    history_length_ = config["history_length"].as<size_t>();
    lower_bound_ = this->ReadYamlParam(config["lower_bound"]);
    upper_bound_ = this->ReadYamlParam(config["upper_bound"]);

    // In this case dimension_ is 1
    history_buffer_ =
        std::make_unique<HistoryBuffer<T>>(this->dimension_, history_length_);
  }

  virtual T Filter(T const &input) final {
    // This is a piece of shit...
    history_buffer_->AddObservation(Eigen::Matrix<T, 1, 1>(
        std::max(lower_bound_, std::min(upper_bound_, input))));
    auto obs = history_buffer_->GetObsHistory();
    return obs.mean();
  }

  virtual void Reset() final { history_buffer_->Reset(); };

 private:
  size_t history_length_;
  T lower_bound_;
  T upper_bound_;

  HistoryBuffer<T>::Ptr history_buffer_;
};

// Vector case
template <typename Scalar, int Rows, int Options, int MaxRows>
class MeanFilter<Eigen::Matrix<Scalar, Rows, 1, Options, MaxRows, 1>>
    : public FilterBase<Eigen::Matrix<Scalar, Rows, 1, Options, MaxRows, 1>> {
  using T = Eigen::Matrix<Scalar, Rows, 1, Options, MaxRows, 1>;

 public:
  MeanFilter() = delete;
  MeanFilter(YAML::Node const &config) : FilterBase<T>(config) {
    history_length_ = config["history_length"].as<size_t>();
    lower_bound_ = this->ReadYamlParam(config["lower_bound"]);
    upper_bound_ = this->ReadYamlParam(config["upper_bound"]);

    if constexpr (is_eigen_vector_v<T>) {
      this->dimension_ = lower_bound_.rows();

      if (lower_bound_.size() != this->dimension_ ||
          upper_bound_.size() != this->dimension_) {
        throw std::runtime_error(
            "MeanFilter: lower_bound or upper_bound size doesn't match "
            "the vector dimension. "
            "Expected dimension: " +
            std::to_string(this->dimension_));
      }
    }

    history_buffer_ = std::make_unique<HistoryBuffer<Scalar>>(this->dimension_,
                                                              history_length_);
  }

  virtual T Filter(T const &input) final {
    history_buffer_->AddObservation(
        input.cwiseMin(upper_bound_).cwiseMax(lower_bound_));
    auto obs_mat = Eigen::Map<Eigen::Matrix<typename T::Scalar, Eigen::Dynamic,
                                            Eigen::Dynamic, Eigen::RowMajor>>(
        history_buffer_->GetObsHistory().data(), history_length_,
        this->dimension_);
    return obs_mat.colwise().mean().transpose();
  }

  virtual void Reset() final { history_buffer_->Reset(); };

 private:
  size_t history_length_;
  T lower_bound_;
  T upper_bound_;

  HistoryBuffer<typename T::Scalar>::Ptr history_buffer_;
};

}  // namespace ovinf

#endif  // !FILTER_MEAN_HPP
