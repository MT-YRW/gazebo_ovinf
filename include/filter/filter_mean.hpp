#ifndef FILTER_MEAN_HPP
#define FILTER_MEAN_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <algorithm>

#include "filter/filter_base.hpp"
#include "utils/history_buffer.hpp"

namespace ovinf {

template <typename T = float>
class MeanFilter : public FilterBase<T> {
 public:
  MeanFilter() = delete;
  MeanFilter(YAML::Node const &config) : FilterBase<T>(config) {
    buffer_size_ = config["buffer_size"].as<size_t>();
    buffer_ = std::make_shared<HistoryBuffer<T>>(1, buffer_size_);
  }

  virtual T operator()(T const &input) final {
    //
    return 0;
  }

  virtual void Reset() final {};

 private:
  size_t buffer_size_;
  HistoryBuffer<T>::Ptr buffer_;
};

}  // namespace ovinf

#endif  // !FILTER_MEAN_HPP
