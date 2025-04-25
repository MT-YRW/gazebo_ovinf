#include <iostream>

#include "filter/filter_factory.hpp"

int main() {
  // 1. Test None filter
  std::cout << "None filter test" << std::endl;
  auto none_filter = ovinf::FilterFactory::CreateFilter(YAML::Load(R"(
        type: "None"
      )"));

  // 2. Test Mean filter
  std::cout << "Path through filter test" << std::endl;
  auto float_filter = ovinf::FilterFactory::CreateFilter(YAML::Load(R"(
        type: "PassThrough"
        lower_bound: 0.0
        upper_bound: 1.0
      )"));

  std::cout << "Float pass through filter output: "
            << float_filter->operator()(-1.0f) << std::endl;

  ovinf::PassThroughFilter<Eigen::Matrix<float, -1, 1>> vector_filter(
      YAML::Load(R"(
        upper_bound: [0.0, 1.0, 2.0, 3.0]
        lower_bound: [-1.0, -2.0, -3.0, -4.0]
      )"));
  std::cout << "Vector pass through filter output: "
            << vector_filter(Eigen::Vector4f(1.0f, 2.0f, 3.0f, 4.0)).transpose()
            << std::endl;

  std::cout
      << "Vector pass through filter output: "
      << vector_filter(Eigen::Vector4f(-2.0f, -3.0f, -4.0f, -5)).transpose()
      << std::endl;

  // 3. Test mean filter
  std::cout << "Mean filter test" << std::endl;
  ovinf::MeanFilter<float> mean_filter(YAML::Load(R"(
        history_length: 3
        upper_bound: 1.0
        lower_bound: 2.0
      )"));
  mean_filter(1.0);
  mean_filter(2.0);
  std::cout << mean_filter(3.0) << std::endl;

  ovinf::MeanFilter<Eigen::VectorXf> mean_filter_vector(YAML::Load(R"(
        history_length: 3
        upper_bound: [0.0, 1.0, 2.0, 3.0]
        lower_bound: [-1.0, -2.0, -3.0, -4.0]
      )"));
  mean_filter_vector(Eigen::Vector4f(1.0f, 2.0f, 3.0f, 1));
  mean_filter_vector(Eigen::Vector4f(2.0f, 3.0f, 4.0f, 1));
  std::cout
      << mean_filter_vector(Eigen::Vector4f(3.0f, 4.0f, 5.0f, 1)).transpose()
      << std::endl;
  return 0;
}
