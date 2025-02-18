// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
class NumberClassifier
{
public:
  NumberClassifier(
    const std::string & model_path, const std::string & label_path, const double threshold,
    const std::vector<std::string> & ignore_classes = {});

  cv::Mat extractNumber(const cv::Mat & src, Armor & armor);
  void classify(const cv::Mat &src, Armor & armor);
  
  void eraseIgnoreClasses(std::vector<Armor> & armors);

  double threshold;

private:
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
