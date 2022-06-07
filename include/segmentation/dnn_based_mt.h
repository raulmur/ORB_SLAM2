/*
*  Software License Agreement (BSD License)
*
*  Copyright (c) 2016-2022, Natalnet Laboratory for Perceptual Robotics
*  All rights reserved.
*  Redistribution and use in source and binary forms, with or without modification, are permitted
* provided
*  that the following conditions are met:
*
*  1. Redistributions of source code must retain the above copyright notice, this list of
* conditions and
*     the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright notice, this list of
* conditions and
*     the following disclaimer in the documentation and/or other materials provided with the
* distribution.
*
*  3. Neither the name of the copyright holder nor the names of its contributors may be used to
* endorse or
*     promote products derived from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY,
*  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE
*  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
 */

#pragma once

#include "object_detected.h"

#include <opencv2/dnn.hpp>
#include <vector>
#include <unordered_map>

class DnnObjectClass : public ObjectDetected
{
public:
  DnnObjectClass(std::string name, uint8_t code) : ObjectDetected(name, code, cv::Rect())
  {
  }

  DnnObjectClass(const DnnObjectClass &dnn_obj_class) : DnnObjectClass(dnn_obj_class.class_name, dnn_obj_class.class_id)
  {
  }

  bool operator==(const DnnObjectClass &other)
  {
    return (other.class_id == class_id && other.class_name == class_name);
  }

  bool operator==(uint8_t code)
  {
    return (class_id == code);
  }
};

class DnnBasedMT
{
public:
  /**
   * @brief Initialize the object with a DNN model
   * @see for more information, see opencv/dnn/dnn.hpp
   * @param[in] model path to the model file
   * @param[in] config path to the config file
   * @param[in] backend_id an integer that represents the id of the backend API @see cv::dnn::Backend
   * @param[in] target_id an integer that represents the id of the target device @see cv::dnn::Target
   */
  DnnBasedMT(const std::string &model, const std::string &config, const std::string &framework,
             const std::vector<std::string> output_layers_name,
             const std::vector<DnnObjectClass> valid_classes,
             const cv::dnn::Backend backend_id = cv::dnn::Backend::DNN_BACKEND_DEFAULT,
             const cv::dnn::Target target_id = cv::dnn::Target::DNN_TARGET_CPU);

  virtual ~DnnBasedMT();

  /**
   * @brief This function should perform the instance segmentation task
   *
   * @param[in] img_in  cv::Mat RGB image
   * @param[out] img_out  cv::Mat image which is a binary mask
   */
  virtual void segment(const cv::Mat &img_in, cv::Mat &img_out, float threshold) = 0;

  /**
   * @brief Perform object detection passing a image
   *
   * @param[in] img_in cv::Mat RGB image
   * @return ObjectDetected , @see "./object_detected.h"
   */
  virtual ObjectDetected detect(const cv::Mat &img_in, float threshold) = 0;

protected:

  virtual void dilateMask(const cv::Mat & img_in, cv::Mat & img_out);

  std::string model_;
  std::string config_;
  std::string framework_;
  cv::dnn::Backend backend_id_;
  cv::dnn::Target target_id_;
  cv::dnn::Net net_;
  std::vector<std::string> output_layers_name_;
  std::vector<DnnObjectClass> valid_classes_;
  std::vector<cv::Mat> dnn_output_;
  int dilation_size_;
  cv::Mat dilatation_kernel_;

};