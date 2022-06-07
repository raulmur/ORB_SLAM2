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

#include <opencv2/opencv.hpp>
#include <string>

/**
 * @brief Store a custom object related to any class
 *
 */
struct ObjectDetected
{
  std::string class_name;
  uint8_t class_id;
  cv::Rect bouding_box;

  ObjectDetected(const std::string &name, const uint8_t id, const cv::Rect &b_box) : class_name(name), class_id(id), bouding_box(b_box)
  {
  }

  /**
  * @brief Check if the object belongs to the same class
   */
  virtual bool operator==(const ObjectDetected &other)
  {
    return (other.class_id == class_id);
  }
};