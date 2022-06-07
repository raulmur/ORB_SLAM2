/*
*  Software License Agreement (BSD License)
*
*  Copyright (c) 2016-2022, Natalnet Laboratory for Perceptual Robotics
*  All rights reserved.
*  Redistribution and use in source and binary forms, with or without modification, are permitted provided
*  that the following conditions are met:
*
*  1. Redistributions of source code must retain the above copyright notice, this list of conditions and
*     the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
*     the following disclaimer in the documentation and/or other materials provided with the distribution.
*
*  3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
*     promote products derived from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
*  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
*  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*  Authors:
*
*  Luiz Correia
 */

#include "dnn_based_mt.h"

using namespace std;
using namespace cv;
using namespace cv::dnn;

DnnBasedMT::DnnBasedMT(const string &model, const string &config, const string &framework,
                       const vector<string> output_layers_name,
                       const std::vector<DnnObjectClass> valid_classes,
                       Backend backend_id,
                       Target target_id)
    : output_layers_name_(output_layers_name),
      backend_id_(backend_id),
      target_id_(target_id),
      valid_classes_(valid_classes)
{
  net_ = readNet(model, config, framework);
  net_.setPreferableBackend(backend_id);
  net_.setPreferableTarget(target_id);
  dilation_size_ = 15;
  dilatation_kernel_ = getStructuringElement(cv::MORPH_ELLIPSE,
                                             Size(2 * dilation_size_ + 1, 2 * dilation_size_ + 1),
                                             Point(dilation_size_, dilation_size_));
}

DnnBasedMT::~DnnBasedMT()
{
}

void DnnBasedMT::dilateMask(const cv::Mat & img_in, cv::Mat & img_out)
{
  erode(img_in, img_out, dilatation_kernel_);
}