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

#include <cassert>

#include <mask_rcnn_dnn_mt.h>


using namespace std;
using namespace cv;
using namespace cv::dnn;

const string mask_rcnn_model_path = "/home/lcorreia/Personal/Projects/DYNAMIC-ORB_SLAM2/models/mask_rcnn_inception_v2_coco_2018_01_28/frozen_inference_graph.pb";
const string mask_rcnn_pbtxt_path = "/home/lcorreia/Personal/Projects/DYNAMIC-ORB_SLAM2/models/mask_rcnn_inception_v2_coco_2018_01_28/mask_rcnn_inception_v2_coco_2018_01_28.pbtxt";

MaskRcnnDnnMT::MaskRcnnDnnMT(Backend backend_id,
                            Target target_id,
                            vector<MaskRcnnClass> valid_classes)
   : DnnBasedMT(mask_rcnn_model_path, mask_rcnn_pbtxt_path, "tensorflow",
                {"detection_out_final", "detection_masks"},
                parseMaskRcnnClasses(valid_classes),
                backend_id,
                target_id)
{
}

MaskRcnnDnnMT::~MaskRcnnDnnMT()
{
}

void MaskRcnnDnnMT::segment(const Mat &img_in, Mat &img_out, float threshold)
{
 const float scale_factor = 1.0;
 const bool crop = false;
 const bool swapRB = true;

 Mat blob_img = blobFromImage(img_in, scale_factor, Size(), Scalar(), swapRB, crop);

 DnnBasedMT::net_.setInput(blob_img);

 net_.forward(dnn_output_, output_layers_name_);

 postProcessSegmentation(img_in, img_out, dnn_output_, threshold);
}

ObjectDetected MaskRcnnDnnMT::detect(const Mat &img_in, float threshold)
{
 assert((false, "MaskRcnnDnnMT::detect not implemented yet"));
 // just for compile
 (void)threshold;
 (void)img_in;
 uint8_t invalidClass = 255;
 return ObjectDetected("Invalid", invalidClass, Rect());
}

void MaskRcnnDnnMT::postProcessSegmentation(const Mat &in_img, Mat &out_img,
                                           const vector<Mat> dnn_guesses, const float threshold)
{
 // preparate the binary image (the mask)
 out_img = Mat(in_img.rows, in_img.cols, CV_8UC1, Scalar(255));

 // no copy performed
 Mat detections(dnn_guesses[0]);
 Mat masks(dnn_guesses[1]);

 // Output size of masks is NxCxHxW where
 // N - number of detected boxes
 // C - number of classes (excluding background)
 // HxW - segmentation shape
 const int num_detections = detections.size[2];
 const int num_classes = masks.size[1];

 // Each detection contains 7 elements,
 // @see https://github.com/matterport/Mask_RCNN/blob/master/mrcnn/model.py#L2487
 // - N - image index (useful just for sequence of images)
 // - class id
 // - score
 // - left coordinate
 // - top coordinate
 // - right coordinate
 // - bottom coordinate
 detections = detections.reshape(1, detections.total() / 7);

 uint16_t total_detected_objects = 0;

 for (int i = 0; i < num_detections; i++)
 {
   float score = detections.at<float>(i, 2);

   if (score > threshold)
   {
     uint8_t class_id = uint8_t(detections.at<float>(i, 1));
     vector<DnnObjectClass>::iterator it = find(valid_classes_.begin(), valid_classes_.end(), class_id);
     if (it != valid_classes_.end())
     {
       int left = int(in_img.cols * detections.at<float>(i, 3));
       int top = int(in_img.rows * detections.at<float>(i, 4));
       int right = int(in_img.cols * detections.at<float>(i, 5));
       int bottom = int(in_img.rows * detections.at<float>(i, 6));

       left = max(0, min(left, in_img.cols - 1));
       top = max(0, min(top, in_img.rows - 1));
       right = max(0, min(right, in_img.cols - 1));
       bottom = max(0, min(bottom, in_img.rows - 1));

       Rect box = Rect(left, top, right - left + 1, bottom - top + 1);

       // Extract the mask for the object
       Mat mask(masks.size[2], masks.size[3], CV_32F, masks.ptr<float>(i, class_id));

       Rect roi(Point(box.x, box.y), Point(box.x + box.width, box.y + box.height));

       resize(mask, mask, box.size());

       // convert the mask to a binary image
       mask = mask > threshold;
       bitwise_not(mask, mask);

       Mat mask_roi = out_img(roi);

       mask.copyTo(mask_roi);

       total_detected_objects++;
     }
   }
 }
 dilateMask(out_img,out_img);

}

vector<DnnObjectClass> MaskRcnnDnnMT::parseMaskRcnnClasses(const vector<MaskRcnnClass> &mask_rcnn_classes) const
{
 vector<DnnObjectClass> dnn_classes;

 for (size_t i = 0; i < mask_rcnn_classes.size(); i++)
 {
   DnnObjectClass dnn_class = parseMaskRcnnClass(mask_rcnn_classes[i]);
   dnn_classes.push_back(dnn_class);
 }

 return dnn_classes;
}