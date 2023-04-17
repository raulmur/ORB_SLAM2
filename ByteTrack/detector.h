#pragma once
#include <opencv2/opencv.hpp>
#include "onnxruntime-linux-x64-gpu-1.11.0/include/onnxruntime_cxx_api.h"
#include <utility>

#include "dataType.h"

class YOLO
{
public:
    explicit YOLO(std::nullptr_t) {};
    YOLO(const std::string& modelPath,
                 const bool& isGPU,
                 const cv::Size& inputSize);

    std::vector<Detection> detect(cv::Mat &image, const float& confThreshold, const float& iouThreshold);

private:
    Ort::Env env{nullptr};
    Ort::SessionOptions sessionOptions{nullptr};
    Ort::Session session{nullptr};

    void preprocessing(cv::Mat &image, float*& blob, std::vector<int64_t>& inputTensorShape);
    std::vector<Detection> postprocessing(const cv::Size& resizedImageShape,
                                          const cv::Size& originalImageShape,
                                          std::vector<Ort::Value>& outputTensors,
                                          const float& confThreshold, const float& iouThreshold);

    static void getBestClassInfo(std::vector<float>::iterator it, const int& numClasses,
                                 float& bestConf, int& bestClassId);

    std::vector<const char*> inputNames;
    std::vector<const char*> outputNames;
    bool isDynamicInputShape{};
    cv::Size2f inputImageShape;

};