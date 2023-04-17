#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "ObjectTracking.h"

using namespace std;

// Load images from kitti dataset
void LoadImages(const std::string &strPathToSequence, std::vector<std::string> &vstrImageLeft,
                std::vector<std::string> &vstrImageRight, std::vector<double> &vTimestamps)
{
    std::ifstream fTimes;
    std::string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof())
    {
        std::string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    std::string strPrefixLeft = strPathToSequence + "/image_0/";
    std::string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for (int i = 0; i < nTimes; i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

int main(int argc, char **argv)
{
    // Check arguments
    if (argc != 3)
    {
        cerr << endl
             << "Usage: ./test_tracking path_to_sequence path_to_model" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[1]), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    // detector parameters
    const float confidence_threshold = 0.3f;
    const float iou_threshold = 0.4f;

    // tracker parameters
    const int nn_budget = 100;
    const float max_cosine_distance = 0.2;

    // create tracker
    const bool bUseGPU = true;
    ORB_SLAM2::ObjectTracking *tracker = new ORB_SLAM2::ObjectTracking(string(argv[2]), bUseGPU, cv::Size(640, 640), confidence_threshold, iou_threshold, max_cosine_distance, nn_budget);

    cout << endl
         << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl
         << endl;

    // Main loop
    for (int ni = 0; ni < nImages; ni++)
    {
        // Read left and right images from file
        cv::Mat imLeft = cv::imread(vstrImageLeft[ni]);
        cv::Mat imLeftToShow = imLeft.clone();

        if (imLeft.empty())
        {
            cerr << endl
                 << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        // track
        imLeft.copyTo(tracker->mImage);
        tracker->Run();

        vector<STrack> tracks = tracker->mTrackResults;
        // show image with detections
        for (unsigned long i = 0; i < tracks.size(); i++)
        {
            std::vector<float> tlwh = tracks[i].tlwh;
            bool vertical = tlwh[2] / tlwh[3] > 1.6;
            cv::putText(imLeftToShow, cv::format("%d %d %.2f", tracks[i].track_id, tracks[i].cls_id, tracks[i].score), cv::Point(tlwh[0], tlwh[1] - 5), 0, 0.6, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
            cv::rectangle(imLeftToShow, cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]), cv::Scalar(0, 0, 255), 2);
        }
        cv::imshow("image", imLeftToShow);

        // Press Q to stop
        if (cv::waitKey(1) == 'q')
            break;
    }

    cv::destroyAllWindows();

    return 0;
}