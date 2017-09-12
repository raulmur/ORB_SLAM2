#include "System.h"

#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>
#include <chrono>

int main(int argc, char **argv)
{
    const int SUCCESS{0};
    const int FAILURE{-1};

    int retVal{SUCCESS};

    if (4 == argc)
    {
        std::string videoFile{argv[1]};
        std::string vocabularyFile{argv[2]};
        std::string settingsFile{argv[3]};

        cv::VideoCapture videoCapture(videoFile);

        if (true == videoCapture.isOpened())
        {
            ORB_SLAM2::System SLAM(vocabularyFile, settingsFile, ORB_SLAM2::System::MONOCULAR);

            bool readValue{true};
            while (true == readValue)
            {
                cv::Mat frame;

                readValue = videoCapture.read(frame);
                if (true == readValue)
                {
                    auto currentVideoTime = videoCapture.get(CV_CAP_PROP_POS_MSEC);
                    SLAM.TrackMonocular(frame, currentVideoTime);
                    std::this_thread::sleep_for(std::chrono::milliseconds(30));
                }
                else
                {
                    readValue = false;
                }
            }

            SLAM.Shutdown();
            SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        }
        else
        {
            std::cerr << "Cannot open the video file " << videoFile << '\n';
            retVal = FAILURE;
        }
    }
    else
    {
        std::cerr << "Invalid input\n";
        std::cerr << "Usage: mono_video VideoFile VocabularyFile SettingsFile\n";
        std::cerr << "Example: /ORB_SLAM2/Examples/Monocular/mono_video "
                  << "/ORB_SLAM2/Examples/video.mp4 "
                  << "/ORB_SLAM2/Vocabulary/ORBVoc.txt "
                  << "/ORB_SLAM2/Examples/Monocular/TUM1.yaml\n";
        retVal = FAILURE;
    }

    return retVal;
}
