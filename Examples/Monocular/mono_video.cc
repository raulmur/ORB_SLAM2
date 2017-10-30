#include "System.h"

#include <opencv2/highgui/highgui.hpp>

#include <chrono>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    enum class status : int
    {
        success = 0,
        failure = -1
    };

    int ret_val{static_cast<int>(status::success)};

    if (4 == argc)
    {
        std::string video_file{argv[1]};
        std::string vocabulary_file{argv[2]};
        std::string settings_file{argv[3]};

        cv::VideoCapture video_capture(video_file);

        if (true == video_capture.isOpened())
        {
            ORB_SLAM2::System slam(vocabulary_file, settings_file,
                                   ORB_SLAM2::System::MONOCULAR);
            cv::Mat frame;
            while (video_capture.read(frame))
            {
                auto current_video_time = video_capture.get(CV_CAP_PROP_POS_MSEC);
                slam.TrackMonocular(frame, current_video_time);
                //std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }

            slam.Shutdown();
            slam.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        }
        else
        {
            std::cerr << "Cannot open the video file " << video_file << '\n';
            ret_val = static_cast<int>(status::failure);
        }
    }
    else
    {
        std::cerr << "Invalid input\n";
        std::cerr
            << "Usage: mono_video VideoFile VocabularyFile SettingsFile\n";
        std::cerr << "Example: /ORB_SLAM2/Examples/Monocular/mono_video "
                  << "/ORB_SLAM2/Examples/video.mp4 "
                  << "/ORB_SLAM2/Vocabulary/ORBVoc.txt "
                  << "/ORB_SLAM2/Examples/Monocular/TUM1.yaml\n";
        ret_val = static_cast<int>(status::failure);
    }

    return ret_val;
}
