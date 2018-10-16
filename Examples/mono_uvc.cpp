/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <opencv/highgui.h>
#include <libuvc/libuvc.h>
#include <unistd.h>
#include <signal.h>

#include<opencv2/core/core.hpp>

#include"System.h"
static bool is_exit=false;
using namespace std;

void uvc_cb(uvc_frame_t *frame, void *ptr);

void SIGINT_handler(int dummy)
{
    is_exit = true;
    puts("exiting...");
}
void register_handler()
{
    // register SIGINT handler
    struct sigaction sigint_hdl;
    sigint_hdl.sa_handler = SIGINT_handler;
    sigemptyset(&sigint_hdl.sa_mask);
    sigint_hdl.sa_flags = 0;
    sigaction(SIGINT, &sigint_hdl, NULL);
}
int main(int argc, char **argv)
{
    register_handler();

    uvc_context_t *ctx;
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;
    uvc_error_t ret;

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        return 1;
    }
    // libuvc setup
    ret = uvc_init(&ctx, NULL);
    if (ret != UVC_SUCCESS) {
        uvc_perror(ret, "uvc_init");
        return ret;
    }
    puts("UVC initialized");
    ret = uvc_find_device(ctx, &dev, 0, 0, NULL);
    if (ret != UVC_SUCCESS) {
        uvc_perror(ret, "uvc_find_device");
        return ret;
    }
    puts("UVC Device found");
    ret = uvc_open(dev, &devh);
    if (ret != UVC_SUCCESS) {
        uvc_perror(ret, "uvc_open");
        return ret;
    }
    puts("UVC Device Opened");
    uvc_print_diag(devh, stderr);
    puts("###############");
    ret = uvc_get_stream_ctrl_format_size(devh, &ctrl, UVC_FRAME_FORMAT_YUYV, 640, 480, 30);
    uvc_print_stream_ctrl(&ctrl, stderr);
    puts("##############");
    if (ret != UVC_SUCCESS) {
        uvc_perror(ret, "get_mode");
        return ret;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ret = uvc_start_streaming(devh, &ctrl, uvc_cb, &SLAM, 0);
    if (ret != UVC_SUCCESS) {
        uvc_perror(ret, "start_streaming");
        return ret;
    }
    puts("Streaming");
    uvc_set_ae_mode(devh, 2);
    while (!is_exit) sleep(1);
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    uvc_stop_streaming(devh);
    puts("Done Streaming");
    uvc_close(devh);
    puts("Device Closed");

    return 0;
}
inline double timeval2double(struct timeval t)
{
    return t.tv_sec + double(t.tv_usec)/1000000;
}
void uvc_cb(uvc_frame_t *frame, void *ptr)
{
    ORB_SLAM2::System *mpSLAM = (ORB_SLAM2::System *)ptr;
    uvc_frame_t *bgr;
    uvc_error_t ret;
    bgr = uvc_allocate_frame(frame->width * frame->height*3);
    if (!bgr) {
        puts("Unable to allocate bgr frame!");
        return;
    }
    ret = uvc_any2bgr(frame, bgr);
    if (ret!=UVC_SUCCESS) {
        uvc_perror(ret, "uvc_any2bgr");
        uvc_free_frame(bgr);
        return;
    }
    cv::Mat img(bgr->height, bgr->width, CV_8UC(3), bgr->data);
    mpSLAM->TrackMonocular(img,timeval2double(frame->capture_time));
}

