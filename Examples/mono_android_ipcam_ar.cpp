#include <curl/curl.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string.h>
#include <stdio.h>
#include <stdexcept>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>
#include"System.h"

#include <chrono>
#include <vector>
#include "AR/ViewerAR.h"

//for fetching video stream with libcurl

#define MAX_JPEG_BUF_LEN 1000000
enum RECV_STATUS {IDLE, WAIT_LENGTH, RECV_DATA};
RECV_STATUS stat;
char signature[50];
size_t content_length;
size_t received_length=0;
char raw_jpeg_buf[MAX_JPEG_BUF_LEN];
char read_buf[2*CURL_MAX_WRITE_SIZE];
size_t read_buf_len;
bool read_buf_isempty = true;
size_t fps_count = 0;
double fps_lasttimestamp = 0;
//for AR
ORB_SLAM2::ViewerAR viewerAR;
bool bRGB=true;
cv::Mat K;
cv::Mat DistCoef;
using namespace std;

inline double timeval2double(struct timeval t)
{
    return t.tv_sec + double(t.tv_usec)/1000000;
}
static bool is_exit=false;
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
static size_t write_cb(char *buf, size_t n, size_t nmemb, void *p);
void PrintARHint();
int main(int argc, char **argv) {
    register_handler();
    if(argc != 4)
    {
	cerr << endl << "Usage: mono_http_stream path_to_vocabulary path_to_settings http://mjpeg/stream/address" << endl;
	return 1;
    }
    stat = IDLE;
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,false);
    // AR setup
    PrintARHint();
    viewerAR.SetSLAM(&SLAM);
    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);
    float fps = fSettings["Camera.fps"];
    viewerAR.SetFPS(fps);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    viewerAR.SetCameraCalibration(fx, fy, cx, cy);
    K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    DistCoef = cv::Mat::zeros(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
	DistCoef.resize(5);
	DistCoef.at<float>(4) = k3;
    }
    thread tARViewer = thread(&ORB_SLAM2::ViewerAR::Run, &viewerAR);
    // curl setup
    CURL *curl;
    curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, argv[3]);
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &SLAM);

    int err;
    fprintf(stdout, "Press Ctrl-C to exit\n");
    err = curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    if (is_exit) {
        fprintf(stdout, "Now Close the ARViewer Window\n");
        tARViewer.join();
    }
    else {
        viewerAR.Quit();
    }
    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    return 0;
}

void PrintARHint()
{
    cout << endl << endl;
    cout << "-----------------------" << endl;
    cout << "Augmented Reality Demo" << endl;
    cout << "1) Translate the camera to initialize SLAM." << endl;
    cout << "2) Look at a planar region and translate the camera." << endl;
    cout << "3) Press Insert Cube to place a virtual cube in the plane. " << endl;
    cout << endl;
    cout << "You can place several cubes in different planes." << endl;
    cout << "-----------------------" << endl;
    cout << endl;
}

static size_t write_cb(char *buf, size_t n, size_t nmemb, void *p)
{
    if (is_exit)
        return 0;
    size_t len = n*nmemb;
LOOP:
    switch (stat) {
    case IDLE:
    {
        if (!read_buf_isempty) {
            memcpy(read_buf + read_buf_len, buf, len);
            buf = read_buf;
            read_buf_isempty = true;
            len = read_buf_len + len;
            read_buf[len] = 0;
        }
        received_length = 0;
        char *sub = strstr(buf, "--");
        if (sub) {
            char *eol = strstr(sub, "\r");
            if (!eol) {
                goto WAIT_NEXT;
            }
            memcpy(signature, sub, eol-sub);
            signature[eol-sub] = 0;
            len -= eol - buf;
            buf = eol;
            stat = WAIT_LENGTH;
        }
        else goto WAIT_NEXT;
    }
    case WAIT_LENGTH:
    {
        if (!read_buf_isempty) {
            memcpy(read_buf + read_buf_len, buf, len);
            buf = read_buf;
            read_buf_isempty = true;
            len = read_buf_len + len;
            read_buf[len] = 0;
        }
        char *sub = strstr(buf, "Content-Length");
        if (sub) {
            char *space = strstr(sub, " ");
            if (!space) goto WAIT_NEXT;
            content_length = atoi(space);
            if (content_length > MAX_JPEG_BUF_LEN) {
                fprintf(stderr, "Buffer overflow!! %lu > %d\n", content_length, MAX_JPEG_BUF_LEN);
                exit(-1);
            }
            char *pos_n = strstr(space, "\n");
            if (!pos_n) goto WAIT_NEXT;
            pos_n = strstr(pos_n+1, "\n");
            if (!pos_n) goto WAIT_NEXT;
            stat = RECV_DATA;
            len -= pos_n+1 - buf;
            if (len > 0)
                buf = pos_n+1;
            else
                break;
        }
        else goto WAIT_NEXT;
    }
    case RECV_DATA:
        if (received_length + len < content_length) {
            memcpy(raw_jpeg_buf+received_length, buf, len);
            received_length += len;
            break;
        }
        else {
            memcpy(raw_jpeg_buf+received_length, buf, content_length - received_length);
            cv::Mat raw_data(1, content_length, CV_8UC1, raw_jpeg_buf);
            cv::Mat img = cv::imdecode(raw_data, CV_LOAD_IMAGE_COLOR);

            // deal with a frame of jpeg
            ORB_SLAM2::System *mpSLAM = (ORB_SLAM2::System *)p;
            struct timeval timestamp;
            gettimeofday(&timestamp, NULL);
            double timestampd = timeval2double(timestamp);
            cv::Mat Tcw = mpSLAM->TrackMonocular(img, timestampd);

            // fps
            ++fps_count;
            if (timestampd - fps_lasttimestamp > 1) {
                printf("\rFPS: %lf", fps_count/(timestampd-fps_lasttimestamp));
                fps_count = 0;
                fps_lasttimestamp = timestampd;
            }
            // for AR
            cv::Mat imu;
            int state = mpSLAM->GetTrackingState();
            vector<ORB_SLAM2::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
            vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();
            cv::undistort(img, imu, K, DistCoef);
            if (!bRGB) {
                viewerAR.SetImagePose(imu, Tcw, state, vKeys, vMPs);
            }
            else {
                cv::cvtColor(imu, imu, CV_RGB2BGR);
                viewerAR.SetImagePose(imu, Tcw, state, vKeys, vMPs);
            }

            // process next frame
            stat = IDLE;
            buf += content_length - received_length;
            len -= content_length - received_length;
            goto LOOP;
            break;
        }
    default:
        fprintf(stderr, "Unknown Status\n");
        exit(-1);
    }
    return n*nmemb;
WAIT_NEXT:
    memcpy(read_buf, buf, len);
    read_buf_isempty = false;
    read_buf_len = len;
    return n*nmemb;
}
