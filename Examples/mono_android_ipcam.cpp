/*
 * This an example base on ORB_SLAM2
 * It supports video input from a webcam or mjpeg stream over HTTP
 * libcurl is needed
 * Usage:
 * ./mono_android_ipcam /path/to/ORBvoc.{bin,txt} /path/to/settings.yaml http://xxx.xxx.xxx.xxx/video UpdateMap(0|1)
 */
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
#include <execinfo.h>
#include"System.h"

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

inline double timeval2double(struct timeval t)
{
    return t.tv_sec + double(t.tv_usec)/1000000;
}
static bool is_exit=false;
static void SIGINT_handler(int dummy)
{
    is_exit = true;
    puts("exiting...");
}
static void SIGSEGV_handler(int sig)
{
    void *array[100];
    size_t size;
    size = backtrace(array, 100);
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(-1);
}

void register_handler()
{
    // register SIGINT handler
    struct sigaction sigint_hdl;
    sigint_hdl.sa_handler = SIGINT_handler;
    sigemptyset(&sigint_hdl.sa_mask);
    sigint_hdl.sa_flags = 0;
    sigaction(SIGINT, &sigint_hdl, NULL);
    // register SIGSEGV handler
    struct sigaction sigsegv_hdl;
    sigsegv_hdl.sa_handler = SIGSEGV_handler;
    sigemptyset(&sigsegv_hdl.sa_mask);
    sigsegv_hdl.sa_flags = 0;
    sigaction(SIGSEGV, &sigsegv_hdl, NULL);
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
            mpSLAM->TrackMonocular(img, timeval2double(timestamp));
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
int main(int argc, char **argv) {
    register_handler();
    if(argc != 5)
    {
        cerr << endl << "Usage: mono_http_stream path_to_vocabulary path_to_settings http://mjpeg/stream/address updateMap(1|0)?" << endl;
        return 1;
    }
    stat = IDLE;
    bool bUpdateMap = (int)atoi(argv[4]);
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true,bUpdateMap);
    CURL *curl;
    curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, argv[3]);
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &SLAM);
    int err;

    err = curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    fprintf(stdout, "Want to exit(y/n) ? ");
    char c;
    do {
        fscanf(stdin, "%c", &c);
    } while  (c == 'y');
    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    return 0;
}
