/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef PANGOLIN_UVC_H
#define PANGOLIN_UVC_H

#include <pangolin/pangolin.h>
#include <pangolin/video/video.h>

#ifdef _MSC_VER
// Define missing timeval struct
typedef struct timeval {
    long tv_sec;
    long tv_usec;
} timeval;
#endif // _MSC_VER

#include <libuvc/libuvc.h>

namespace pangolin
{

class PANGOLIN_EXPORT UvcVideo : public VideoInterface, public VideoUvcInterface
{
public:
    UvcVideo(int vendor_id, int product_id, const char* sn, int deviceid, int width, int height, int fps);
    ~UvcVideo();
    
    void InitDevice(int vid, int pid, const char* sn, int deviceid, int width, int height, int fps);
    void DeinitDevice();
    
    //! Implement VideoInput::Start()
    void Start();
    
    //! Implement VideoInput::Stop()
    void Stop();

    //! Implement VideoInput::SizeBytes()
    size_t SizeBytes() const;

    //! Implement VideoInput::Streams()
    const std::vector<StreamInfo>& Streams() const;
    
    //! Implement VideoInput::GrabNext()
    bool GrabNext( unsigned char* image, bool wait = true );
    
    //! Implement VideoInput::GrabNewest()
    bool GrabNewest( unsigned char* image, bool wait = true );

    //! Implement VideoUvcInterface::GetCtrl()
    int IoCtrl(uint8_t unit, uint8_t ctrl, unsigned char* data, int len, UvcRequestCode req_code);

protected:
    static uvc_error_t FindDevice(
        uvc_context_t *ctx, uvc_device_t **dev,
        int vid, int pid, const char *sn, int device_id);

    std::vector<StreamInfo> streams;
    size_t size_bytes;
    
    uvc_context* ctx_;
    uvc_device*  dev_;
    uvc_device_handle* devh_;
    uvc_stream_handle* strm_;
    uvc_stream_ctrl_t ctrl_;
    uvc_frame_t* frame_;
};

}

#endif // PANGOLIN_UVC_H
