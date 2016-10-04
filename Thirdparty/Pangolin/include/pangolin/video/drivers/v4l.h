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

#ifndef PANGOLIN_V4L_H
#define PANGOLIN_V4L_H

#include <pangolin/pangolin.h>
#include <pangolin/video/video.h>

#include <asm/types.h>
#include <linux/videodev2.h>

namespace pangolin
{

typedef enum {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
} io_method;

struct buffer {
    void*  start;
    size_t length;
};

class PANGOLIN_EXPORT V4lVideo : public VideoInterface, public VideoUvcInterface
{
public:
    V4lVideo(const char* dev_name, io_method io = IO_METHOD_MMAP, unsigned iwidth=0, unsigned iheight=0);
    ~V4lVideo();
    
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

    //! Implement VideoUvcInterface::IoCtrl()
    int IoCtrl(uint8_t unit, uint8_t ctrl, unsigned char* data, int len, UvcRequestCode req_code);

    int GetFileDescriptor() const{
        return fd;
    }
    
protected:
    int ReadFrame(unsigned char* image);
    void Mainloop();
    
    void init_read(unsigned int buffer_size);
    void init_mmap(const char* dev_name);
    void init_userp(const char* dev_name, unsigned int buffer_size);
    
    void init_device(const char* dev_name, unsigned iwidth, unsigned iheight, unsigned ifps, unsigned v4l_format = V4L2_PIX_FMT_YUYV, v4l2_field field = V4L2_FIELD_INTERLACED);
    void uninit_device();
    
    void open_device(const char* dev_name);
    void close_device();
    
    std::vector<StreamInfo> streams;
    
    io_method io;
    int       fd;
    buffer*   buffers;
    unsigned  int n_buffers;
    bool running;
    unsigned width;
    unsigned height;
    float fps;
    size_t image_size;
};

}

#endif // PANGOLIN_V4L_H
