/* This file is part of the Pangolin Project,
 * http://github.com/stevenlovegrove/Pangolin
 * Copyright (c) 2011 Steven Lovegrove
 *
 * adapted from V4L2 video capture example
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

#include <pangolin/video/drivers/v4l.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))

using namespace std;

namespace pangolin
{

static int xioctl(int fd, int request, void* arg)
{
    int r;
    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}

inline std::string V4lToString(int32_t v)
{
    //	v = ((__u32)(a) | ((__u32)(b) << 8) | ((__u32)(c) << 16) | ((__u32)(d) << 24))
    char cc[5];
    cc[0] = v       & 0xff;
    cc[1] = (v>>8)  & 0xff;
    cc[2] = (v>>16) & 0xff;
    cc[3] = (v>>24) & 0xff;
    cc[4] = 0;
    return std::string(cc);
}

V4lVideo::V4lVideo(const char* dev_name, io_method io, unsigned iwidth, unsigned iheight)
    : io(io), fd(-1), buffers(0), n_buffers(0), running(false)
{
    open_device(dev_name);
    init_device(dev_name,iwidth,iheight,0);
    Start();
}

V4lVideo::~V4lVideo()
{
    if(running)
    {
        Stop();
    }
    
    uninit_device();
    close_device();
}

const std::vector<StreamInfo>& V4lVideo::Streams() const
{
    return streams;
}

size_t V4lVideo::SizeBytes() const
{
    return image_size;
}

bool V4lVideo::GrabNext( unsigned char* image, bool wait )
{
    for (;;) {
        fd_set fds;
        struct timeval tv;
        int r;
        
        FD_ZERO (&fds);
        FD_SET (fd, &fds);
        
        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        
        r = select (fd + 1, &fds, NULL, NULL, &tv);
        
        if (-1 == r) {
            if (EINTR == errno)
                continue;
            
            throw VideoException ("select", strerror(errno));
        }
        
        if (0 == r) {
            throw VideoException("select Timeout", strerror(errno));
        }
        
        if (ReadFrame(image))
            break;
        
        /* EAGAIN - continue select loop. */
    }
    return true;
}

bool V4lVideo::GrabNewest( unsigned char* image, bool wait )
{
    // TODO: Implement
    return GrabNext(image,wait);
}

int V4lVideo::ReadFrame(unsigned char* image)
{
    struct v4l2_buffer buf;
    unsigned int i;
    
    switch (io) {
    case IO_METHOD_READ:
        if (-1 == read (fd, buffers[0].start, buffers[0].length)) {
            switch (errno) {
            case EAGAIN:
                return 0;
                
            case EIO:
                /* Could ignore EIO, see spec. */
                
                /* fall through */
                
            default:
                throw VideoException("read", strerror(errno));
            }
        }
        
        //            process_image(buffers[0].start);
        memcpy(image,buffers[0].start,buffers[0].length);
        
        break;
        
    case IO_METHOD_MMAP:
        CLEAR (buf);
        
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
            switch (errno) {
            case EAGAIN:
                return 0;
                
            case EIO:
                /* Could ignore EIO, see spec. */
                
                /* fall through */
                
            default:
                throw VideoException("VIDIOC_DQBUF", strerror(errno));
            }
        }
        
        assert (buf.index < n_buffers);
        
        //            process_image (buffers[buf.index].start);
        memcpy(image,buffers[buf.index].start,buffers[buf.index].length);
        
        
        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
            throw VideoException("VIDIOC_QBUF", strerror(errno));
        
        break;
        
    case IO_METHOD_USERPTR:
        CLEAR (buf);
        
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        
        if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
            switch (errno) {
            case EAGAIN:
                return 0;
                
            case EIO:
                /* Could ignore EIO, see spec. */
                
                /* fall through */
                
            default:
                throw VideoException("VIDIOC_DQBUF", strerror(errno));
            }
        }
        
        for (i = 0; i < n_buffers; ++i)
            if (buf.m.userptr == (unsigned long) buffers[i].start
                    && buf.length == buffers[i].length)
                break;
        
        assert (i < n_buffers);
        
        //            process_image ((void *) buf.m.userptr);
        memcpy(image,(void *)buf.m.userptr,buf.length);
        
        
        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
            throw VideoException("VIDIOC_QBUF", strerror(errno));
        
        break;
    }
    
    return 1;
}

void V4lVideo::Stop()
{
    if(running) {
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
            /* Nothing to do. */
            break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
                throw VideoException("VIDIOC_STREAMOFF", strerror(errno));

            break;
        }

        running = false;
    }
}

void V4lVideo::Start()
{
    if(!running) {
        unsigned int i;
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
            /* Nothing to do. */
            break;

        case IO_METHOD_MMAP:
            for (i = 0; i < n_buffers; ++i) {
                struct v4l2_buffer buf;

                CLEAR (buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = i;

                if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                    throw VideoException("VIDIOC_QBUF", strerror(errno));
            }

            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
                throw VideoException("VIDIOC_STREAMON", strerror(errno));

            break;

        case IO_METHOD_USERPTR:
            for (i = 0; i < n_buffers; ++i) {
                struct v4l2_buffer buf;

                CLEAR (buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_USERPTR;
                buf.index       = i;
                buf.m.userptr   = (unsigned long) buffers[i].start;
                buf.length      = buffers[i].length;

                if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                    throw VideoException("VIDIOC_QBUF", strerror(errno));
            }

            type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

            if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
                throw VideoException ("VIDIOC_STREAMON", strerror(errno));

            break;
        }

        running = true;
    }
}

void V4lVideo::uninit_device()
{
    unsigned int i;
    
    switch (io) {
    case IO_METHOD_READ:
        free (buffers[0].start);
        break;
        
    case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers; ++i)
            if (-1 == munmap (buffers[i].start, buffers[i].length))
                throw VideoException ("munmap");
        break;
        
    case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers; ++i)
            free (buffers[i].start);
        break;
    }
    
    free (buffers);
}

void V4lVideo::init_read(unsigned int buffer_size)
{
    buffers = (buffer*)calloc (1, sizeof (buffer));
    
    if (!buffers) {
        throw VideoException("Out of memory\n");
    }
    
    buffers[0].length = buffer_size;
    buffers[0].start = malloc (buffer_size);
    
    if (!buffers[0].start) {
        throw VideoException("Out of memory\n");
    }
}

void V4lVideo::init_mmap(const char* dev_name)
{
    struct v4l2_requestbuffers req;
    
    CLEAR (req);
    
    req.count               = 4;
    req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory              = V4L2_MEMORY_MMAP;
    
    if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            throw VideoException("does not support memory mapping", strerror(errno));
        } else {
            throw VideoException ("VIDIOC_REQBUFS", strerror(errno));
        }
    }
    
    if (req.count < 2) {
        throw VideoException("Insufficient buffer memory");
    }
    
    buffers = (buffer*)calloc(req.count, sizeof(buffer));
    
    if (!buffers) {
        throw VideoException( "Out of memory\n");
    }
    
    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;
        
        CLEAR (buf);
        
        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = n_buffers;
        
        if (-1 == xioctl (fd, VIDIOC_QUERYBUF, &buf))
            throw VideoException ("VIDIOC_QUERYBUF", strerror(errno));
        
        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start =
                mmap (NULL /* start anywhere */,
                      buf.length,
                      PROT_READ | PROT_WRITE /* required */,
                      MAP_SHARED /* recommended */,
                      fd, buf.m.offset);
        
        if (MAP_FAILED == buffers[n_buffers].start)
            throw VideoException ("mmap");
    }
}

void V4lVideo::init_userp(const char* dev_name, unsigned int buffer_size)
{
    struct v4l2_requestbuffers req;
    unsigned int page_size;
    
    page_size = getpagesize ();
    buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);
    
    CLEAR (req);
    
    req.count               = 4;
    req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory              = V4L2_MEMORY_USERPTR;
    
    if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            throw VideoException( "Does not support user pointer i/o", strerror(errno));
        } else {
            throw VideoException ("VIDIOC_REQBUFS", strerror(errno));
        }
    }
    
    buffers = (buffer*)calloc(4, sizeof(buffer));
    
    if (!buffers) {
        throw VideoException( "Out of memory\n");
    }
    
    for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
        buffers[n_buffers].length = buffer_size;
        buffers[n_buffers].start = memalign (/* boundary */ page_size,
                                             buffer_size);
        
        if (!buffers[n_buffers].start) {
            throw VideoException( "Out of memory\n");
        }
    }
}

void V4lVideo::init_device(const char* dev_name, unsigned iwidth, unsigned iheight, unsigned ifps, unsigned v4l_format, v4l2_field field)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    struct v4l2_streamparm strm;
    
    unsigned int min;
    
    if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            throw VideoException("Not a V4L2 device", strerror(errno));
        } else {
            throw VideoException ("VIDIOC_QUERYCAP", strerror(errno));
        }
    }
    
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        throw VideoException("Not a video capture device");
    }
    
    switch (io) {
    case IO_METHOD_READ:
        if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
            throw VideoException("Does not support read i/o");
        }
        
        break;
        
    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            throw VideoException("Does not support streaming i/o");
        }
        
        break;
    }
    
    
    /* Select video input, video standard and tune here. */
    
    CLEAR (cropcap);
    
    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    
    if (0 == xioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */
        
        if (-1 == xioctl (fd, VIDIOC_S_CROP, &crop)) {
            switch (errno) {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    } else {
        /* Errors ignored. */
    }    
    
    CLEAR (fmt);

    if(iwidth!=0 && iheight!=0) {
        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = iwidth;
        fmt.fmt.pix.height      = iheight;
        fmt.fmt.pix.pixelformat = v4l_format;
        fmt.fmt.pix.field       = field;
        
        if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt))
            throw VideoException("VIDIOC_S_FMT", strerror(errno));
    }else{
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
            throw VideoException("VIDIOC_G_FMT", strerror(errno));
    }
    
    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;
    
    /* Note VIDIOC_S_FMT may change width and height. */
    width = fmt.fmt.pix.width;
    height = fmt.fmt.pix.height;
    image_size = fmt.fmt.pix.sizeimage;
    
    if(ifps!=0)
    {
        CLEAR(strm);
        strm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        strm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
        strm.parm.capture.timeperframe.numerator = 1;
        strm.parm.capture.timeperframe.denominator = ifps;
        
        if (-1 == xioctl (fd, VIDIOC_S_PARM, &fmt))
            throw VideoException("VIDIOC_S_PARM", strerror(errno));
        
        fps = (float)strm.parm.capture.timeperframe.denominator / strm.parm.capture.timeperframe.numerator;
    }else{
        fps = 0;    
    }
    
    switch (io) {
    case IO_METHOD_READ:
        init_read (fmt.fmt.pix.sizeimage);
        break;
        
    case IO_METHOD_MMAP:
        init_mmap (dev_name );
        break;
        
    case IO_METHOD_USERPTR:
        init_userp (dev_name, fmt.fmt.pix.sizeimage);
        break;
    }
    
    std::string spix="GRAY8";
    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY) {
        spix="GRAY8";
    }else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
        spix="YUYV422";
    }else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_Y16) {
        spix="GRAY16LE";
    }else{
        // TODO: Add method to translate from V4L to FFMPEG type.
        std::cerr << "V4L Format " << V4lToString(fmt.fmt.pix.pixelformat)
                  << " not recognised. Defaulting to '" << spix << std::endl;
    }

    const VideoPixelFormat pfmt = VideoFormatFromString(spix);
    const StreamInfo stream_info(pfmt, width, height, (width*pfmt.bpp)/8, 0);

    streams.push_back(stream_info);
}

void V4lVideo::SetExposureUs(int exposure_us)
{
    struct v4l2_control control;
    control.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    // v4l specifies exposure in 100us units
    control.value = exposure_us / 100;

    if (-1 == xioctl (fd, VIDIOC_S_CTRL, &control))
        pango_print_warn("V4lVideo::SetExposureUs() ioctl error: %d\n", errno);

}

void V4lVideo::close_device()
{
    if (-1 == close (fd))
        throw VideoException("close");
    
    fd = -1;
}

void V4lVideo::open_device(const char* dev_name)
{
    struct stat st;
    
    if (-1 == stat (dev_name, &st)) {
        throw VideoException("Cannot stat device", strerror(errno));
    }
    
    if (!S_ISCHR (st.st_mode)) {
        throw VideoException("Not device");
    }
    
    fd = open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
    
    if (-1 == fd) {
        throw VideoException("Cannot open device");
    }
}

int V4lVideo::IoCtrl(uint8_t unit, uint8_t ctrl, unsigned char* data, int len, UvcRequestCode req_code)
{
    struct uvc_xu_control_query xu;
    xu.unit = unit;
    xu.selector = ctrl;
    xu.size = len;
    xu.data = data;
    xu.query = req_code;

    int ret = ioctl(fd, UVCIOC_CTRL_QUERY, &xu);
    if (ret == -1) {
        pango_print_warn("V4lVideo::IoCtrl() ioctl error: %d\n", errno);
        return ret;
    }
    return 0;
}

}

