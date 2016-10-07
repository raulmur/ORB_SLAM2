/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2014 Steven Lovegrove
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

#ifndef PANGOLIN_VIDEO_DEBAYER_H
#define PANGOLIN_VIDEO_DEBAYER_H

#include <pangolin/pangolin.h>
#include <pangolin/video/video.h>

namespace pangolin
{

// Enum to match libdc1394's dc1394_bayer_method_t
typedef enum {
    BAYER_METHOD_NEAREST = 0,
    BAYER_METHOD_SIMPLE,
    BAYER_METHOD_BILINEAR,
    BAYER_METHOD_HQLINEAR,
    BAYER_METHOD_DOWNSAMPLE_,
    BAYER_METHOD_EDGESENSE,
    BAYER_METHOD_VNG,
    BAYER_METHOD_AHD,

    // Pangolin custom defines
    BAYER_METHOD_NONE = 512,
    BAYER_METHOD_DOWNSAMPLE,
    BAYER_METHOD_DOWNSAMPLE_MONO
} bayer_method_t;

// Enum to match libdc1394's dc1394_color_filter_t
typedef enum {
    DC1394_COLOR_FILTER_RGGB = 512,
    DC1394_COLOR_FILTER_GBRG,
    DC1394_COLOR_FILTER_GRBG,
    DC1394_COLOR_FILTER_BGGR
} color_filter_t;

// Video class that debayers its video input using the given method.
class PANGOLIN_EXPORT DebayerVideo :
        public VideoInterface,
        public VideoFilterInterface,
        public BufferAwareVideoInterface
{
public:
    DebayerVideo(VideoInterface* videoin, const std::vector<bayer_method_t> &method, color_filter_t tile);
    ~DebayerVideo();

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

    std::vector<VideoInterface*>& InputStreams();

    static color_filter_t ColorFilterFromString(std::string str);

    static bayer_method_t BayerMethodFromString(std::string str);

    uint32_t AvailableFrames() const;

    bool DropNFrames(uint32_t n);

protected:
    void ProcessStreams(unsigned char* out, const unsigned char* in);

    std::vector<VideoInterface*> videoin;
    std::vector<StreamInfo> streams;

    size_t size_bytes;
    unsigned char* buffer;

    std::vector<bayer_method_t> methods;
    color_filter_t tile;

    json::value device_properties;
    json::value frame_properties;
};

}

#endif // PANGOLIN_VIDEO_DEBAYER_H
