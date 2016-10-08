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

#ifndef PANGOLIN_VIDEO_THREAD_H
#define PANGOLIN_VIDEO_THREAD_H

#include <pangolin/pangolin.h>
#include <pangolin/video/video.h>

#include <pangolin/utils/fix_size_buffer_queue.h>

namespace pangolin
{


// Video class that creates a thread that keeps pulling frames and processing from its children.
class PANGOLIN_EXPORT ThreadVideo :  public VideoInterface, public VideoPropertiesInterface,
        public BufferAwareVideoInterface, public VideoFilterInterface
{
public:
    ThreadVideo(VideoInterface* videoin, size_t num_buffers);
    ~ThreadVideo();

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

    const json::value& DeviceProperties() const;

    const json::value& FrameProperties() const;

    uint32_t AvailableFrames() const;

    bool DropNFrames(uint32_t n);

    void operator()();

    std::vector<VideoInterface*>& InputStreams();

protected:
    struct GrabResult
    {
        GrabResult(const size_t buffer_size)
            : return_status(false),
              buffer(new unsigned char[buffer_size])
        {
        }

        bool return_status;
        unsigned char* buffer;
        json::value frame_properties;
    };

    std::vector<VideoInterface*> videoin;

    bool quit_grab_thread;
    FixSizeBuffersQueue<GrabResult> queue;

    boostd::condition_variable cv;
    boostd::mutex cvMtx;
    boostd::thread grab_thread;

    mutable json::value device_properties;
    json::value frame_properties;
};

}

#endif // PANGOLIN_VIDEO_THREAD_H
