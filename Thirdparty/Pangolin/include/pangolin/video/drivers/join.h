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

#ifndef PANGOLIN_VIDEO_JOIN_H
#define PANGOLIN_VIDEO_JOIN_H

#include <pangolin/video/video.h>

namespace pangolin
{

class PANGOLIN_EXPORT VideoJoiner
    : public VideoInterface, public VideoFilterInterface
{
public:
    VideoJoiner(const std::vector<VideoInterface *> &src);

    ~VideoJoiner();

    size_t SizeBytes() const;

    const std::vector<StreamInfo>& Streams() const;

    void Start();

    void Stop();

    bool GrabNext( unsigned char* image, bool wait = true );

    bool GrabNewest( unsigned char* image, bool wait = true );

    std::vector<VideoInterface*>& InputStreams();

protected:
    std::vector<VideoInterface*> src;
    std::vector<StreamInfo> streams;
    size_t size_bytes;
};


}

#endif // PANGOLIN_VIDEO_JOIN_H
