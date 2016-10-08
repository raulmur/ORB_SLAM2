/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2013 Steven Lovegrove
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

#include <pangolin/video/drivers/video_splitter.h>

namespace pangolin
{

VideoSplitter::VideoSplitter(VideoInterface *src, const std::vector<StreamInfo>& streams)
    : streams(streams)
{
    videoin.push_back(src);

    // Warn if stream over-runs input stream
    for(unsigned int i=0; i < streams.size(); ++i) {
        if(src->SizeBytes() < (size_t)streams[i].Offset() + streams[i].SizeBytes() ) {
            pango_print_warn("VideoSplitter: stream extends past end of input.\n");
            break;
        }
    }
}

VideoSplitter::~VideoSplitter()
{
    delete videoin[0];
}

size_t VideoSplitter::SizeBytes() const
{
    return videoin[0]->SizeBytes();
}

const std::vector<StreamInfo>& VideoSplitter::Streams() const
{
    return streams;
}

void VideoSplitter::Start()
{
    videoin[0]->Start();
}

void VideoSplitter::Stop()
{
    videoin[0]->Stop();
}

bool VideoSplitter::GrabNext( unsigned char* image, bool wait )
{
    return videoin[0]->GrabNext(image, wait);
}

bool VideoSplitter::GrabNewest( unsigned char* image, bool wait )
{
    return videoin[0]->GrabNewest(image, wait);
}

std::vector<VideoInterface*>& VideoSplitter::InputStreams()
{
    return videoin;
}



}
