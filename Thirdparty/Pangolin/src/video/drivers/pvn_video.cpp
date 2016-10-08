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

#include <pangolin/video/drivers/pvn_video.h>
#include <pangolin/utils/file_utils.h>

#include <iostream>

using namespace std;

namespace pangolin
{

PvnVideo::PvnVideo(const std::string& filename, bool realtime )
    : frame_size_bytes(0), realtime(realtime), last_frame(TimeNow())
{
    file.open( PathExpand(filename).c_str(), ios::binary );
    
    if(!file.is_open() )
        throw VideoException("Cannot open file - does not exist or bad permissions.");
    
    ReadFileHeader();
}

PvnVideo::~PvnVideo()
{
}

void PvnVideo::ReadFileHeader()
{
    string sfmt;
    float framerate;
    unsigned w, h;
        
    file >> sfmt;
    file >> w;
    file >> h;
    file >> framerate;
    file.get();

    if(file.bad() || !(w >0 && h >0) )
        throw VideoException("Unable to read video header");
    
    const VideoPixelFormat fmt = VideoFormatFromString(sfmt);
    StreamInfo strm0( fmt, w, h, (w*fmt.bpp) / 8, 0);
    
    frame_size_bytes += strm0.Pitch() * strm0.Height();
    frame_interval = TimeFromSeconds( 1.0 / framerate);
    
    streams.push_back(strm0);
}


void PvnVideo::Start()
{
}

void PvnVideo::Stop()
{
}

size_t PvnVideo::SizeBytes() const
{
    return frame_size_bytes;
}

const std::vector<StreamInfo>& PvnVideo::Streams() const
{
    return streams;
}

bool PvnVideo::GrabNext( unsigned char* image, bool /*wait*/ )
{
    file.read((char*)image, frame_size_bytes);
    
    const basetime next_frame = TimeAdd(last_frame, frame_interval);
    
    if( realtime ) {
        WaitUntil(next_frame);
    }
    
    last_frame = TimeNow();
    return file.good();
}

bool PvnVideo::GrabNewest( unsigned char* image, bool wait )
{
    return GrabNext(image,wait);
}

}
