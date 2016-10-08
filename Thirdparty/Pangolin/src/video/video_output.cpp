/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011-2013 Steven Lovegrove
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

#include <pangolin/video/video_output.h>

#include <pangolin/video/drivers/pango_video_output.h>

#ifdef HAVE_FFMPEG
#include <pangolin/video/drivers/ffmpeg.h>
#endif

#include <pangolin/utils/file_utils.h>

namespace pangolin
{

VideoOutputInterface* OpenVideoOutput(const Uri& uri)
{
    VideoOutputInterface* recorder = 0;
    
    if(!uri.scheme.compare("pango"))
    {
        const size_t mb = 1024*1024;
        const size_t buffer_size_bytes = uri.Get("buffer_size_mb", 100) * mb;
        const std::string filename = uri.url;
        recorder = new PangoVideoOutput(filename, buffer_size_bytes);
    }else
#ifdef HAVE_FFMPEG    
    if(!uri.scheme.compare("ffmpeg") )
    {
        int desired_frame_rate = uri.Get("fps", 60);
        int desired_bit_rate = uri.Get("bps", 20000*1024);
        std::string filename = uri.url;

        if(uri.Contains("unique_filename")) {        
            filename = MakeUniqueFilename(filename);
        }
        
        recorder = new FfmpegVideoOutput(filename, desired_frame_rate, desired_bit_rate);
    }else
#endif
    {
        throw VideoException("Unable to open recorder URI");
    }
    
    return recorder;
}

VideoOutputInterface* OpenVideoOutput(std::string str_uri)
{
    Uri uri = ParseUri(str_uri);
    return OpenVideoOutput(uri);
}

VideoOutput::VideoOutput()
    : recorder(NULL)
{
}

VideoOutput::VideoOutput(const std::string& uri)
    : recorder(NULL)
{
    Open(uri);
}

VideoOutput::~VideoOutput()
{
    delete recorder;
}

bool VideoOutput::IsOpen() const
{
    return recorder != 0;
}

void VideoOutput::Open(const std::string& str_uri)
{
    Close();
    uri = ParseUri(str_uri);
    recorder = OpenVideoOutput(uri);
}

void VideoOutput::Close()
{
    if(recorder) {
        delete recorder;
        recorder = 0;
    }    
}

const std::vector<StreamInfo>& VideoOutput::Streams() const
{
    return recorder->Streams();
}

void VideoOutput::SetStreams(const std::vector<StreamInfo>& streams, const std::string& uri, const json::value &properties)
{
    recorder->SetStreams(streams, uri, properties);
}

int VideoOutput::WriteStreams(unsigned char* data, const json::value& frame_properties)
{
    return recorder->WriteStreams(data, frame_properties);
}

bool VideoOutput::IsPipe() const
{
    return recorder->IsPipe();
}

}
