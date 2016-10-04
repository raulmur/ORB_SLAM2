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

#ifndef PANGOLIN_VIDEO_RECORD_REPEAT_H
#define PANGOLIN_VIDEO_RECORD_REPEAT_H

#include <pangolin/video/video.h>
#include <pangolin/video/video_output.h>

namespace pangolin
{

struct PANGOLIN_EXPORT VideoRecordRepeat
    : public VideoInterface, public VideoPropertiesInterface
{
    VideoRecordRepeat();
    VideoRecordRepeat(const std::string &input_uri, const std::string &output_uri = "video_log.pango", int buffer_size_bytes = 10240000);
    ~VideoRecordRepeat();

    void Open(const std::string &input_uri, const std::string &output_uri = "video_log.pango", int buffer_size_bytes = 10240000);
    void Close();

    /////////////////////////////////////////////////////////////
    // VideoInterface Methods
    /////////////////////////////////////////////////////////////

    size_t SizeBytes() const;
    const std::vector<StreamInfo>& Streams() const;

    // Return details of first stream
    unsigned int Width() const {
        return (unsigned int)Streams()[0].Width();
    }
    unsigned int Height() const {
        return (unsigned int)Streams()[0].Height();
    }
    VideoPixelFormat PixFormat() const {
        return Streams()[0].PixFormat();
    }
    const Uri& VideoUri() const {
        return uri_input;
    }

    void Start();
    void Stop();
    bool GrabNext( unsigned char* image, bool wait = true );
    bool GrabNewest( unsigned char* image, bool wait = true );

    const json::value& DeviceProperties() const;
    const json::value& FrameProperties() const;

    /////////////////////////////////////////////////////////////
    // VideoInput Methods
    /////////////////////////////////////////////////////////////

    // experimental - not stable
    bool Grab( unsigned char* buffer, std::vector<Image<unsigned char> >& images, bool wait = true, bool newest = false);
    
    /////////////////////////////////////////////////////////////
    // VideoRecordRepeat Methods
    /////////////////////////////////////////////////////////////

    // Return pointer to inner video class as VideoType
    template<typename VideoType>
    VideoType* Cast() {
        return dynamic_cast<VideoType*>(video_src);
    }

    const std::string& LogFilename() const;

    void Record();
    void Play(bool realtime = true);
    void Source();
    
    
    bool IsRecording() const;
    bool IsPlaying() const;
    
    int FrameId();

protected:
    json::value null_props;
    std::string str_uri_input;
    Uri uri_input;
    Uri uri_output;

    VideoInterface* video_src;
    VideoPropertiesInterface* video_src_props;

    VideoInterface* video_file;
    VideoPropertiesInterface* video_file_props;
    VideoOutputInterface* video_recorder;
    
    int buffer_size_bytes;
    
    int frame_num;
};

}

#endif // PANGOLIN_VIDEO_RECORD_REPEAT_H
