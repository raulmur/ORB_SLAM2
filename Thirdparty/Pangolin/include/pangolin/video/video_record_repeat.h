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
    : public VideoInterface,
      public VideoFilterInterface
{
    /////////////////////////////////////////////////////////////
    // VideoInterface Methods
    /////////////////////////////////////////////////////////////

    size_t SizeBytes() const PANGOLIN_OVERRIDE;
    const std::vector<StreamInfo>& Streams() const PANGOLIN_OVERRIDE;
    void Start() PANGOLIN_OVERRIDE;
    void Stop() PANGOLIN_OVERRIDE;
    bool GrabNext( unsigned char* image, bool wait = true ) PANGOLIN_OVERRIDE;
    bool GrabNewest( unsigned char* image, bool wait = true ) PANGOLIN_OVERRIDE;

    /////////////////////////////////////////////////////////////
    // VideoFilterInterface Methods
    /////////////////////////////////////////////////////////////

    std::vector<VideoInterface*>& InputStreams() PANGOLIN_OVERRIDE
    {
        return videos;
    }

    /////////////////////////////////////////////////////////////
    // VideoRecordRepeat Methods
    /////////////////////////////////////////////////////////////

    VideoRecordRepeat();
    VideoRecordRepeat(const std::string &input_uri, const std::string &output_uri = "pango:[buffer_size_mb=100]//video_log.pango");
    ~VideoRecordRepeat();

    void Open(const std::string &input_uri, const std::string &output_uri = "pango:[buffer_size_mb=100]//video_log.pango");
    void Close();

    // experimental - not stable
    bool Grab( unsigned char* buffer, std::vector<Image<unsigned char> >& images, bool wait = true, bool newest = false);

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

    // Return pointer to inner video class as VideoType
    template<typename VideoType>
    VideoType* Cast() {
        return dynamic_cast<VideoType*>(video_src);
    }

    const std::string& LogFilename() const;

    // Switch to live video source
    void Source();

    // Switch to previously recorded input
    void Play(bool realtime = true);

    // Switch to live video and record output to file
    void Record();

    // Switch to live video and record a single frame
    void RecordOneFrame();
    
    // Specify that one in n frames are logged to file. Default is 1.
    void SetTimelapse(size_t one_in_n_frames);

    // True iff grabbed live frames are being logged to file
    bool IsRecording() const;

    // True iff grabbed frames are from previously recorded video file
    bool IsPlaying() const;
    
    int FrameId();


protected:
    void InitialiseRecorder();

    mutable json::value frame_properties;
    mutable json::value device_properties;
    std::string str_uri_input;
    Uri uri_input;
    Uri uri_output;

    VideoInterface* video_src;
    VideoInterface* video_file;
    VideoOutputInterface* video_recorder;

    // Use to store either video_src or video_file for VideoFilterInterface,
    // depending on which is active
    std::vector<VideoInterface*> videos;
    
    int buffer_size_bytes;
    
    int frame_num;
    size_t record_frame_skip;

    bool record_once;
    bool record_continuous;
};

}

#endif // PANGOLIN_VIDEO_RECORD_REPEAT_H
