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

#ifndef PANGOLIN_VIDEO_IMAGES_H
#define PANGOLIN_VIDEO_IMAGES_H

#include <pangolin/pangolin.h>
#include <pangolin/video/video.h>
#include <pangolin/image/image_io.h>

#include <deque>
#include <vector>

namespace pangolin
{

// Video class that outputs test video signal.
class PANGOLIN_EXPORT ImagesVideo : public VideoInterface, public VideoPlaybackInterface
{
public:
    ImagesVideo(const std::string& wildcard_path);
    ImagesVideo(const std::string& wildcard_path, const VideoPixelFormat& raw_fmt, size_t raw_width, size_t raw_height);

    ~ImagesVideo();

    // Implement VideoInterface
    
    //! Implement VideoInput::Start()
    void Start() PANGOLIN_OVERRIDE;
    
    //! Implement VideoInput::Stop()
    void Stop() PANGOLIN_OVERRIDE;

    //! Implement VideoInput::SizeBytes()
    size_t SizeBytes() const PANGOLIN_OVERRIDE;

    //! Implement VideoInput::Streams()
    const std::vector<StreamInfo>& Streams() const PANGOLIN_OVERRIDE;
    
    //! Implement VideoInput::GrabNext()
    bool GrabNext( unsigned char* image, bool wait = true ) PANGOLIN_OVERRIDE;
    
    //! Implement VideoInput::GrabNewest()
    bool GrabNewest( unsigned char* image, bool wait = true ) PANGOLIN_OVERRIDE;

    // Implement VideoPlaybackInterface

    int GetCurrentFrameId() const PANGOLIN_OVERRIDE;

    int GetTotalFrames() const PANGOLIN_OVERRIDE;

    int Seek(int frameid) PANGOLIN_OVERRIDE;
    
protected:
    typedef std::vector<TypedImage> Frame;
    
    const std::string& Filename(size_t frameNum, size_t channelNum) {
        return filenames[channelNum][frameNum];
    }
    
    void PopulateFilenames(const std::string& wildcard_path);

    bool LoadFrame(size_t i);

    void ConfigureStreamSizes();
    
    std::vector<StreamInfo> streams;
    size_t size_bytes;
    
    int num_files;
    size_t num_channels;
    size_t next_frame_id;
    std::vector<std::vector<std::string> > filenames;
    std::vector<Frame> loaded;

    bool unknowns_are_raw;
    VideoPixelFormat raw_fmt;
    size_t raw_width;
    size_t raw_height;
};

}

#endif // PANGOLIN_VIDEO_IMAGES_H
