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

#ifndef PANGOLIN_VIDEO_OUTPUT_H
#define PANGOLIN_VIDEO_OUTPUT_H

// Pangolin video output supports various formats using
// different 3rd party libraries. (Only one right now)
//
// VideoOutput URI's take the following form:
//  scheme:[param1=value1,param2=value2,...]//device
//
// scheme = ffmpeg
//
// ffmpeg - encode to compressed file using ffmpeg
//  fps : fps to embed in encoded file.
//  bps : bits per second
//  unique_filename : append unique suffix if file already exists
//
//  e.g. ffmpeg://output_file.avi
//  e.g. ffmpeg:[fps=30,bps=1000000,unique_filename]//output_file.avi

#include <pangolin/video/video.h>

namespace pangolin
{

//! Interface to video recording destinations
struct PANGOLIN_EXPORT VideoOutputInterface
{
    virtual ~VideoOutputInterface() {}

    //! Get format and dimensions of all video streams
    virtual const std::vector<StreamInfo>& Streams() const = 0;

    virtual void SetStreams(const std::vector<StreamInfo>& streams, const std::string& uri ="", const json::value& properties = json::value() ) = 0;

    virtual int WriteStreams(unsigned char* data, const json::value& frame_properties ) = 0;

    virtual bool IsPipe() const = 0;
};

//! VideoOutput wrap to generically construct instances of VideoOutputInterface.
class PANGOLIN_EXPORT VideoOutput : public VideoOutputInterface
{
public:
    VideoOutput();
    VideoOutput(const std::string& uri);
    ~VideoOutput();
    
    bool IsOpen() const;
    void Open(const std::string& uri);
    void Close();

    const std::vector<StreamInfo>& Streams() const;

    void SetStreams(const std::vector<StreamInfo>& streams, const std::string& uri = "", const json::value& properties = json::value() );

    int WriteStreams(unsigned char* data, const json::value& frame_properties = json::value() );
    
    bool IsPipe() const;

protected:
    Uri uri;
    VideoOutputInterface* recorder;
};

//! Open VideoOutput Interface from string specification (as described in this files header)
PANGOLIN_EXPORT
VideoOutputInterface* OpenVideoOutput(std::string str_uri);

//! Open VideoOutput Interface from Uri specification
PANGOLIN_EXPORT
VideoOutputInterface* OpenVideoOutput(const Uri& uri);

}

#endif // PANGOLIN_VIDEO_OUTPUT_H
