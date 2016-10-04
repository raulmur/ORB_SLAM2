/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2015 Steven Lovegrove
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

#ifndef PANGOLIN_VIDEO_TELI_H
#define PANGOLIN_VIDEO_TELI_H

#include <pangolin/pangolin.h>
#include <pangolin/video/video.h>

#include <TeliCamApi.h>

namespace pangolin
{

// Video class that outputs test video signal.
class PANGOLIN_EXPORT TeliVideo : public VideoInterface
{
public:
    TeliVideo();
    TeliVideo(const ImageRoi& roi);
    ~TeliVideo();
    
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

    inline Teli::CAM_HANDLE GetCameraHandle() {
        return cam;
    }

    inline Teli::CAM_STRM_HANDLE GetCameraStreamHandle() {
        return strm;
    }

protected:
    void Initialise(const ImageRoi& roi);

    std::vector<StreamInfo> streams;
    size_t size_bytes;

    Teli::CAM_HANDLE cam;
    Teli::CAM_STRM_HANDLE strm;
    HANDLE hStrmCmpEvt;
};

}

#endif // PANGOLIN_VIDEO_TELI_H
