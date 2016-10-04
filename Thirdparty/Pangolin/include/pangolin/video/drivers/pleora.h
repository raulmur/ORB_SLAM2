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

#ifndef PANGOLIN_VIDEO_PLEORA_H
#define PANGOLIN_VIDEO_PLEORA_H

#include <pangolin/pangolin.h>
#include <pangolin/video/video.h>

#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvBuffer.h>

#include <PvSystem.h>

#include <stdlib.h>
#include <list>

namespace pangolin
{

typedef std::list<PvBuffer *> BufferList;

class PANGOLIN_EXPORT PleoraVideo : public VideoInterface
{
public:

    PleoraVideo(const char *model_name, const char *serial_num, size_t index, size_t bpp = 8, size_t binX = 1, size_t binY = 1, size_t buffer_count = 4,
                size_t desired_size_x = 0, size_t desired_size_y = 0, size_t desired_pos_x = 0, size_t desired_pos_y = 0);
    ~PleoraVideo();

    void Start();

    void Stop();

    size_t SizeBytes() const;

    const std::vector<StreamInfo>& Streams() const;

    bool GrabNext( unsigned char* image, bool wait = true );

    bool GrabNewest( unsigned char* image, bool wait = true );

protected:
    template<typename T>
    T DeviceParam(const char* name);

    template<typename T>
    T StreamParam(const char* name);

    std::vector<StreamInfo> streams;
    size_t size_bytes;

    // Pleora handles
    PvSystem* lPvSystem;
    PvDevice* lDevice;
    PvStream* lStream;

    // Genicam device parameters
    PvGenParameterArray* lDeviceParams;
    PvGenCommand* lStart;
    PvGenCommand* lStop;

    // Genicam stream parameters
    PvGenParameterArray* lStreamParams;

    BufferList lBufferList;
};

}

#endif // PANGOLIN_VIDEO_PLEORA_H
