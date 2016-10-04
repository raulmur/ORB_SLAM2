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

#ifndef PANGOLIN_PANGO_VIDEO_OUTPUT_H
#define PANGOLIN_PANGO_VIDEO_OUTPUT_H

#include <pangolin/video/video_output.h>
#include <pangolin/log/packetstream.h>

namespace pangolin
{

class PANGOLIN_EXPORT PangoVideoOutput : public VideoOutputInterface
{
public:
    PangoVideoOutput(const std::string& filename);
    ~PangoVideoOutput();

    const std::vector<StreamInfo>& Streams() const PANGOLIN_OVERRIDE;
    void SetStreams(const std::vector<StreamInfo>& streams, const std::string& uri, const json::value& device_properties) PANGOLIN_OVERRIDE;
    int WriteStreams(unsigned char* data, const json::value& frame_properties) PANGOLIN_OVERRIDE;

protected:
    void WriteHeader();

    std::vector<StreamInfo> streams;
    std::string input_uri;
    json::value device_properties;

    PacketStreamWriter packetstream;
    int packetstreamsrcid;
    size_t total_frame_size;
};

}
#endif // PANGOLIN_PANGO_VIDEO_OUTPUT_H
