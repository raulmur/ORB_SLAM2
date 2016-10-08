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

#include <pangolin/video/drivers/pango_video.h>
#include <pangolin/utils/file_utils.h>
#include <pangolin/compat/bind.h>

#ifndef _WIN_
#  include <unistd.h>
#endif

namespace pangolin
{

const std::string pango_video_type = "raw_video";

PangoVideo::PangoVideo(const std::string& filename, bool realtime)
    : reader(filename, realtime), filename(filename), realtime(realtime),
      is_pipe(pangolin::IsPipe(filename)),
      is_pipe_open(true),
      pipe_fd(-1)
{
    // N.B. is_pipe_open can default to true since the reader opens the file and
    // reads header information from it, which means the pipe must be open and
    // filled with data.
    src_id = FindSource();

    if(src_id == -1) {
        throw pangolin::VideoException("No appropriate video streams found in log.");
    }
}

PangoVideo::~PangoVideo()
{
#ifndef _WIN_
    if (pipe_fd != -1) {
        close(pipe_fd);
    }
#endif
}

size_t PangoVideo::SizeBytes() const
{
    return size_bytes;
}

const std::vector<StreamInfo>& PangoVideo::Streams() const
{
    return streams;
}

void PangoVideo::Start()
{

}

void PangoVideo::Stop()
{

}

bool PangoVideo::GrabNext( unsigned char* image, bool /*wait*/ )
{
#ifndef _WIN_
    if (is_pipe && !is_pipe_open) {
        if (pipe_fd == -1) {
            pipe_fd = ReadablePipeFileDescriptor(filename);
        }

        if (pipe_fd == -1) {
            return false;
        }

        // Test whether the pipe has data to be read. If so, open the
        // file stream and start reading. After this point, the file
        // descriptor is owned by the reader.
        if (PipeHasDataToRead(pipe_fd)) {
            reader.Open(filename, realtime);
            close(pipe_fd);
            is_pipe_open = true;
        } else {
            return false;
        }
    }
#endif

    try {
        if(reader.ReadToSourcePacketAndLock(src_id)) {
            // read this frames actual data
            reader.Read((char*)image, size_bytes);
            reader.ReleaseSourcePacketLock(src_id);
            return true;
        } else {
            if (is_pipe && !reader.stream().good()) {
                HandlePipeClosed();
            }
            return false;
        }
    } catch (std::exception& ex) {
        if (is_pipe) {
            HandlePipeClosed();
            return false;
        } else {
            throw ex;
        }
    }
}

bool PangoVideo::GrabNewest( unsigned char* image, bool wait )
{
    return GrabNext(image, wait);
}

const json::value& PangoVideo::DeviceProperties() const
{
    if(src_id >=0) {
        return reader.Sources()[src_id].info["device"];
    }else{
        throw std::runtime_error("Not initialised");
    }
}

const json::value& PangoVideo::FrameProperties() const
{
    if(src_id >=0) {
        return reader.Sources()[src_id].meta;
    }else{
        throw std::runtime_error("Not initialised");
    }
}

int PangoVideo::GetCurrentFrameId() const
{
    return (int)reader.GetPacketIndex(src_id);
}

int PangoVideo::GetTotalFrames() const
{
    return (int)reader.GetNumPackets(src_id);
}

int PangoVideo::Seek(int frameid)
{
    return (int)reader.Seek(src_id, std::max(0,frameid) );
}

int PangoVideo::FindSource()
{
    for(PacketStreamSourceId src_id=0; src_id < reader.Sources().size(); ++src_id)
    {
        const PacketStreamSource& src = reader.Sources()[src_id];

        try {
            if( !src.driver.compare(pango_video_type) ) {
                // Read sources header
                size_bytes = src.data_size_bytes;

                device_properties = src.info["device"];
                const json::value& json_streams = src.info["streams"];
                const size_t num_streams = json_streams.size();
                for(size_t i=0; i<num_streams; ++i) {
                    const json::value& json_stream = json_streams[i];
                    StreamInfo si(
                        VideoFormatFromString(
                            json_stream["encoding"].get<std::string>()
                        ),
                        json_stream["width"].get<int64_t>(),
                        json_stream["height"].get<int64_t>(),
                        json_stream["pitch"].get<int64_t>(),
                        (unsigned char*)0 + json_stream["offset"].get<int64_t>()
                    );

                    streams.push_back(si);
                }

                return (int)src_id;
            }
        }catch(...) {
            pango_print_info("Unable to parse PacketStream Source. File version incompatible.\n");
        }
    }

    return -1;
}

void PangoVideo::HandlePipeClosed()
{
    // The pipe was closed by the other end. The pipe will have to be
    // re-opened, but it is not desirable to block at this point.
    //
    // The next time a frame is grabbed, the pipe will be checked and if
    // it is open, the stream will be re-opened.
    reader.Close();
    is_pipe_open = false;
}

}
