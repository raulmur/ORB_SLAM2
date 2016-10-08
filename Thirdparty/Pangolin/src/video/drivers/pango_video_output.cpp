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

#include <pangolin/video/drivers/pango_video_output.h>
#include <pangolin/utils/picojson.h>
#include <pangolin/utils/file_utils.h>
#include <pangolin/utils/sigstate.h>
#include <set>

#ifndef _WIN_
#  include <unistd.h>
#endif

namespace pangolin
{

const std::string pango_video_type = "raw_video";

void SigPipeHandler(int sig)
{
    SigState::I().sig_callbacks.at(sig).value = true;
}

PangoVideoOutput::PangoVideoOutput(const std::string& filename, size_t buffer_size_bytes)
    : filename(filename),
      packetstream_buffer_size_bytes(buffer_size_bytes),
      packetstreamsrcid(-1),
      first_frame(true),
      is_pipe(pangolin::IsPipe(filename))
{
    if(!is_pipe)
    {
        packetstream.Open(filename, packetstream_buffer_size_bytes);
    }
    else
    {
        RegisterNewSigCallback(&SigPipeHandler, (void*)this, SIGPIPE);
    }
}

PangoVideoOutput::~PangoVideoOutput()
{
}

const std::vector<StreamInfo>& PangoVideoOutput::Streams() const
{
    return streams;
}

bool PangoVideoOutput::IsPipe() const
{
    return is_pipe;
}

void PangoVideoOutput::SetStreams(const std::vector<StreamInfo>& st, const std::string& uri, const json::value& properties)
{
    std::set<unsigned char*> unique_ptrs;
    for(size_t i=0; i<st.size(); ++i) {
        unique_ptrs.insert(st[i].Offset());
    }

    if(unique_ptrs.size() < st.size()) {
        throw std::invalid_argument("Each image must have unique offset into buffer.");
    }

    if(packetstreamsrcid == -1) {
        input_uri = uri;
        streams = st;
        device_properties = properties;

        json::value json_header(json::object_type,false);
        json::value& json_streams = json_header["streams"];
        json_header["device"] = device_properties;

        total_frame_size = 0;
        for(unsigned int i=0; i< streams.size(); ++i) {
            StreamInfo& si = streams[i];
            total_frame_size = std::max( total_frame_size, (size_t)si.Offset() + si.SizeBytes());

            json::value& json_stream = json_streams.push_back();
            json_stream["encoding"] = si.PixFormat().format;
            json_stream["width"] =    si.Width();
            json_stream["height"] =   si.Height();
            json_stream["pitch"] =    si.Pitch();
            json_stream["offset"] =   (size_t)si.Offset();
        }

        PacketStreamSource pss = packetstream.CreateSource(
                pango_video_type, input_uri, json_header,
                total_frame_size,
                "struct Frame{"
                " uint8 stream_data[" + pangolin::Convert<std::string,size_t>::Do(total_frame_size) + "];"
                "};"
            );

        packetstreamsrcid = (int)pss.id;

        packetstream.AddSource(pss);
    }else{
        throw std::runtime_error("Unable to add new streams");
    }
}

void PangoVideoOutput::WriteHeader()
{
    packetstream.WriteSources();
}

int PangoVideoOutput::WriteStreams(unsigned char* data, const json::value& frame_properties)
{
#ifndef _WIN_
    if(is_pipe)
    {
        // If there is a reader waiting on the other side of the pipe, open
        // a file descriptor to the file and close it only after the file
        // has been opened by the PacketStreamWriter. This avoids the reader
        // from seeing EOF on its next read because all file descriptors on
        // the write side have been closed.
        //
        // When the stream is already open but the reader has disappeared,
        // opening a file descriptor will fail and errno will be ENXIO.
        int fd = WritablePipeFileDescriptor(filename);

        if (!packetstream.IsOpen()) {
            if (fd != -1) {
                packetstream.Open(filename, packetstream_buffer_size_bytes);
                close(fd);
                first_frame = true;
            }
        } else {
            if (fd != -1) {
                // There's a reader on the other side of the pipe.
                close(fd);
            } else {
                if (errno == ENXIO) {
                    packetstream.ForceClose();
                    SigState::I().sig_callbacks.at(SIGPIPE).value = false;

                    // This should be unnecessary since per the man page,
                    // data should be dropped from the buffer upon closing the
                    // writable file descriptors.
                    pangolin::FlushPipe(filename);
                }
            }
        }

        if(!packetstream.IsOpen())
        {
            return 0;
        }
    }
#endif

    if(first_frame)
    {
        first_frame = false;
        WriteHeader();
    }

    if(!frame_properties.is<json::null>()) {
        packetstream.WriteSourcePacketMeta(packetstreamsrcid, frame_properties);
    }

    packetstream.WriteSourcePacket(
        packetstreamsrcid,
        (char*)data, total_frame_size
    );

    return 0;
}

}
