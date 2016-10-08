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

#include <pangolin/log/packetstream.h>
#include <pangolin/utils/timer.h>
#include <pangolin/compat/thread.h>
#include <pangolin/utils/file_utils.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <time.h>

namespace pangolin
{

const char* json_src_driver          = "driver";
const char* json_src_id              = "id";
const char* json_src_info            = "info";
const char* json_src_uri             = "uri";
const char* json_src_packet          = "packet";
const char* json_src_version         = "version";
const char* json_pkt_alignment_bytes = "alignment_bytes";
const char* json_pkt_definitions     = "definitions";
const char* json_pkt_size_bytes      = "size_bytes";

//////////////////////////////////////////////////////////////////////////
// Timer utils
//////////////////////////////////////////////////////////////////////////

int playback_devices = 0;
int64_t playback_start_time_us = 0;

int64_t PlaybackTime_us()
{
    return Time_us(TimeNow()) - playback_start_time_us;
}

void SetCurrentPlaybackTime_us(int64_t time_us)
{
    playback_start_time_us = Time_us(TimeNow()) - time_us;
}

void WaitUntilPlaybackTime_us(int64_t time_us)
{
#if defined(CPP11_NO_BOOST) || BOOST_VERSION >= 104500
    // Wait correct amount of time
    const int64_t time_diff_us = time_us - PlaybackTime_us();

    boostd::this_thread::sleep_for(
        boostd::chrono::duration_cast<boostd::chrono::milliseconds>(
            boostd::chrono::microseconds(time_diff_us)
        )
    );
#else
    // TODO: Find method instead of busy-wait
    while (time_us - PlaybackTime_us() > 0);
#endif

}

//////////////////////////////////////////////////////////////////////////
// PacketStreamWriter
//////////////////////////////////////////////////////////////////////////

PacketStreamWriter::PacketStreamWriter()
    : writer(&buffer), is_open(false), is_pipe(false), bytes_written(0)
{
}

PacketStreamWriter::PacketStreamWriter(const std::string& filename, size_t buffer_size_bytes )
    : buffer(pangolin::PathExpand(filename), buffer_size_bytes), writer(&buffer), is_open(true), is_pipe(pangolin::IsPipe(filename)), bytes_written(0)
{
    // Start of file magic
    writer.write(PANGO_MAGIC.c_str(), PANGO_MAGIC.size());

    WritePangoHeader();
}

void PacketStreamWriter::Open(const std::string& filename, size_t buffer_size_bytes)
{
    is_pipe = pangolin::IsPipe(filename);

    // Open file for writing
    buffer.open(pangolin::PathExpand(filename), buffer_size_bytes);
    
    // Start of file magic
    writer.write(PANGO_MAGIC.c_str(), PANGO_MAGIC.size());

    WritePangoHeader();

    is_open = true;
}

void PacketStreamWriter::Close()
{
    if(is_open)
    {
        buffer.close();
        is_open = false;
    }
}

void PacketStreamWriter::ForceClose()
{
    if(is_open)
    {
        buffer.force_close();
        is_open = false;
    }
}

bool PacketStreamWriter::IsOpen() const
{
    return is_open;
}

PacketStreamWriter::~PacketStreamWriter()
{
    if(is_open && !is_pipe)
    {
        WriteStats();
    }
}

PacketStreamSource PacketStreamWriter::CreateSource(
    const std::string& source_driver,
    const std::string& source_uri,
    const json::value& source_info,
    const size_t       packet_size_bytes,
    const std::string& packet_definitions
) {
    PacketStreamSource pss;

    pss.driver = source_driver;
    pss.id = sources.size();
    pss.uri = source_uri;
    pss.info = source_info;
    pss.data_size_bytes = packet_size_bytes;
    pss.data_definitions = packet_definitions;
    pss.version = 1;
    pss.data_alignment_bytes = 1;

    return pss;
}

void PacketStreamWriter::AddSource(const PacketStreamSource& pss)
{
    sources.push_back(pss);
}

void PacketStreamWriter::WriteSources()
{
    for(size_t i = 0; i < sources.size(); i++)
    {
        json::value json_src;
        json_src[json_src_driver] = sources[i].driver;
        json_src[json_src_id] = sources[i].id;
        json_src[json_src_uri] = sources[i].uri;
        json_src[json_src_info] = sources[i].info;
        json_src[json_src_version] = sources[i].version;

        json::value& json_packet = json_src[json_src_packet];
        json_packet[json_pkt_size_bytes] = sources[i].data_size_bytes;
        json_packet[json_pkt_definitions] = sources[i].data_definitions;
        json_packet[json_pkt_alignment_bytes] = sources[i].data_alignment_bytes;

        WriteTag(TAG_ADD_SOURCE);
        json_src.serialize(std::ostream_iterator<char>(writer), true);
    }
}

void PacketStreamWriter::WriteSourcePacketMeta(PacketStreamSourceId src, const json::value& json)
{
    WriteTag(TAG_SRC_JSON);
    WriteCompressedUnsignedInt(src);
    json.serialize(std::ostream_iterator<char>(writer), false);
}

void PacketStreamWriter::WriteSourcePacket(PacketStreamSourceId src, const char* data, size_t n)
{
    // build jump cache.
    const std::streampos source_packet_pos = writer.tellp();
    std::vector<std::streampos>& packet_seek = src_packet_positions[src];
    packet_seek.push_back(source_packet_pos);

    // Write SOURCE_PACKET tag and source id
    WriteTag(TAG_SRC_PACKET);
    WriteTimestamp();
    WriteCompressedUnsignedInt(src);
    // Write packet size if dynamic so it can be skipped over easily
    size_t packet_size = sources[src].data_size_bytes;
    if(packet_size == 0) {
        WriteCompressedUnsignedInt(n);
    }else if(packet_size != n) {
        throw std::runtime_error("Attempting to write packet of wrong size");
    }

    // Write data
    writer.write(data, n);
    if(writer.bad()) {
        throw std::runtime_error("Error writing data.");
    }
    bytes_written += n;
}

const std::string CurrentTimeStr() {
    time_t time_now = time(0);
    struct tm time_struct = *localtime(&time_now);
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %X", &time_struct);
    return buffer;
}

void PacketStreamWriter::WritePangoHeader()
{
    // Write Header
    json::value pango;
    pango["pangolin_version"] = PANGOLIN_VERSION_STRING;
    pango["time_us"] = PlaybackTime_us();
    pango["date_created"] = CurrentTimeStr();
    pango["endian"] = "little_endian";

    WriteTag(TAG_PANGO_HDR);
    pango.serialize(std::ostream_iterator<char>(writer), true);
}

void PacketStreamWriter::WriteStats()
{
    const std::streampos footer_pos = writer.tellp();

    WriteTag(TAG_PANGO_STATS);

    // Build seek index
    json::array index;
    for(size_t src=0; src < sources.size(); ++src) {
        std::vector<std::streampos>& packet_seek = src_packet_positions[src];
        json::array seek_positions;
        for(size_t f=0; f < packet_seek.size(); ++f) {
            seek_positions.push_back( json::value( packet_seek[f] ) );
        }
        index.push_back(seek_positions);
    }

    // Make JSON stats
    json::value stat;
    stat["num_sources"]   = sources.size();
    stat["bytes_written"] = bytes_written;
    stat["src_packet_index"] = index;

    // Write to stream
    stat.serialize(std::ostream_iterator<char>(writer));

    WriteFooter(footer_pos);
}

void PacketStreamWriter::WriteSync()
{
    for(int i=0; i<10; ++i) {
        WriteTag(TAG_PANGO_SYNC);
    }
}

void PacketStreamWriter::WriteFooter(const std::streampos footer_pos)
{
    WriteTag(TAG_PANGO_FOOTER);
    const uint64_t footer_pos_64 = footer_pos;
    writer.write((char*)&footer_pos_64, sizeof(uint64_t));
}

//////////////////////////////////////////////////////////////////////////
// PacketStreamReader
//////////////////////////////////////////////////////////////////////////

std::string TagName(int v)
{
    char b[4];
    b[0] = v&0xff;
    b[1] = (v>>8)&0xff;
    b[2] = (v>>16)&0xff;
    b[3] = 0x00;
    return std::string(b);
}

PacketStreamReader::PacketStreamReader()
    : next_tag(0), packets(0), is_pipe(false)
{
}

PacketStreamReader::PacketStreamReader(const std::string& filename, bool realtime)
    : next_tag(0), packets(0), is_pipe(pangolin::IsPipe(filename))
{
    Open(filename, realtime);
}

void PacketStreamReader::Open(const std::string& filename, bool realtime)
{
    if (reader.is_open()) {
        Close();
    }

    this->realtime = realtime;
    is_pipe = pangolin::IsPipe(filename);

    reader.open(filename.c_str(), std::ios::in | std::ios::binary);
    if (!reader.good()) {
        throw std::runtime_error("Unable to open file '" + filename + "'.");
    }

    InitInternal();
}

void PacketStreamReader::InitInternal()
{
    ++playback_devices;

    const size_t PANGO_MAGIC_LEN = PANGO_MAGIC.size();
    char buffer[10];

    // Check file magic matches expected value
    reader.read(buffer, PANGO_MAGIC_LEN);
    if (!reader.good() || strncmp((char*)buffer, PANGO_MAGIC.c_str(), PANGO_MAGIC_LEN)) {
        throw std::runtime_error("Unrecognised or corrupted file header.");
    }

    if(!is_pipe) {
        ReadSeekIndex();
    }

    ReadTag();

    // Read any source headers
    while (next_tag == TAG_PANGO_HDR || next_tag == TAG_ADD_SOURCE) {
        ProcessMessage();
    }
}

void PacketStreamReader::Close()
{
    reader.close();
    --playback_devices;

    sources.clear();
}

PacketStreamReader::~PacketStreamReader()
{    
    Close();
}

size_t PacketStreamReader::Seek(PacketStreamSourceId src_id, size_t framenum)
{
    if(src_id > sources.size()) {
        throw std::runtime_error("Invalid Frame Source ID.");
    }

    if(!is_pipe) {
        read_mutex.lock();

        const size_t backup_frame = GetPacketIndex(src_id);

        std::vector<std::streampos>& packet_seek = src_packet_positions[src_id];
        while(framenum >= packet_seek.size()) {
            // We need to read ahead
            int nxt_src_id;
            int64_t time_us;
            ProcessMessagesUntilSourcePacket(nxt_src_id, time_us);
            if(nxt_src_id == -1) {
                framenum = backup_frame;
                break;
            }else{
                ReadOverSourcePacket(nxt_src_id);
                ReadTag();
            }
        }

        // jump to correct position
        src_packet_index[src_id] = framenum;
        reader.clear();
        reader.seekg(packet_seek[framenum]);
        ReadTag();
        if(next_tag != TAG_SRC_PACKET) {
            throw std::runtime_error("Bad seek");
        }

        if(realtime) {
            // Reset the packet count so that playback time is reset.
            packets = 0;
        }

        read_mutex.unlock();
        return framenum;
    }else{
        pango_print_warn("Can't seek on pipe.\n");
        return GetPacketIndex(src_id);
    }
}

bool PacketStreamReader::ReadToSourcePacketAndLock(PacketStreamSourceId src_id)
{
    read_mutex.lock();

    // Walk over data that no-one is interested in
    int nxt_src_id;
    int64_t time_us;

    ProcessMessagesUntilSourcePacket(nxt_src_id, time_us);

    while(nxt_src_id != (int)src_id) {
        if(nxt_src_id == -1) {
            // EOF or something critical
            read_mutex.unlock();
            return false;
        }else{
            ReadOverSourcePacket(nxt_src_id);
            ReadTag();
        }

        ProcessMessagesUntilSourcePacket(nxt_src_id, time_us);
    }

    // Sync time to start of stream if there are no other playback devices
    if(packets == 0 && playback_devices == 1) {
        SetCurrentPlaybackTime_us(time_us);
    }

    if(realtime) {
        WaitUntilPlaybackTime_us(time_us);
    }

    return true;
}

void PacketStreamReader::ReleaseSourcePacketLock(PacketStreamSourceId src_id)
{
    ReadTag();
    ++packets;
    read_mutex.unlock();
}

void PacketStreamReader::ProcessMessage()
{
    // Read one packet / header
    switch (next_tag) {
    case TAG_PANGO_HDR:
        ReadHeaderPacket();
        break;
    case TAG_ADD_SOURCE:
        ReadNewSourcePacket();
        break;
    case TAG_PANGO_STATS:
        ReadStatsPacket();
        break;
    case TAG_PANGO_FOOTER:
        // Ignore this packet
        ReadFooterPacket();
        break;
    case TAG_SRC_JSON:
    {
        size_t src_id = ReadCompressedUnsignedInt();
        if (src_id == static_cast<size_t>(-1)) {
            next_tag = TAG_END;
            return;
        } else if(src_id >= sources.size()) {
            std::cerr << src_id << std::endl;
            throw std::runtime_error("Invalid Frame Source ID.");
        }
        ReadSourcePacketMeta(sources[src_id].meta);
        break;
    }
    case TAG_SRC_PACKET:
    {
        const std::streampos src_packet_pos = reader.tellg() - (std::streamoff)TAG_LENGTH;
        ReadTimestamp(); // read and ignore.
        const size_t src_id = ReadCompressedUnsignedInt();
        if(src_id >= sources.size()) {
            throw std::runtime_error("Invalid Packet Source ID.");
        }
        CacheSrcPacketLocationIncFrame(src_packet_pos, src_id);
        ReadOverSourcePacket(src_id);
        break;
    }
    case TAG_END:
        return;
    default:
        // TODO: Resync
        throw std::runtime_error("Unknown packet type: '" + TagName(next_tag) + "'");
    }

    if(!ReadTag()) {
        // Dummy end tag
        next_tag = TAG_END;
    }
}

// return src_id
void PacketStreamReader::ProcessMessagesUntilSourcePacket(int &nxt_src_id, int64_t& time_us)
{
    while(true)
    {
        // Read one frame / header
        switch (next_tag) {
        case TAG_PANGO_HDR:
            ReadHeaderPacket();
            break;
        case TAG_ADD_SOURCE:
            ReadNewSourcePacket();
            break;
        case TAG_PANGO_STATS:
            ReadStatsPacket();
            break;
        case TAG_PANGO_FOOTER:
            // Ignore this packet
            ReadFooterPacket();
            break;
        case TAG_SRC_JSON:
        {
            size_t src_id = ReadCompressedUnsignedInt();
            if (src_id == static_cast<size_t>(-1)) {
                break;
            } else if(src_id >= sources.size()) {
                std::cerr << src_id << std::endl;
                throw std::runtime_error("Invalid Frame Source ID.");
            }
            ReadSourcePacketMeta(sources[src_id].meta);
            break;
        }
        case TAG_SRC_PACKET:
        {
            const std::streampos src_packet_pos = reader.tellg() - (std::streamoff)TAG_LENGTH;
            time_us = ReadTimestamp();
            size_t src_id = ReadCompressedUnsignedInt();
            if (src_id == static_cast<size_t>(-1)) {
                break;
            } else if(src_id >= sources.size()) {
                throw std::runtime_error("Invalid Packet Source ID.");
            }
            nxt_src_id = static_cast<int>(src_id);
            CacheSrcPacketLocationIncFrame(src_packet_pos, nxt_src_id);
            // return, don't break. We're in the middle of this packet.
            return;
        }
        case TAG_END:
            nxt_src_id = -1;
            return;
        case TAG_PANGO_MAGIC:
        {
            SkipSync();
            nxt_src_id = -1;
            return;
        }
        default:
        {
            if(is_pipe)
            {
                ReSync();
                continue;
            }
            throw std::runtime_error("Unknown packet type.");
        }
        }

        if(!ReadTag()) {
            // Dummy end tag
            next_tag = TAG_END;
        }
    }
}

void PacketStreamReader::ReSync()
{
    uint32_t curr_tag = next_tag;
    char * buffer = (char*)&curr_tag;

    buffer[3] = 0;

    do
    {
        buffer[0] = buffer[1];
        buffer[1] = buffer[2];
        reader.read((char*)&buffer[2], 1);
    } while (reader.good() && curr_tag != TAG_SRC_PACKET);

    if (!reader.good()) {
        next_tag = TAG_END;
    } else {
        next_tag = curr_tag;
    }
}

void PacketStreamReader::SkipSync()
{
    //Assume we have just read PAN, read GO
    char buffer[2];
    reader.read(buffer, 2);

    if (!reader.good()) {
        next_tag = TAG_END;
    }

    if(buffer[0] == 'G' && buffer[1] == 'O')
    {
        do
        {
            //Skip past the header (assuming it's unchanged), to the next src packet
            ReadTag();
        } while (reader.good() && next_tag != TAG_SRC_PACKET);

        if (!reader.good()) {
            next_tag = TAG_END;
        }
    }
    else
        throw std::runtime_error("Unknown packet type.");
}

bool PacketStreamReader::ReadTag()
{
    if(reader.good()) {
        reader.read((char*)&next_tag, TAG_LENGTH );
    }

    return reader.good();
}

void PacketStreamReader::ReadHeaderPacket()
{
    json::value json_header;
    json::parse(json_header, reader);
    reader.get(); // consume newline
}

void PacketStreamReader::ReadSourcePacketMeta(json::value& json)
{
    json::parse(json, reader);
}

void PacketStreamReader::ReadNewSourcePacket()
{
    json::value json;
    json::parse(json, reader);
    reader.get(); // consume newline

    json::value src_driver  = json[json_src_driver];
    json::value src_id      = json[json_src_id];
    json::value src_info    = json[json_src_info];
    json::value src_version = json[json_src_version];
    json::value src_packet  = json[json_src_packet];
    json::value data_size   = src_packet[json_pkt_size_bytes];
    json::value data_defs   = src_packet[json_pkt_definitions];
    json::value data_align  = src_packet[json_pkt_alignment_bytes];

    PacketStreamSource ps;
    ps.driver = src_driver.get<std::string>();
    ps.id = src_id.get<int64_t>();
    ps.info = src_info;
    ps.version = src_version.get<int64_t>();
    ps.data_size_bytes = data_size.get<int64_t>();
    ps.data_definitions = data_defs.get<std::string>();
    ps.data_alignment_bytes = data_align.get<int64_t>();

    sources.push_back(ps);
}

void PacketStreamReader::ReadStatsPacket()
{
    json::value json;
    json::parse(json, reader);

    if(json.contains("src_packet_index")) {
        const json::array& json_index = json["src_packet_index"].get<json::array>();
        for(size_t src=0; src < json_index.size(); ++src) {
            const json::array& json_src_index = json_index[src].get<json::array>();
            src_num_packets[src] = json_src_index.size();
            std::vector<std::streampos>& index = src_packet_positions[src];
            index.resize(json_src_index.size());
            for(size_t f=0; f < json_src_index.size(); ++f) {
                index[f] = json_src_index[f].get<int64_t>();
            }
        }
    }
}

std::streampos PacketStreamReader::ReadFooterPacket()
{
    uint64_t footer_position;
    reader.read((char*)&footer_position, sizeof(uint64_t) );
    return (std::streampos) footer_position;
}

void PacketStreamReader::ReadSeekIndex()
{
    // Store current stream position
    const std::streampos current_pos = reader.tellg();

    // If we were able to backup our position, jump ahead to read footer.
    if(current_pos >= 0) {
        // Move to where we expect TAG_PANGO_PTR to be
        reader.seekg( -(std::ifstream::off_type)(sizeof(uint64_t)+TAG_LENGTH), std::ios_base::end);

        if(ReadTag() && next_tag == TAG_PANGO_FOOTER) {
            const std::streampos stats_pos = ReadFooterPacket();
            reader.seekg(stats_pos);
            if(ReadTag() && next_tag == TAG_PANGO_STATS) {
                ReadStatsPacket();
            }
        }

        // Reset position, regardless of what we achieved above.
        reader.clear();
        reader.seekg(current_pos);
    }
}


void PacketStreamReader::ReadOverSourcePacket(PacketStreamSourceId src_id)
{
    const PacketStreamSource& src = sources[src_id];

    if (src.data_size_bytes > 0) {
        reader.ignore(src.data_size_bytes);
    } else {
        size_t size_bytes = ReadCompressedUnsignedInt();
        if (size_bytes == static_cast<size_t>(-1)) {
            return;
        } else {
            reader.ignore(size_bytes);
        }
    }
}

void PacketStreamReader::CacheSrcPacketLocationIncFrame(std::streampos src_packet_pos, size_t src_id)
{
    const size_t frame_num = src_packet_index[src_id]++;
    std::vector<std::streampos>& packet_seek = src_packet_positions[src_id];
    packet_seek.resize( std::max(frame_num+1, packet_seek.size()) );
    packet_seek[frame_num] = src_packet_pos;
}


}
