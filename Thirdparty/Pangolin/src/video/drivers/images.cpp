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

#include <pangolin/video/drivers/images.h>
#include <pangolin/utils/file_utils.h>

#include <cstring>

namespace pangolin
{

bool ImagesVideo::LoadFrame(size_t i)
{
    if( (int)i < num_files) {
        Frame& frame = loaded[i];
        for(size_t c=0; c< num_channels; ++c) {
            const std::string& filename = Filename(i,c);
            const ImageFileType file_type = FileType(filename);

            if(file_type == ImageFileTypeUnknown && unknowns_are_raw) {
                frame.push_back( LoadImage( filename, raw_fmt, raw_width, raw_height, raw_fmt.bpp * raw_width / 8) );
            }else{
                frame.push_back( LoadImage( filename, file_type ) );
            }
        }
        return true;
    }
    return false;
}

void ImagesVideo::PopulateFilenames(const std::string& wildcard_path)
{
    const std::vector<std::string> wildcards = Expand(wildcard_path, '[', ']', ',');
    num_channels = wildcards.size();

    filenames.resize(num_channels);

    for(size_t i = 0; i < wildcards.size(); ++i) {
        const std::string channel_wildcard = PathExpand(wildcards[i]);
        FilesMatchingWildcard(channel_wildcard, filenames[i]);
        if(num_files < 0) {
            num_files = (int)filenames[i].size();
        }else{
            if( num_files != (int)filenames[i].size() ) {
                std::cerr << "Warning: Video Channels have unequal number of files" << std::endl;
            }
            num_files = std::min(num_files, (int)filenames[i].size());
        }
        if(num_files == 0) {
            throw VideoException("No files found for wildcard '" + channel_wildcard + "'");
        }
    }

    // Resize empty frames vector to hold future images.
    loaded.resize(num_files);
}

void ImagesVideo::ConfigureStreamSizes()
{
    size_bytes = 0;
    for(size_t c=0; c < num_channels; ++c) {
        const TypedImage& img = loaded[0][c];
        const StreamInfo stream_info(img.fmt, img.w, img.h, img.pitch, (unsigned char*)0 + size_bytes);
        streams.push_back(stream_info);
        size_bytes += img.h*img.pitch;
    }
}

ImagesVideo::ImagesVideo(const std::string& wildcard_path)
    : num_files(-1), num_channels(0), next_frame_id(0),
      unknowns_are_raw(false)
{
    // Work out which files to sequence
    PopulateFilenames(wildcard_path);
    
    // Load first image in order to determine stream sizes etc
    LoadFrame(next_frame_id);

    ConfigureStreamSizes();
    
    // TODO: Queue frames in another thread.
}

ImagesVideo::ImagesVideo(const std::string& wildcard_path,
                         const VideoPixelFormat& raw_fmt,
                         size_t raw_width, size_t raw_height
)   : num_files(-1), num_channels(0), next_frame_id(0),
      unknowns_are_raw(true), raw_fmt(raw_fmt),
      raw_width(raw_width), raw_height(raw_height)
{
    // Work out which files to sequence
    PopulateFilenames(wildcard_path);

    // Load first image in order to determine stream sizes etc
    LoadFrame(next_frame_id);

    ConfigureStreamSizes();

    // TODO: Queue frames in another thread.
}

ImagesVideo::~ImagesVideo()
{
    // Free all allocated image data
    for(size_t i=0; i<loaded.size(); ++i) {
        for(size_t c=0; c < loaded[i].size(); ++c) {
            loaded[i][c].Dealloc();
        }
    }
}

//! Implement VideoInput::Start()
void ImagesVideo::Start()
{
    
}

//! Implement VideoInput::Stop()
void ImagesVideo::Stop()
{
    
}

//! Implement VideoInput::SizeBytes()
size_t ImagesVideo::SizeBytes() const
{
    return size_bytes;
}

//! Implement VideoInput::Streams()
const std::vector<StreamInfo>& ImagesVideo::Streams() const
{
    return streams;
}

//! Implement VideoInput::GrabNext()
bool ImagesVideo::GrabNext( unsigned char* image, bool wait )
{
    if(next_frame_id < loaded.size()) {
        Frame& frame = loaded[next_frame_id];

        if(frame.size() != num_channels) {
            LoadFrame(next_frame_id);
        }

        for(size_t c=0; c < num_channels; ++c){
            TypedImage& img = frame[c];
            if(!img.ptr || img.w != streams[c].Width() || img.h != streams[c].Height() ) {
                return false;
            }
            const StreamInfo& si = streams[c];
            std::memcpy(image + (size_t)si.Offset(), img.ptr, si.SizeBytes());
            img.Dealloc();
        }
        frame.clear();

        next_frame_id++;
        return true;
    }
    
    return false;
}

//! Implement VideoInput::GrabNewest()
bool ImagesVideo::GrabNewest( unsigned char* image, bool wait )
{
    return GrabNext(image,wait);
}

int ImagesVideo::GetCurrentFrameId() const
{
    return (int)next_frame_id - 1;
}

int ImagesVideo::GetTotalFrames() const
{
    return num_files;
}

int ImagesVideo::Seek(int frameid)
{
    next_frame_id = std::max(0, std::min(frameid, num_files));
    return (int)next_frame_id;
}


}
