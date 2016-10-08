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

#include <pangolin/video/video_record_repeat.h>
#include <pangolin/video/drivers/pango_video.h>
#include <pangolin/video/drivers/pango_video_output.h>

namespace pangolin
{

VideoRecordRepeat::VideoRecordRepeat() 
    : video_src(0), video_file(0), video_recorder(0),
    frame_num(0), record_frame_skip(1), record_once(false), record_continuous(false)
{
}

VideoRecordRepeat::VideoRecordRepeat(
    const std::string& input_uri,
    const std::string& output_uri
    ) : video_src(0), video_file(0), video_recorder(0),
    frame_num(0), record_frame_skip(1), record_once(false), record_continuous(false)
{
    Open(input_uri, output_uri);
}

void VideoRecordRepeat::Open(
    const std::string& input_uri,
    const std::string& output_uri
    ) 
{
    str_uri_input = input_uri;
    uri_input = ParseUri(input_uri);
    uri_output = ParseUri(output_uri);

    if (uri_output.scheme == "file") {
        // Default to pango output
        uri_output.scheme = "pango";
    }

    video_src = OpenVideo(input_uri);

    // Start off playing from video_src
    Source();
}

void VideoRecordRepeat::Close()
{
    if (video_recorder) {
        delete video_recorder;
        video_recorder = 0;
    }
    if (video_src) {
        delete video_src;
        video_src = 0;
    }
    if (video_file) {
        delete video_file;
        video_file = 0;
    }
}

VideoRecordRepeat::~VideoRecordRepeat()
{
    Close();
}

const std::string& VideoRecordRepeat::LogFilename() const
{
    return uri_output.url;
}

bool VideoRecordRepeat::Grab( unsigned char* buffer, std::vector<Image<unsigned char> >& images, bool wait, bool newest)
{
    if( !video_src ) throw VideoException("No video source open");

    bool success;

    if(newest) {
        success = GrabNewest(buffer, wait);
    }else{
        success = GrabNext(buffer, wait);
    }

    if(success) {
        images.clear();
        for(size_t s=0; s < Streams().size(); ++s) {
            images.push_back(Streams()[s].StreamImage(buffer));
        }
    }

    return success;
}

void VideoRecordRepeat::InitialiseRecorder()
{
    if( video_recorder ) {
        video_src->Stop();
        delete video_recorder;
        video_recorder = 0;
    }

    if(video_file) {
        delete video_file;
        video_file = 0;
    }

    video_recorder = OpenVideoOutput(uri_output);
    video_recorder->SetStreams(
        video_src->Streams(), str_uri_input, GetVideoDeviceProperties(video_src) );
}

void VideoRecordRepeat::Record()
{
    // Switch sub-video
    videos.resize(1);
    videos[0] = video_src;

    // Initialise recorder and ensure src is started
    InitialiseRecorder();
    video_src->Start();
    frame_num = 0;
    record_continuous = true;
}

void VideoRecordRepeat::RecordOneFrame()
{
    // Append to existing video.
    if(!video_recorder) {
        InitialiseRecorder();
    }
    record_continuous = false;
    record_once = true;

    // Switch sub-video
    videos.resize(1);
    videos[0] = video_src;
}

void VideoRecordRepeat::Play(bool realtime)
{
    if( video_file ) {
        delete video_file;
        video_file = 0;
    }

    video_src->Stop();

    if(video_recorder) {
        delete video_recorder;
        video_recorder = 0;
    }

    video_file = OpenVideo(
        realtime ? "file:[realtime]//" + uri_output.url :
                   uri_output.url
    );

    frame_num = 0;

    // Switch sub-video
    videos.resize(1);
    videos[0] = video_file;
}

void VideoRecordRepeat::Source()
{
    if(video_file) {
        delete video_file;
        video_file = 0;
    }

    if(video_recorder) {
        delete video_recorder;
        video_recorder = 0;
    }

    frame_num = 0;

    // Switch sub-video
    videos.resize(1);
    videos[0] = video_src;
}

size_t VideoRecordRepeat::SizeBytes() const
{
    if( !video_src ) throw VideoException("No video source open");
    return video_src->SizeBytes();
}

const std::vector<StreamInfo>& VideoRecordRepeat::Streams() const
{
    return video_src->Streams();
}

void VideoRecordRepeat::Start()
{
    // Semantics of this?
//    video_src->Start();
}

void VideoRecordRepeat::Stop()
{
    // Semantics of this?
    if(video_recorder) {
        delete video_recorder;
        video_recorder = 0;
    }
}

bool VideoRecordRepeat::GrabNext( unsigned char* image, bool wait )
{
    frame_num++;

    const bool should_record = (record_continuous && !(frame_num % record_frame_skip)) || record_once;

    if( should_record && video_recorder != 0 ) {
        bool success = video_src->GrabNext(image, wait);
        if( success ) {
            video_recorder->WriteStreams(image, GetVideoFrameProperties(video_src) );
            record_once = false;
        }
        return success;
    }else if( video_file != 0 ) {
        return video_file->GrabNext(image,wait);
    }else{
        return video_src->GrabNext(image,wait);
    }
}

bool VideoRecordRepeat::GrabNewest( unsigned char* image, bool wait )
{
    frame_num++;

    const bool should_record = (record_continuous && !(frame_num % record_frame_skip)) || record_once;

    if( should_record && video_recorder != 0 )
    {
        bool success = video_src->GrabNewest(image,wait);
        if( success) {
            video_recorder->WriteStreams(image, GetVideoFrameProperties(video_src) );
            record_once = false;
        }
        return success;
    }else if( video_file != 0 ) {
        return video_file->GrabNewest(image,wait);
    }else{
        return video_src->GrabNewest(image,wait);
    }
}

int VideoRecordRepeat::FrameId()
{
    return frame_num;
}

void VideoRecordRepeat::SetTimelapse(size_t one_in_n_frames)
{
    record_frame_skip = one_in_n_frames;
}

bool VideoRecordRepeat::IsRecording() const
{
    return video_recorder != 0;
}

bool VideoRecordRepeat::IsPlaying() const
{
    return video_file != 0;
}



}

