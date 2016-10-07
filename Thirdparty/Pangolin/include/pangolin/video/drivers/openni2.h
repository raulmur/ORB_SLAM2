/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2014 Richard Newcombe
 *               2014 Steven Lovegrove
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

#ifndef PANGOLIN_OPENNI2_H
#define PANGOLIN_OPENNI2_H

#include <pangolin/pangolin.h>

#include <pangolin/video/video.h>
#include <pangolin/video/drivers/openni_common.h>

#include <OpenNI.h>

namespace pangolin
{
const int MAX_OPENNI2_STREAMS = 2 * ONI_MAX_SENSORS;

//! Interface to video capture sources
struct OpenNiVideo2 : public VideoInterface, public VideoPropertiesInterface, public VideoPlaybackInterface
{
public:

    // Open all RGB and Depth streams from all devices
    OpenNiVideo2(ImageDim dim=ImageDim(640,480), int fps=30);

    // Open streams specified
    OpenNiVideo2(std::vector<OpenNiStreamMode>& stream_modes);

    // Open openni file
    OpenNiVideo2(const std::string& filename);

    // Open openni file with certain params
    OpenNiVideo2(const std::string& filename, std::vector<OpenNiStreamMode>& stream_modes);
    
    void UpdateProperties();

    void SetMirroring(bool enable);
    void SetAutoExposure(bool enable);
    void SetAutoWhiteBalance(bool enable);
    void SetDepthCloseRange(bool enable);
    void SetDepthHoleFilter(bool enable);
    void SetDepthColorSyncEnabled(bool enable);
    void SetRegisterDepthToImage(bool enable);
    void SetPlaybackSpeed(float speed);
    void SetPlaybackRepeat(bool enabled);

    ~OpenNiVideo2();

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

    //! Implement VideoPropertiesInterface::Properties()
    const json::value& DeviceProperties() const {
        return device_properties;
    }

    //! Implement VideoPropertiesInterface::Properties()
    const json::value& FrameProperties() const {
        return frame_properties;
    }

    //! Implement VideoPlaybackInterface::GetCurrentFrameId
    int GetCurrentFrameId() const;

    //! Implement VideoPlaybackInterface::GetTotalFrames
    int GetTotalFrames() const ;

    //! Implement VideoPlaybackInterface::Seek
    int Seek(int frameid);

    openni::VideoStream* GetVideoStream(int stream);

protected:
    void InitialiseOpenNI();
    int AddDevice(const std::string& device_uri);
    void AddStream(const OpenNiStreamMode& mode);
    void SetupStreamModes();
    void PrintOpenNI2Modes(openni::SensorType sensorType);
    openni::VideoMode FindOpenNI2Mode(openni::Device &device, openni::SensorType sensorType, int width, int height, int fps, openni::PixelFormat fmt );

    int numDevices;
    int numStreams;

    openni::Device devices[ONI_MAX_SENSORS];
    OpenNiStreamMode sensor_type[ONI_MAX_SENSORS];

    openni::VideoStream video_stream[ONI_MAX_SENSORS];
    openni::VideoFrameRef video_frame[ONI_MAX_SENSORS];

    std::vector<StreamInfo> streams;
    size_t sizeBytes;

    json::value device_properties;
    json::value frame_properties;
    json::value* streams_properties;

    bool use_depth;
    bool use_ir;
    bool use_rgb;
    bool depth_to_color;
    bool use_ir_and_rgb;
    bool fromFile;

    int current_frame_index;
    int total_frames;
};

}

#endif // PANGOLIN_OPENNI2_H
