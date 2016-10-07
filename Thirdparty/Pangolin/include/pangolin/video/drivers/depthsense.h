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

#ifndef PANGOLIN_DEPTHSENSE_H
#define PANGOLIN_DEPTHSENSE_H

#include <pangolin/pangolin.h>
#include <pangolin/video/video.h>
#include <pangolin/compat/thread.h>
#include <pangolin/compat/mutex.h>
#include <pangolin/compat/condition_variable.h>

// DepthSense SDK for SoftKinetic cameras from Creative
#include <DepthSense.hxx>

namespace pangolin
{

enum DepthSenseSensorType
{
    DepthSenseUnassigned = -1,
    DepthSenseRgb = 0,
    DepthSenseDepth
};

// Video class that outputs test video signal.
class PANGOLIN_EXPORT DepthSenseVideo :
        public VideoInterface, public VideoPropertiesInterface
{
public:
    DepthSenseVideo(DepthSense::Device device, DepthSenseSensorType s1, DepthSenseSensorType s2, ImageDim dim1, ImageDim dim2, unsigned int fps1, unsigned int fps2, const Uri& uri);
    ~DepthSenseVideo();
    
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
    
    //! Implement VideoInput::DeviceProperties()
    const json::value& DeviceProperties() const {
        return device_properties;
    }

    //! Implement VideoInput::DeviceProperties()
    const json::value& FrameProperties() const {
        return frame_properties;
    }
protected:
    void onNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data);
    void onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data);

    struct SensorConfig
    {
        DepthSenseSensorType type;
        ImageDim dim;
        unsigned int fps;
    };

    void UpdateParameters(const DepthSense::Node& node, const Uri& uri);
    void ConfigureNodes(const Uri& uri);
    void ConfigureDepthNode(const SensorConfig& sensorConfig, const Uri& uri);
    void ConfigureColorNode(const SensorConfig& sensorConfig, const Uri& uri);

    double GetDeltaTime() const;

    std::vector<StreamInfo> streams;
    json::value device_properties;
    json::value frame_properties;
    json::value* streams_properties;


    DepthSense::Device device;
    DepthSense::DepthNode g_dnode;
    DepthSense::ColorNode g_cnode;

    unsigned char* fill_image;

    int depthmap_stream;
    int rgb_stream;

    int gotDepth;
    int gotColor;
    boostd::mutex update_mutex;
    boostd::condition_variable cond_image_filled;
    boostd::condition_variable cond_image_requested;

    SensorConfig sensorConfig[2];

    bool enableDepth;
    bool enableColor;
    double depthTs;
    double colorTs;

    size_t size_bytes;
};

class DepthSenseContext
{
    friend class DepthSenseVideo;

public:
    // Singleton Instance
    static DepthSenseContext& I();

    DepthSenseVideo* GetDepthSenseVideo(size_t device_num, DepthSenseSensorType s1, DepthSenseSensorType s2, ImageDim dim1, ImageDim dim2, unsigned int fps1, unsigned int fps2, const Uri& uri);

protected:
    // Protected Constructor 
    DepthSenseContext();
    ~DepthSenseContext();

    DepthSense::Context& Context();

    void NewDeviceRunning();
    void DeviceClosing();

    void StartNodes();
    void StopNodes();

    void EventLoop();

    void onDeviceConnected(DepthSense::Context context, DepthSense::Context::DeviceAddedData data);
    void onDeviceDisconnected(DepthSense::Context context, DepthSense::Context::DeviceRemovedData data);

    DepthSense::Context g_context;

    boostd::thread event_thread;
    bool is_running;

    int running_devices;
};

}

#endif // PANGOLIN_DEPTHSENSE_H
