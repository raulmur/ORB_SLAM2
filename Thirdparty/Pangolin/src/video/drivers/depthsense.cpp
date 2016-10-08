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

#include <pangolin/video/drivers/depthsense.h>
#include <iomanip>

namespace pangolin
{

const size_t ROGUE_ADDR = 0x01;
const double MAX_DELTA_TIME = 20000.0; //u_s

DepthSenseContext& DepthSenseContext::I()
{
    static DepthSenseContext s;
    return s;
}

DepthSense::Context& DepthSenseContext::Context()
{
    return g_context;
}

void DepthSenseContext::NewDeviceRunning()
{
    running_devices++;
    if(running_devices == 1) {
        StartNodes();
    }
}

void DepthSenseContext::DeviceClosing()
{
    running_devices--;
    if(running_devices == 0) {
        StopNodes();

        // Force destruction of current context
        g_context = DepthSense::Context();
    }
}

DepthSenseVideo* DepthSenseContext::GetDepthSenseVideo(size_t device_num, DepthSenseSensorType s1, DepthSenseSensorType s2, ImageDim dim1, ImageDim dim2, unsigned int fps1, unsigned int fps2, const Uri& uri)
{
    if(running_devices == 0) {
        // Initialise SDK
        g_context = DepthSense::Context::create("localhost");
    }

    // Get the list of currently connected devices
    std::vector<DepthSense::Device> da = g_context.getDevices();

    if( da.size() > device_num )
    {
        return new DepthSenseVideo(da[device_num], s1, s2, dim1, dim2, fps1, fps2, uri);
    }

    throw VideoException("DepthSense device not connected.");
}

DepthSenseContext::DepthSenseContext()
    : is_running(false), running_devices(0)
{
}

DepthSenseContext::~DepthSenseContext()
{
    StopNodes();
}


void DepthSenseContext::StartNodes()
{
    if(!is_running) {
        // Launch EventLoop thread
        event_thread = boostd::thread(&DepthSenseContext::EventLoop, this );
    }
}

void DepthSenseContext::StopNodes()
{
    if(is_running && event_thread.joinable()) {
        g_context.quit();
        event_thread.join();
    }
}

void DepthSenseContext::EventLoop()
{
    is_running = true;
    g_context.startNodes();
    g_context.run();
    g_context.stopNodes();
    is_running = false;
}

DepthSenseVideo::DepthSenseVideo(DepthSense::Device device, DepthSenseSensorType s1, DepthSenseSensorType s2, ImageDim dim1, ImageDim dim2, unsigned int fps1, unsigned int fps2, const Uri& uri)
    : device(device), fill_image(0), depthmap_stream(-1), rgb_stream(-1), gotDepth(0), gotColor(0),
      enableDepth(false), enableColor(false), depthTs(0.0), colorTs(0.0), size_bytes(0)
{
    streams_properties = &frame_properties["streams"];
    *streams_properties = json::value(json::array_type, false);
    streams_properties->get<json::array>().resize(2);

    sensorConfig[0] = {s1, dim1, fps1};
    sensorConfig[1] = {s2, dim2, fps2};
    ConfigureNodes(uri);

    DepthSenseContext::I().NewDeviceRunning();
}

DepthSenseVideo::~DepthSenseVideo()
{
    if (g_cnode.isSet()) DepthSenseContext::I().Context().unregisterNode(g_cnode);
    if (g_dnode.isSet()) DepthSenseContext::I().Context().unregisterNode(g_dnode);
    
    fill_image = (unsigned char*)ROGUE_ADDR;
    cond_image_requested.notify_all();

    DepthSenseContext::I().DeviceClosing();
}

json::value Json(DepthSense::IntrinsicParameters& p)
{
    json::value js;
    js["model"] = "polynomial";
    js["width"] = p.width;
    js["height"] = p.height;
    js["RDF"] = "[1,0,0; 0,1,0; 0,0,1]";

    js["fx"] = p.fx;
    js["fy"] = p.fy;
    js["u0"] = p.cx;
    js["v0"] = p.cy;
    js["k1"] = p.k1;
    js["k2"] = p.k2;
    js["k3"] = p.k3;
    js["p1"] = p.p1;
    js["p2"] = p.p2;

    return js;
}

json::value Json(DepthSense::ExtrinsicParameters& p)
{
    json::value js;
    js["rows"] = "3";
    js["cols"] = "4";

    std::ostringstream oss;
    oss << std::setprecision(17);
    oss << "[" << p.r11 << "," << p.r12 << "," << p.r13 << "," << p.t1 << ";";
    oss        << p.r21 << "," << p.r22 << "," << p.r23 << "," << p.t2 << ";";
    oss        << p.r31 << "," << p.r32 << "," << p.r33 << "," << p.t3 << "]";

    js["data"] = oss.str();
    return js;
}

void DepthSenseVideo::ConfigureNodes(const Uri& uri)
{
    std::vector<DepthSense::Node> nodes = device.getNodes();

    for (int i = 0; i<2; ++i)
    {
        switch (sensorConfig[i].type)
        {
        case DepthSenseDepth:
        {
            for (int n = 0; n < (int)nodes.size(); n++)
            {
                DepthSense::Node node = nodes[n];
                if ((node.is<DepthSense::DepthNode>()) && (!g_dnode.isSet()))
                {
                    depthmap_stream = i;
                    g_dnode = node.as<DepthSense::DepthNode>();
                    ConfigureDepthNode(sensorConfig[i], uri);
                    DepthSenseContext::I().Context().registerNode(node);
                }
            }
            break;
        }
        case DepthSenseRgb:
        {
            for (int n = 0; n < (int)nodes.size(); n++)
            {
                DepthSense::Node node = nodes[n];
                if ((node.is<DepthSense::ColorNode>()) && (!g_cnode.isSet()))
                {
                    rgb_stream = i;
                    g_cnode = node.as<DepthSense::ColorNode>();
                    ConfigureColorNode(sensorConfig[i], uri);
                    DepthSenseContext::I().Context().registerNode(node);
                }
            }
            break;
        }
        default:
            continue;
        }
    }

    DepthSense::StereoCameraParameters scp = device.getStereoCameraParameters();

    //Set json device properties for intrinsics and extrinsics
    json::value& jsintrinsics = device_properties["intrinsics"];
    if (jsintrinsics.is<json::null>()) {
        jsintrinsics = json::value(json::array_type, false);
        jsintrinsics.get<json::array>().resize(streams.size());
        if (depthmap_stream >= 0) jsintrinsics[depthmap_stream] = Json(scp.depthIntrinsics);
        if (rgb_stream >= 0) jsintrinsics[rgb_stream] = Json(scp.colorIntrinsics);
    }

    json::value& jsextrinsics = device_properties["extrinsics"];
    if(jsextrinsics.is<json::null>()){
        jsextrinsics = Json(scp.extrinsics);
    }
}

inline DepthSense::FrameFormat ImageDim2FrameFormat(const ImageDim& dim)
{
    DepthSense::FrameFormat retVal = DepthSense::FRAME_FORMAT_UNKNOWN;
    if(dim.x == 160 && dim.y == 120)
    {
        retVal = DepthSense::FRAME_FORMAT_QQVGA;
    }
    else if(dim.x == 176 && dim.y == 144)
    {
        retVal = DepthSense::FRAME_FORMAT_QCIF;
    }
    else if(dim.x == 240 && dim.y == 160)
    {
        retVal = DepthSense::FRAME_FORMAT_HQVGA;
    }
    else if(dim.x == 320 && dim.y == 240)
    {
        retVal = DepthSense::FRAME_FORMAT_QVGA;
    }
    else if(dim.x == 352 && dim.y == 288)
    {
        retVal = DepthSense::FRAME_FORMAT_CIF;
    }
    else if(dim.x == 480 && dim.y == 320)
    {
        retVal = DepthSense::FRAME_FORMAT_HVGA;
    }
    else if(dim.x == 640 && dim.y == 480)
    {
        retVal = DepthSense::FRAME_FORMAT_VGA;
    }
    else if(dim.x == 1280 && dim.y == 720)
    {
        retVal = DepthSense::FRAME_FORMAT_WXGA_H;
    }
    else if(dim.x == 320 && dim.y == 120)
    {
        retVal = DepthSense::FRAME_FORMAT_DS311;
    }
    else if(dim.x == 1024 && dim.y == 768)
    {
        retVal = DepthSense::FRAME_FORMAT_XGA;
    }
    else if(dim.x == 800 && dim.y == 600)
    {
        retVal = DepthSense::FRAME_FORMAT_SVGA;
    }
    else if(dim.x == 636 && dim.y == 480)
    {
        retVal = DepthSense::FRAME_FORMAT_OVVGA;
    }
    else if(dim.x == 640 && dim.y == 240)
    {
        retVal = DepthSense::FRAME_FORMAT_WHVGA;
    }
    else if(dim.x == 640 && dim.y == 360)
    {
        retVal = DepthSense::FRAME_FORMAT_NHD;
    }
    return retVal;
}

void DepthSenseVideo::UpdateParameters(const DepthSense::Node& node, const Uri& uri)
{
    DepthSense::Type type = node.getType();
    json::value& jsnode = device_properties[type.name()];
    
    std::vector<DepthSense::PropertyBase> properties = type.getProperties();
    for(std::vector<DepthSense::PropertyBase>::const_iterator it = properties.begin(); it != properties.end(); ++it) {
        const DepthSense::PropertyBase& prop = *it;

        if (prop.is<DepthSense::Property<int32_t> >()) {
            DepthSense::Property<int32_t> tprop = prop.as<DepthSense::Property<int32_t> >();
            if (uri.Contains(prop.name())) {
                if (!prop.isReadOnly()) {
                    tprop.setValue(node, uri.Get<int32_t>(prop.name(), tprop.getValue(node)));
                } else {
                    pango_print_warn("DepthSense property '%s' is read-only\n", prop.name().c_str() );
                }
            }
            jsnode[prop.name()] = tprop.getValue(node);
        } else if (prop.is<DepthSense::Property<float> >()) {
            DepthSense::Property<float> tprop = prop.as<DepthSense::Property<float> >();
            if (uri.Contains(prop.name())) {
                if (!prop.isReadOnly()) {
                    tprop.setValue(node, uri.Get<float>(prop.name(), tprop.getValue(node)));
                } else {
                    pango_print_warn("DepthSense property '%s' is read-only\n", prop.name().c_str() );
                }
            }
            jsnode[prop.name()] = tprop.getValue(node);
        } else if (prop.is<DepthSense::Property<bool> >()) {
            DepthSense::Property<bool> tprop = prop.as<DepthSense::Property<bool> >();
            if (uri.Contains(prop.name())) {
                if (!prop.isReadOnly()) {
                    tprop.setValue(node, uri.Get<bool>(prop.name(), tprop.getValue(node)));
                } else {
                    pango_print_warn("DepthSense property '%s' is read-only\n", prop.name().c_str() );
                }
            }
            jsnode[prop.name()] = tprop.getValue(node);
        } else if (prop.is<DepthSense::Property<std::string> >()){
            DepthSense::Property<std::string> tprop = prop.as<DepthSense::Property<std::string> >();
            if (uri.Contains(prop.name())) {
                if (!prop.isReadOnly()) {
                    tprop.setValue(node, uri.Get<std::string>(prop.name(), tprop.getValue(node)).c_str() );
                } else {
                    pango_print_warn("DepthSense property '%s' is read-only\n", prop.name().c_str() );
                }
            }
            jsnode[prop.name()] = tprop.getValue(node);
        }
    }
}

void DepthSenseVideo::ConfigureDepthNode(const SensorConfig& sensorConfig, const Uri& uri)
{
    g_dnode.newSampleReceivedEvent().connect(this, &DepthSenseVideo::onNewDepthSample);

    DepthSense::DepthNode::Configuration config = g_dnode.getConfiguration();

    config.frameFormat = ImageDim2FrameFormat(sensorConfig.dim);
    config.framerate = sensorConfig.fps;
    config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
    config.saturation = true;

    try {
        DepthSenseContext::I().Context().requestControl(g_dnode, 0);
        g_dnode.setConfiguration(config);
        g_dnode.setEnableDepthMap(true);
    } catch (DepthSense::Exception& e) {
        throw pangolin::VideoException("DepthSense exception whilst configuring node", e.what());
    }

    //Set pangolin stream for this channel
    const int w = sensorConfig.dim.x;
    const int h = sensorConfig.dim.y;

    const VideoPixelFormat pfmt = VideoFormatFromString("GRAY16LE");

    const StreamInfo stream_info(pfmt, w, h, (w*pfmt.bpp) / 8, (unsigned char*)0);
    streams.push_back(stream_info);

    size_bytes += stream_info.SizeBytes();

    enableDepth = true;

    UpdateParameters(g_dnode, uri);
}

void DepthSenseVideo::ConfigureColorNode(const SensorConfig& sensorConfig, const Uri& uri)
{
    // connect new color sample handler
    g_cnode.newSampleReceivedEvent().connect(this, &DepthSenseVideo::onNewColorSample);

    DepthSense::ColorNode::Configuration config = g_cnode.getConfiguration();
    config.frameFormat = ImageDim2FrameFormat(sensorConfig.dim);
    config.compression = DepthSense::COMPRESSION_TYPE_MJPEG;
    config.powerLineFrequency = DepthSense::POWER_LINE_FREQUENCY_50HZ;
    config.framerate = sensorConfig.fps;

    try {
        DepthSenseContext::I().Context().requestControl(g_cnode,0);
        g_cnode.setConfiguration(config);
        g_cnode.setEnableColorMap(true);
        UpdateParameters(g_cnode, uri);
    } catch (DepthSense::Exception& e) {
        throw pangolin::VideoException("DepthSense exception whilst configuring node", e.what());
    }

    //Set pangolin stream for this channel
    const int w = sensorConfig.dim.x;
    const int h = sensorConfig.dim.y;

    const VideoPixelFormat pfmt = VideoFormatFromString("BGR24");

    const StreamInfo stream_info(pfmt, w, h, (w*pfmt.bpp) / 8, (unsigned char*)0 + size_bytes);
    streams.push_back(stream_info);

    size_bytes += stream_info.SizeBytes();

    enableColor = true;
}

void DepthSenseVideo::onNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data)
{
    {
        boostd::unique_lock<boostd::mutex> lock(update_mutex);

        // Wait for fill request
        while (!fill_image) {
            cond_image_requested.wait(lock);
        }

        // Update per-frame parameters
        //printf("Color delta: %.1f\n", fabs(colorTs - data.timeOfCapture));
        colorTs = data.timeOfCapture;
        json::value& jsstream = frame_properties["streams"][rgb_stream];
        jsstream["time_us"] = data.timeOfCapture;

        if (fill_image != (unsigned char*)ROGUE_ADDR) {
            // Fill with data
            unsigned char* imagePtr = fill_image;
            bool copied = false;
            for (int i = 0; !copied && i < 2; ++i)
            {
                switch (sensorConfig[i].type)
                {
                case DepthSenseDepth:
                {
                    imagePtr += streams[i].SizeBytes();
                    break;
                }
                case DepthSenseRgb:
                {
                    // Leave as BGR
                    std::memcpy(imagePtr, data.colorMap, streams[i].SizeBytes());
                    copied = true;
                    break;
                }
                default:
                    continue;
                }
            }
            gotColor++;
        }

        //printf("Got color at: %.1f\n", colorTs);

        if(gotDepth)
        {
            double delta = fabs(GetDeltaTime());
            if(delta > MAX_DELTA_TIME)
            {
                //printf("**** Waiting for another depth, delta: %.1f ****\n", delta);
                gotDepth = 0;
                return;
            }
        }
    }

    cond_image_filled.notify_one();
}

void DepthSenseVideo::onNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
{
    {
        boostd::unique_lock<boostd::mutex> lock(update_mutex);

        // Wait for fill request
        while(!fill_image) {
            cond_image_requested.wait(lock);
        }

        // Update per-frame parameters
        //printf("Depth delta: %.1f\n", fabs(depthTs - data.timeOfCapture));
        depthTs = data.timeOfCapture;

        json::value& jsstream = frame_properties["streams"][depthmap_stream];
        jsstream["time_us"] = depthTs;

        if(fill_image != (unsigned char*)ROGUE_ADDR) {
            // Fill with data
            unsigned char* imagePtr = fill_image;
            bool copied = false;
            for (int i = 0; i < 2; ++i)
            {
                switch (sensorConfig[i].type)
                {
                case DepthSenseDepth:
                {
                    memcpy(imagePtr, data.depthMap, streams[i].SizeBytes());
                    copied = true;
                    break;
                }
                case DepthSenseRgb:
                {
                    imagePtr += streams[i].SizeBytes();
                    break;
                }
                default:
                    continue;
                }
                if(copied)
                {
                    break;
                }
            }
            gotDepth++;
        }

        //printf("Got depth at: %.1f\n", depthTs);

        if(gotColor)
        {
            double delta = fabs(GetDeltaTime());
            if(delta > MAX_DELTA_TIME)
            {
                //printf("**** Waiting for another color, delta: %.1f ****\n", delta);
                gotColor = 0;
                return;
            }
        }
    }

    cond_image_filled.notify_one();
}

void DepthSenseVideo::Start()
{
}

void DepthSenseVideo::Stop()
{
}

size_t DepthSenseVideo::SizeBytes() const
{
    return size_bytes;
}

const std::vector<StreamInfo>& DepthSenseVideo::Streams() const
{
    return streams;
}

bool DepthSenseVideo::GrabNext( unsigned char* image, bool wait )
{
    if(fill_image) {
        throw std::runtime_error("GrabNext Cannot be called concurrently");
    }

    //printf("#### Grab Next ####\n");

    // Request that image is filled with data
    fill_image = image;
    cond_image_requested.notify_one();

    // Wait until it has been filled successfully. 
    {
        boostd::unique_lock<boostd::mutex> lock(update_mutex);
        while ((enableDepth && !gotDepth) || (enableColor && !gotColor))
        {
            cond_image_filled.wait(lock);
        }

        if (gotDepth)
        {
            gotDepth = 0;
        }
        if (gotColor)
        {
            gotColor = 0;
        }
        fill_image = 0;
    }

    //printf("Delta time: %.1f\n", fabs(GetDeltaTime()));

    return true;
}

bool DepthSenseVideo::GrabNewest( unsigned char* image, bool wait )
{
    return GrabNext(image,wait);
}

double DepthSenseVideo::GetDeltaTime() const
{
    return depthTs - colorTs;
}

}
