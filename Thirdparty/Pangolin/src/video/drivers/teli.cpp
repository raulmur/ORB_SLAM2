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

#include <pangolin/video/drivers/teli.h>
#include <pangolin/video/iostream_operators.h>
#include <XmlFeatures.h>

namespace pangolin
{

// Represet lifetime of Teli SDK. Destructed by static deinitialisation.
class TeliSystem
{
public:
    static TeliSystem& Instance() {
        static TeliSystem sys;
        return sys;
    }

private:
    TeliSystem()
    {
        Teli::CAM_API_STATUS uiStatus = Teli::Sys_Initialize();
        if (uiStatus != Teli::CAM_API_STS_SUCCESS && uiStatus != Teli::CAM_API_STS_ALREADY_INITIALIZED)
            throw pangolin::VideoException("Unable to initialise TeliSDK.");
    }

    ~TeliSystem()
    {
        Teli::CAM_API_STATUS uiStatus = Teli::Sys_Terminate();
        if (uiStatus != Teli::CAM_API_STS_SUCCESS) {
            pango_print_warn("TeliSDK: Error uninitialising.");
        }
    }
};

std::string GetNodeValStr(Teli::CAM_HANDLE cam, Teli::CAM_NODE_HANDLE node)
{
    Teli::TC_NODE_TYPE node_type;
    Teli::CAM_API_STATUS st = Teli::Nd_GetType(cam, node, &node_type);
    if(st != Teli::CAM_API_STS_SUCCESS) {
        throw std::runtime_error("TeliSDK: Unable to get Teli node type.");
    }

    switch(node_type) {
    case Teli::TC_NODE_TYPE_INTEGER:
    {
        int64_t val;
        Teli::CAM_API_STATUS st = Teli::Nd_GetIntValue(cam, node, &val);
        if(st == Teli::CAM_API_STS_SUCCESS) {
            return pangolin::Convert<std::string, int64_t>::Do(val);
        }else{
            throw std::runtime_error("TeliSDK: Unable to get Teli parameter");
        }
    }
    case Teli::TC_NODE_TYPE_BOOLEAN:
    {
        bool8_t val;
        Teli::CAM_API_STATUS st = Teli::Nd_GetBoolValue(cam, node, &val);
        if(st == Teli::CAM_API_STS_SUCCESS) {
            return pangolin::Convert<std::string, bool8_t>::Do(val);
        }else{
            throw std::runtime_error("TeliSDK: Unable to get Teli parameter");
        }
    }
    case Teli::TC_NODE_TYPE_FLOAT:
    {
        float64_t val;
        Teli::CAM_API_STATUS st = Teli::Nd_GetFloatValue(cam, node, &val);
        if(st == Teli::CAM_API_STS_SUCCESS) {
            return pangolin::Convert<std::string, float64_t>::Do(val);
        }else{
            throw std::runtime_error("TeliSDK: Unable to get Teli parameter");
        }
    }
    case Teli::TC_NODE_TYPE_STRING:
    {
        uint32_t buffer_size = 10*1024;
        char* buffer = new char[buffer_size];
        Teli::CAM_API_STATUS st = Teli::Nd_GetStrValue(cam, node, buffer, &buffer_size);
        std::string val(buffer);
        delete[] buffer;
        if(st == Teli::CAM_API_STS_SUCCESS) {
            return val;
        }else{
            throw std::runtime_error("TeliSDK: Unable to get Teli parameter");
        }
    }
    case Teli::TC_NODE_TYPE_ENUMERATION:
    {
        uint32_t buffer_size = 10*1024;
        char* buffer = new char[buffer_size];
        Teli::CAM_API_STATUS st = Teli::Nd_GetEnumStrValue(cam, node, buffer, &buffer_size);
        std::string val(buffer);
        if(st == Teli::CAM_API_STS_SUCCESS) {
            return val;
        }else{
            throw std::runtime_error("TeliSDK: Unable to get Teli parameter");
        }
    }
    case Teli::TC_NODE_TYPE_COMMAND:
    case Teli::TC_NODE_TYPE_REGISTER:
    case Teli::TC_NODE_TYPE_CATEGORY:
    case Teli::TC_NODE_TYPE_ENUM_ENTRY:
    case Teli::TC_NODE_TYPE_PORT:
    default:
        throw std::runtime_error("TeliSDK: Unsupported node_type: " + node_type);
    }
}

void SetNodeValStr(Teli::CAM_HANDLE cam, Teli::CAM_NODE_HANDLE node, std::string val_str)
{
    Teli::TC_NODE_TYPE node_type;
    Teli::CAM_API_STATUS st = Teli::Nd_GetType(cam, node, &node_type);
    if(st != Teli::CAM_API_STS_SUCCESS) {
        throw std::runtime_error("TeliSDK: Unable to get Teli node type.");
    }

    switch(node_type) {
    case Teli::TC_NODE_TYPE_INTEGER:
    {
        const int64_t val = pangolin::Convert<int64_t, std::string>::Do(val_str);
        Teli::CAM_API_STATUS st = Teli::Nd_SetIntValue(cam, node, val);
        if(st != Teli::CAM_API_STS_SUCCESS) {
            throw std::runtime_error("TeliSDK: Unable to set Teli parameter");
        }
        break;
    }
    case Teli::TC_NODE_TYPE_BOOLEAN:
    {
        const bool8_t val = pangolin::Convert<bool8_t, std::string>::Do(val_str);
        Teli::CAM_API_STATUS st = Teli::Nd_SetBoolValue(cam, node, val);
        if(st != Teli::CAM_API_STS_SUCCESS) {
            throw std::runtime_error("TeliSDK: Unable to set Teli parameter");
        }
        break;
    }
    case Teli::TC_NODE_TYPE_FLOAT:
    {
        const float64_t val = pangolin::Convert<float64_t, std::string>::Do(val_str);
        Teli::CAM_API_STATUS st = Teli::Nd_SetFloatValue(cam, node, val);
        if(st != Teli::CAM_API_STS_SUCCESS) {
            throw std::runtime_error("TeliSDK: Unable to set Teli parameter");
        }
        break;
    }
    case Teli::TC_NODE_TYPE_STRING:
    {
        Teli::CAM_API_STATUS st = Teli::Nd_SetStrValue(cam, node, val_str.c_str());
        if(st != Teli::CAM_API_STS_SUCCESS) {
            throw std::runtime_error("TeliSDK: Unable to set Teli parameter");
        }
        break;
    }
    case Teli::TC_NODE_TYPE_ENUMERATION:
    {
        Teli::CAM_API_STATUS st = Teli::Nd_SetEnumStrValue(cam, node, val_str.c_str());
        if(st != Teli::CAM_API_STS_SUCCESS) {
            throw std::runtime_error("TeliSDK: Unable to set Teli parameter");
        }
        break;
    }
    case Teli::TC_NODE_TYPE_COMMAND:
    case Teli::TC_NODE_TYPE_REGISTER:
    case Teli::TC_NODE_TYPE_CATEGORY:
    case Teli::TC_NODE_TYPE_ENUM_ENTRY:
    case Teli::TC_NODE_TYPE_PORT:
    default:
        throw std::runtime_error("TeliSDK: Unsupported node_type: " + node_type);
    }
}

TeliVideo::TeliVideo(const Params& uri)
	: cam(0), strm(0), hStrmCmpEvt(0)
{
    TeliSystem::Instance();

    uint32_t num_cams = 0;
    Teli::CAM_API_STATUS uiStatus = Teli::Sys_GetNumOfCameras(&num_cams);
    if (uiStatus != Teli::CAM_API_STS_SUCCESS)
        throw pangolin::VideoException("Unable to enumerate TeliSDK cameras.");

    if (num_cams == 0)
     throw pangolin::VideoException("No TeliSDK Cameras available.");

    // Default to rogue values
    ImageRoi roi;
    std::string sn;
    std::string mn;
    int cam_index = 0;

    Params device_params;

    for(Params::ParamMap::const_iterator it = uri.params.begin(); it != uri.params.end(); it++) {
        if(it->first == "model"){
            mn = it->second;
        } else if(it->first == "sn"){
            sn = it->second;
        } else if(it->first == "idx"){
            cam_index = uri.Get<int>("idx", 0);
        } else if(it->first == "roi") {
            roi = uri.Get<ImageRoi>("roi", ImageRoi(0,0,0,0) );
        } else {
            device_params.Set(it->first, it->second);
        }
    }

    if(sn.empty() && mn.empty()) {
        uiStatus = Teli::Cam_Open(cam_index, &cam, 0, true, 0);
    }else{
        uiStatus = Teli::Cam_OpenFromInfo(
            (sn.empty() ? 0 : sn.c_str()),
            (mn.empty() ? 0 : mn.c_str()),
            0, &cam, 0, true, 0
        );
    }
    if (uiStatus != Teli::CAM_API_STS_SUCCESS)
        throw pangolin::VideoException("TeliSDK: Error opening camera");

    uint32_t width = 0;
    uint32_t height = 0;
    uiStatus = Teli::GetCamSensorWidth(cam, &width);
    if (uiStatus != Teli::CAM_API_STS_SUCCESS)
      throw pangolin::VideoException("Unable to get TeliSDK Camera dimensions");

    uiStatus = Teli::GetCamSensorHeight(cam, &height);
    if (uiStatus != Teli::CAM_API_STS_SUCCESS)
        throw pangolin::VideoException("Unable to get TeliSDK Camera dimensions");

    // If roi not set, use cameras native resolution.
    if(roi.w ==0 || roi.h==0) {
        roi = ImageRoi(0, 0, width, height);
    }

    SetDeviceParams(device_params);
    Initialise(roi);
}

std::string TeliVideo::GetParameter(const std::string& name)
{
    Teli::CAM_NODE_HANDLE node;
    Teli::CAM_API_STATUS st = Teli::Nd_GetNode(cam, name.c_str(), &node);
    if( st == Teli::CAM_API_STS_SUCCESS) {
        return GetNodeValStr(cam, node);
    }else{
        throw std::runtime_error("TeliSDK: Unable to get reference to node:" + name);
    }
}

void TeliVideo::SetParameter(const std::string& name, const std::string& value)
{
    Teli::CAM_NODE_HANDLE node;
    Teli::CAM_API_STATUS st = Teli::Nd_GetNode(cam, name.c_str(), &node);
    if( st == Teli::CAM_API_STS_SUCCESS) {
        SetNodeValStr(cam, node, value);
    }else{
        throw std::runtime_error("TeliSDK: Unable to get reference to node:" + name);
    }
}

void TeliVideo::Initialise(const ImageRoi& roi)
{
    Teli::CAM_API_STATUS uiStatus = Teli::SetCamRoi(cam, (uint32_t)roi.w, (uint32_t)roi.h, (uint32_t)roi.x, (uint32_t)roi.y);
    if (uiStatus != Teli::CAM_API_STS_SUCCESS) {
        std::cerr << "Error: " << std::hex << uiStatus << std::endl;
        throw pangolin::VideoException("TeliSDK: Error setting SetCamRoi.");
    }

    // Create completion event object for stream.
#ifdef _WIN_
    hStrmCmpEvt = CreateEvent(NULL, FALSE, FALSE, NULL);
    if (hStrmCmpEvt == NULL)
        throw pangolin::VideoException("TeliSDK: Error creating event.");
#endif
#ifdef _LINUX_
    uiStatus = Teli::Sys_CreateSignal(&hStrmCmpEvt);
    if (uiStatus != Teli::CAM_API_STS_SUCCESS)
        throw pangolin::VideoException("TeliSDK: Error creating event.");
#endif
    uint32_t uiPyldSize = 0;
    uiStatus = Teli::Strm_OpenSimple(cam, &strm, &uiPyldSize, hStrmCmpEvt);
    if (uiStatus != Teli::CAM_API_STS_SUCCESS)
        throw pangolin::VideoException("TeliSDK: Error opening camera stream.");

//    Teli::SetCamPixelFormat(cam, Teli::PXL_FMT_Mono12);

    // Read pixel format
    VideoPixelFormat pfmt;
    Teli::CAM_PIXEL_FORMAT teli_fmt;
    uiStatus = Teli::GetCamPixelFormat(cam, &teli_fmt);
    if (uiStatus != Teli::CAM_API_STS_SUCCESS)
        throw pangolin::VideoException("TeliSDK: Error calling GetCamPixelFormat.");

    switch( teli_fmt) {
    case Teli::PXL_FMT_Mono8:
    case Teli::PXL_FMT_BayerGR8:
    case Teli::PXL_FMT_BayerBG8:
        pfmt = pangolin::VideoFormatFromString("GRAY8");
        break;
    case Teli::PXL_FMT_Mono10:
    case Teli::PXL_FMT_Mono12:
    case Teli::PXL_FMT_Mono16:
    case Teli::PXL_FMT_BayerGR10:
    case Teli::PXL_FMT_BayerGR12:
    case Teli::PXL_FMT_BayerBG10:
    case Teli::PXL_FMT_BayerBG12:
        pfmt = pangolin::VideoFormatFromString("GRAY16LE");
        break;
    case Teli::PXL_FMT_RGB8:
        pfmt = pangolin::VideoFormatFromString("RGB24");
        break;
    case Teli::PXL_FMT_BGR8:
        pfmt = pangolin::VideoFormatFromString("BGR24");
        break;
    default:
        throw std::runtime_error("TeliSDK: Unknown pixel format: " + ToString<int>(teli_fmt) );
    }

    size_bytes = 0;
    
    const int n = 1;
    for(size_t c=0; c < n; ++c) {
        const StreamInfo stream_info(pfmt, roi.w, roi.h, (roi.w*pfmt.bpp) / 8, 0);
        streams.push_back(stream_info);        
        size_bytes += uiPyldSize;
    }

    Start();
}

void TeliVideo::SetDeviceParams(const Params& p)
{
    for(Params::ParamMap::const_iterator it = p.params.begin(); it != p.params.end(); it++) {
        try{
            if (it->second == "Execute") {
                //
                std::runtime_error("TeliSDK: Execution commands not yet supported.");
            } else {
                SetParameter(it->first, it->second);
            }
        }catch(std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
    }
}

TeliVideo::~TeliVideo()
{
    Teli::CAM_API_STATUS uiStatus = Teli::Strm_Close(strm);
    if (uiStatus != Teli::CAM_API_STS_SUCCESS)
        pango_print_warn("TeliSDK: Error closing camera stream.");

    uiStatus = Teli::Cam_Close(cam);
    if (uiStatus != Teli::CAM_API_STS_SUCCESS)
        pango_print_warn("TeliSDK: Error closing camera.");
}

//! Implement VideoInput::Start()
void TeliVideo::Start()
{
    Teli::CAM_API_STATUS uiStatus = Teli::Strm_Start(strm);
    if (uiStatus != Teli::CAM_API_STS_SUCCESS)
        throw pangolin::VideoException("TeliSDK: Error starting stream.");
}

//! Implement VideoInput::Stop()
void TeliVideo::Stop()
{
    Teli::CAM_API_STATUS uiStatus = Teli::Strm_Stop(strm);
    if (uiStatus != Teli::CAM_API_STS_SUCCESS)
        throw pangolin::VideoException("TeliSDK: Error stopping stream.");
}

//! Implement VideoInput::SizeBytes()
size_t TeliVideo::SizeBytes() const
{
    return size_bytes;
}

//! Implement VideoInput::Streams()
const std::vector<StreamInfo>& TeliVideo::Streams() const
{
    return streams;
}

//! Implement VideoInput::GrabNext()
bool TeliVideo::GrabNext(unsigned char* image, bool wait)
{
#ifdef _WIN_
    unsigned int uiRet = WaitForSingleObject(hStrmCmpEvt, 2000);
    if (uiRet == WAIT_OBJECT_0) {
#endif
#ifdef _LINUX_
    unsigned int uiRet = Teli::Sys_WaitForSignal(hStrmCmpEvt, 2000);
    if (uiRet == Teli::CAM_API_STS_SUCCESS) {
#endif
        Teli::CAM_IMAGE_INFO sImageInfo;
        uint32_t uiPyldSize = (uint32_t)size_bytes;
        Teli::CAM_API_STATUS uiStatus = Teli::Strm_ReadCurrentImage(strm, image, &uiPyldSize, &sImageInfo);
        frame_properties[PANGO_CAPTURE_TIME_US] = json::value(sImageInfo.ullTimestamp/1000);
        frame_properties[PANGO_HOST_RECEPTION_TIME_US] = json::value(pangolin::Time_us(pangolin::TimeNow()));
        return (uiStatus == Teli::CAM_API_STS_SUCCESS);
    }

    return false;
}

//! Implement VideoInput::GrabNewest()
bool TeliVideo::GrabNewest(unsigned char* image, bool wait)
{
    return GrabNext(image,wait);
}

//! Returns number of available frames
uint32_t TeliVideo::AvailableFrames() const
{
    uint32_t puiCount = 0;
    Teli::CAM_API_STATUS uiStatus = Teli::GetCamImageBufferFrameCount(cam, &puiCount);
    if (uiStatus != Teli::CAM_API_STS_SUCCESS)
        throw pangolin::VideoException("TeliSDK: Error reading frame buffer frame count.");
    return puiCount;
}

//! Drops N frames in the queue starting from the oldest
//! returns false if less than n frames arae available
bool TeliVideo::DropNFrames(uint32_t n)
{
    for (uint32_t i=0;i<n;++i) {
#ifdef _WIN_
        unsigned int uiRet = WaitForSingleObject(hStrmCmpEvt, 2000);
        if (uiRet == WAIT_OBJECT_0) {
#endif
#ifdef _LINUX_
            unsigned int uiRet = Teli::Sys_WaitForSignal(hStrmCmpEvt, 2000);
            if (uiRet == Teli::CAM_API_STS_SUCCESS) {
#endif
                Teli::CAM_IMAGE_INFO sImageInfo;
                uint32_t uiPyldSize = 0 ;
                Teli::Strm_ReadCurrentImage(strm, 0, &uiPyldSize, &sImageInfo);
            } else {
                return false;
            }
        }
    return true;
}

//! Access JSON properties of device
const json::value& TeliVideo::DeviceProperties() const
{
        return device_properties;
}

//! Access JSON properties of most recently captured frame
const json::value& TeliVideo::FrameProperties() const
{
    return frame_properties;
}

}
