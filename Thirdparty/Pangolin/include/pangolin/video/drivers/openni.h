#ifndef PANGOLIN_OPENNI_H
#define PANGOLIN_OPENNI_H

#include <pangolin/pangolin.h>

#include <pangolin/video/video.h>
#include <pangolin/video/drivers/openni_common.h>

// Workaround poor OpenNI Platform test on Linux before including XnCppWrapper.h
// See https://github.com/dennishamester/OpenNI/commit/ca99f6181234c682bba42a6ba
#ifdef _LINUX_
#define linux 1
#endif // _LINUX_

// OpenNI generates SO MANY warnings, we'll just disable all for this header(!)
// GCC and clang will listen to this pramga.
#ifndef _MSVC_
#pragma GCC system_header
#endif
#include <XnCppWrapper.h>

namespace pangolin
{

//! Interface to video capture sources
struct PANGOLIN_EXPORT OpenNiVideo : public VideoInterface
{
public:
    OpenNiVideo(OpenNiSensorType s1, OpenNiSensorType s2, ImageDim dim = ImageDim(640,480), int fps = 30);
    ~OpenNiVideo();

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

    void SetAutoExposure(bool enabled)
    {
#if XN_MINOR_VERSION > 5 || (XN_MINOR_VERSION == 5 && XN_BUILD_VERSION >= 7)
        if(imageNode.IsValid()) {
            imageNode.GetAutoExposureCap().Set(enabled ? 1 : 0);
        }
#else
        throw pangolin::VideoException("SetAutoExposure Not supported for this version of OpenNI.");
#endif
    }
    
protected:
    std::vector<StreamInfo> streams;
    OpenNiSensorType sensor_type[2];
    
    xn::Context context;
    xn::DepthGenerator depthNode;
    xn::ImageGenerator imageNode;
    xn::IRGenerator irNode;
    
    size_t sizeBytes;
};

}

#endif // PANGOLIN_OPENNI_H
