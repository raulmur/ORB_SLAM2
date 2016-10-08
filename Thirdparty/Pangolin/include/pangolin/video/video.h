/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove
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

#ifndef PANGOLIN_VIDEO_H
#define PANGOLIN_VIDEO_H

// Pangolin video supports various cameras and file formats through
// different 3rd party libraries.
//
// Video URI's take the following form:
//  scheme:[param1=value1,param2=value2,...]//device
//
// scheme = file | files | pango | shmem | dc1394 | uvc | v4l | openni2 |
//          openni | depthsense | pleora | teli | mjpeg | test |
//          thread | convert | debayer | split | join | shift | mirror | unpack
//
// file/files - read one or more streams from image file(s) / video
//  e.g. "files://~/data/dataset/img_*.jpg"
//  e.g. "files://~/data/dataset/img_[left,right]_*.pgm"
//  e.g. "files:///home/user/sequence/foo%03d.jpeg"
//
//  e.g. "file:[fmt=GRAY8,size=640x480]///home/user/raw_image.bin"
//  e.g. "file:[realtime=1]///home/user/video/movie.pango"
//  e.g. "file:[stream=1]///home/user/video/movie.avi"
//
// dc1394 - capture video through a firewire camera
//  e.g. "dc1394:[fmt=RGB24,size=640x480,fps=30,iso=400,dma=10]//0"
//  e.g. "dc1394:[fmt=FORMAT7_1,size=640x480,pos=2+2,iso=400,dma=10]//0"
//  e.g. "dc1394:[fmt=FORMAT7_3,deinterlace=1]//0"
//
// v4l - capture video from a Video4Linux (USB) camera (normally YUVY422 format)
//           method=mmap|read|userptr
//  e.g. "v4l:///dev/video0"
//  e.g. "v4l[method=mmap]:///dev/video0"
//
// openni2 - capture video / depth from OpenNI2 SDK  (Kinect / Xtrion etc)
//           imgN=grey|rgb|ir|ir8|ir24|depth|reg_depth
//  e.g. "openni2://'
//  e.g. "openni2:[img1=rgb,img2=depth,coloursync=true]//"
//  e.g. "openni2:[img1=depth,close=closerange,holefilter=true]//"
//  e.g. "openni2:[size=320x240,fps=60,img1=ir]//"
//
// openni - capture video / depth from OpenNI 1.0 SDK (Kinect / Xtrion etc)
//           Sensor modes containing '8' will truncate to 8-bits.
//           Sensor modes containing '+' explicitly enable IR illuminator
//           imgN=rgb|ir|ir8|ir+|ir8+|depth|reg_depth
//           autoexposure=true|false
//  e.g. "openni://'
//  e.g. "openni:[img1=rgb,img2=depth]//"
//  e.g. "openni:[size=320x240,fps=60,img1=ir]//"
//
// depthsense - capture video / depth from DepthSense SDK.
//              DepthSenseViewer can be used to alter capture settings.
//              imgN=depth|rgb
//              sizeN=QVGA|320x240|...
//              fpsN=25|30|60|...
//  e.g. "depthsense://"
//  e.g. "depthsense:[img1=depth,img2=rgb]//"
//
// pleora - USB 3 vision cameras accepts any option in the same format reported by eBUSPlayer
//  e.g. for lightwise cameras: "pleora:[size=512x256,pos=712x512,sn=00000274,ExposureTime=10000,PixelFormat=Mono12p,AcquisitionMode=SingleFrame,TriggerSource=Line0,TriggerMode=On]//"
//  e.g. for toshiba cameras: "pleora:[size=512x256,pos=712x512,sn=0300056,PixelSize=Bpp12,ExposureTime=10000,ImageFormatSelector=Format1,BinningHorizontal=2,BinningVertical=2]//"
//  e.g. toshiba alternated "pleora:[UserSetSelector=UserSet1,ExposureTime=10000,PixelSize=Bpp12,Width=1400,OffsetX=0,Height=1800,OffsetY=124,LineSelector=Line1,LineSource=ExposureActive,LineSelector=Line2,LineSource=Off,LineModeAll=6,LineInverterAll=6,UserSetSave=Execute,
//                                   UserSetSelector=UserSet2,PixelSize=Bpp12,Width=1400,OffsetX=1048,Height=1800,OffsetY=124,ExposureTime=10000,LineSelector=Line1,LineSource=Off,LineSelector=Line2,LineSource=ExposureActive,LineModeAll=6,LineInverterAll=6,UserSetSave=Execute,
//                                   SequentialShutterIndex=1,SequentialShutterEntry=1,SequentialShutterIndex=2,SequentialShutterEntry=2,SequentialShutterTerminateAt=2,SequentialShutterEnable=On,,AcquisitionFrameRateControl=Manual,AcquisitionFrameRate=70]//"
//
// thread - thread that continuously pulls from the child streams so that data in, unpacking, debayering etc can be decoupled from the main application thread
//  e.g. thread://pleora://
//  e.g. thread://unpack://pleora:[PixelFormat=Mono12p]//
//
// convert - use FFMPEG to convert between video pixel formats
//  e.g. "convert:[fmt=RGB24]//v4l:///dev/video0"
//  e.g. "convert:[fmt=GRAY8]//v4l:///dev/video0"
//
// mjpeg - capture from (possibly networked) motion jpeg stream using FFMPEG
//  e.g. "mjpeg://http://127.0.0.1/?action=stream"
//
// debayer - debayer an input video stream
//  e.g.  "debayer:[tile="BGGR",method="downsample"]//v4l:///dev/video0
//
// split - split an input video into a one or more streams based on Region of Interest / memory specification
//           roiN=X+Y+WxH
//           memN=Offset:WxH:PitchBytes:Format
//  e.g. "split:[roi1=0+0+640x480,roi2=640+0+640x480]//files:///home/user/sequence/foo%03d.jpeg"
//  e.g. "split:[mem1=307200:640x480:1280:GRAY8,roi2=640+0+640x480]//files:///home/user/sequence/foo%03d.jpeg"
//  e.g. "split:[stream1=2,stream2=1]//pango://video.pango"
//
// join - join streams
//  e.g. "join:[sync_tolerance_us=100, sync_continuously=true]//{pleora:[sn=00000274]//}{pleora:[sn=00000275]//}"
//
// test - output test video sequence
//  e.g. "test://"
//  e.g. "test:[size=640x480,fmt=RGB24]//"

#include <pangolin/compat/function.h>
#include <pangolin/image/image.h>
#include <pangolin/image/image_common.h>
#include <pangolin/utils/picojson.h>

#include <vector>

#define PANGO_HOST_RECEPTION_TIME_US "host_reception_time_us"
#define PANGO_CAPTURE_TIME_US "capture_time_us"
#define PANGO_EXPOSURE_US "exposure_us"
#define PANGO_GAMMA "gamma"
#define PANGO_ANALOG_GAIN "analog_gain"
#define PANGO_ANALOG_BLACK_LEVEL "analog_black_level"
#define PANGO_SENSOR_TEMPERATURE_C "sensor_temperature_C"


namespace pangolin
{

class PANGOLIN_EXPORT StreamInfo
{
public:
    inline StreamInfo()
        : fmt(VideoFormatFromString("GRAY8")) {}

    inline StreamInfo(VideoPixelFormat fmt, const Image<unsigned char> img_offset )
        : fmt(fmt), img_offset(img_offset) {}

    inline StreamInfo(VideoPixelFormat fmt, size_t w, size_t h, size_t pitch, unsigned char* offset = 0)
        : fmt(fmt), img_offset(w,h,pitch,offset) {}
    
    //! Format representing how image is layed out in memory
    inline VideoPixelFormat PixFormat() const { return fmt; }
    
    //! Image width in pixels
    inline size_t Width() const { return img_offset.w; }
    
    //! Image height in pixels
    inline size_t Height() const { return img_offset.h; }

    inline double Aspect() const { return (double)Width() / (double)Height(); }
    
    //! Pitch: Number of bytes between one image row and the next
    inline size_t Pitch() const { return img_offset.pitch; }

    //! Number of contiguous bytes in memory that the image occupies
    inline size_t RowBytes() const {
        // Row size without padding
        return (fmt.bpp*img_offset.w)/8;
    }
    
    //! Returns true iff image contains padding or stridded access
    //! This implies that the image data is not contiguous in memory.
    inline bool IsPitched() const {
        return Pitch() != RowBytes();
    }

    //! Number of contiguous bytes in memory that the image occupies
    inline size_t SizeBytes() const {
        return (img_offset.h-1) * img_offset.pitch + RowBytes();
    }

    //! Offset in bytes relative to start of frame buffer
    inline unsigned char* Offset() const { return img_offset.ptr; }
    
    //! Return Image wrapper around raw base pointer
    inline Image<unsigned char> StreamImage(unsigned char* base_ptr) const {
        Image<unsigned char> img = img_offset;
        img.ptr += (size_t)base_ptr;
        return img;
    }

    //! Return Image wrapper around raw base pointer
    inline const Image<unsigned char> StreamImage(const unsigned char* base_ptr) const {
        Image<unsigned char> img = img_offset;
        img.ptr += (size_t)base_ptr;
        return img;
    }

protected:
    VideoPixelFormat fmt;        
    Image<unsigned char> img_offset;
};

//! Interface to video capture sources
struct PANGOLIN_EXPORT VideoInterface
{
    virtual ~VideoInterface() {}

    //! Required buffer size to store all frames
    virtual size_t SizeBytes() const = 0;
    
    //! Get format and dimensions of all video streams
    virtual const std::vector<StreamInfo>& Streams() const = 0;
    
    //! Start Video device
    virtual void Start() = 0;
    
    //! Stop Video device
    virtual void Stop() = 0;
    
    //! Copy the next frame from the camera to image.
    //! Optionally wait for a frame if one isn't ready
    //! Returns true iff image was copied
    virtual bool GrabNext( unsigned char* image, bool wait = true ) = 0;
    
    //! Copy the newest frame from the camera to image
    //! discarding all older frames.
    //! Optionally wait for a frame if one isn't ready
    //! Returns true iff image was copied
    virtual bool GrabNewest( unsigned char* image, bool wait = true ) = 0;
};

//! Interface to GENICAM video capture sources
struct PANGOLIN_EXPORT GenicamVideoInterface
{
    virtual ~GenicamVideoInterface() {}

    virtual std::string GetParameter(const std::string& name) = 0;

    virtual void SetParameter(const std::string& name, const std::string& value) = 0;

};

struct PANGOLIN_EXPORT BufferAwareVideoInterface
{
    virtual ~BufferAwareVideoInterface() {}

    //! Returns number of available frames
    virtual uint32_t AvailableFrames() const = 0;

    //! Drops N frames in the queue starting from the oldest
    //! returns false if less than n frames arae available
    virtual bool DropNFrames(uint32_t n) = 0;
};

struct PANGOLIN_EXPORT VideoPropertiesInterface
{
    virtual ~VideoPropertiesInterface() {}

    //! Access JSON properties of device
    virtual const json::value& DeviceProperties() const = 0;

    //! Access JSON properties of most recently captured frame
    virtual const json::value& FrameProperties() const = 0;
};

enum UvcRequestCode {
  UVC_RC_UNDEFINED = 0x00,
  UVC_SET_CUR = 0x01,
  UVC_GET_CUR = 0x81,
  UVC_GET_MIN = 0x82,
  UVC_GET_MAX = 0x83,
  UVC_GET_RES = 0x84,
  UVC_GET_LEN = 0x85,
  UVC_GET_INFO = 0x86,
  UVC_GET_DEF = 0x87
};

struct PANGOLIN_EXPORT VideoFilterInterface
{
    virtual ~VideoFilterInterface() {}

    template<typename T>
    std::vector<T*> FindMatchingStreams()
    {
        std::vector<T*> matches;
        std::vector<VideoInterface*> children = InputStreams();
        for(size_t c=0; c < children.size(); ++c) {
            T* concrete_video = dynamic_cast<T*>(children[c]);
            if(concrete_video) {
                matches.push_back(concrete_video);
            }else{
                VideoFilterInterface* filter_video = dynamic_cast<VideoFilterInterface*>(children[c]);
                if(filter_video) {
                    std::vector<T*> child_matches = filter_video->FindMatchingStreams<T>();
                    matches.insert(matches.end(), child_matches.begin(), child_matches.end());
                }
            }
        }
        return matches;
    }

    virtual std::vector<VideoInterface*>& InputStreams() = 0;
};

struct PANGOLIN_EXPORT VideoUvcInterface
{
    virtual ~VideoUvcInterface() {}
    virtual int IoCtrl(uint8_t unit, uint8_t ctrl, unsigned char* data, int len, UvcRequestCode req_code) = 0;
};

struct PANGOLIN_EXPORT VideoPlaybackInterface
{
    virtual ~VideoPlaybackInterface() {}

    /// Return monotonic id of current frame
    virtual int GetCurrentFrameId() const = 0;

    /// Return total number of frames to be captured from device,
    /// or std::numeric_limits<int>::max() on failure.
    virtual int GetTotalFrames() const = 0;

    /// Return -1 on failure, frameid on success
    virtual int Seek(int frameid) = 0;
};

typedef boostd::function<VideoInterface*(const Uri& uri)> VideoInterfaceFactory;

//! Generic wrapper class for different video sources
struct PANGOLIN_EXPORT VideoInput :
    public VideoInterface,
    public VideoFilterInterface
{
    VideoInput();
    VideoInput(const std::string& uri);
    ~VideoInput();
    
    void Open(const std::string& uri);
    void Reset();
    void Close();
    
    size_t SizeBytes() const;
    const std::vector<StreamInfo>& Streams() const;
    
    // Return details of first stream
    unsigned Width() const;
    unsigned Height() const;
    VideoPixelFormat PixFormat() const;
    const Uri& VideoUri() const;
    
    void Start();
    void Stop();
    bool GrabNext( unsigned char* image, bool wait = true );
    bool GrabNewest( unsigned char* image, bool wait = true );

    // Return pointer to inner video class as VideoType
    template<typename VideoType>
    VideoType* Cast() {
        return videos.size() ? dynamic_cast<VideoType*>(videos[0]) : 0;
    }

    // experimental - not stable
    bool Grab( unsigned char* buffer, std::vector<Image<unsigned char> >& images, bool wait = true, bool newest = false);
    
    std::vector<VideoInterface*>& InputStreams();

protected:
    Uri uri;
    std::vector<VideoInterface*> videos;
};

//! Allows the client to register a URI scheme
PANGOLIN_EXPORT
void RegisterVideoScheme(std::string scheme, const VideoInterfaceFactory& factory);

//! Open Video Interface from string specification (as described in this files header)
PANGOLIN_EXPORT
VideoInterface* OpenVideo(const std::string& uri);

//! Open Video Interface from Uri specification
PANGOLIN_EXPORT
VideoInterface* OpenVideo(const Uri& uri);

//! Create vector of matching interfaces either through direct cast or filter interface.
template<typename T>
std::vector<T*> FindMatchingVideoInterfaces( VideoInterface& video )
{
    std::vector<T*> matches;

    T* vid = dynamic_cast<T*>(&video);
    if(vid) {
        matches.push_back(vid);
    }

    VideoFilterInterface* vidf = dynamic_cast<VideoFilterInterface*>(&video);
    if(vidf) {
        std::vector<T*> fmatches = vidf->FindMatchingStreams<T>();
        matches.insert(matches.begin(), fmatches.begin(), fmatches.end());
    }

    return matches;
}

template<typename T>
T* FindFirstMatchingVideoInterface( VideoInterface& video )
{
    T* vid = dynamic_cast<T*>(&video);
    if(vid) {
        return vid;
    }

    VideoFilterInterface* vidf = dynamic_cast<VideoFilterInterface*>(&video);
    if(vidf) {
        std::vector<T*> fmatches = vidf->FindMatchingStreams<T>();
        if(fmatches.size()) {
            return fmatches[0];
        }
    }

    return 0;
}

inline
json::value GetVideoFrameProperties(VideoInterface* video)
{
    VideoPropertiesInterface* pi = dynamic_cast<VideoPropertiesInterface*>(video);
    VideoFilterInterface* fi = dynamic_cast<VideoFilterInterface*>(video);

    if(pi) {
        return pi->FrameProperties();
    }else if(fi){
        if(fi->InputStreams().size() == 1) {
            return GetVideoFrameProperties(fi->InputStreams()[0]);
        }else if(fi->InputStreams().size() > 0){
            // Use first stream's properties as base, but also populate children.
            json::value json = GetVideoFrameProperties(fi->InputStreams()[0]);
            json::value& streams = json["streams"];
            for(size_t i=0; i< fi->InputStreams().size(); ++i) {
                streams.push_back( GetVideoFrameProperties(fi->InputStreams()[i]) );
            }
            return json;
        }
    }
    return json::value();
}

inline
json::value GetVideoDeviceProperties(VideoInterface* video)
{
    VideoPropertiesInterface* pi = dynamic_cast<VideoPropertiesInterface*>(video);
    VideoFilterInterface* fi = dynamic_cast<VideoFilterInterface*>(video);

    if(pi) {
        return pi->DeviceProperties();
    }else if(fi){
        if(fi->InputStreams().size() == 1) {
            return GetVideoDeviceProperties(fi->InputStreams()[0]);
        }else if(fi->InputStreams().size() > 0){
            // Use first stream's properties as base, but also populate children.
            json::value json = GetVideoDeviceProperties(fi->InputStreams()[0]);
            json::value& streams = json["streams"];
            for(size_t i=0; i< fi->InputStreams().size(); ++i) {
                streams.push_back( GetVideoDeviceProperties(fi->InputStreams()[i]) );
            }
            return json;
        }
    }
    return json::value();
}

}

#endif // PANGOLIN_VIDEO_H
