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

#include <pangolin/video/drivers/firewire.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>

using namespace std;

namespace pangolin
{

void FirewireVideo::init_camera(
        uint64_t guid, int dma_frames,
        dc1394speed_t iso_speed,
        dc1394video_mode_t video_mode,
        dc1394framerate_t framerate
        ) {
    
    if(video_mode>=DC1394_VIDEO_MODE_FORMAT7_0)
        throw VideoException("format7 modes need to be initialized through the constructor that allows for specifying the roi");
    
    camera = dc1394_camera_new (d, guid);
    if (!camera)
        throw VideoException("Failed to initialize camera");
    
    // Attempt to stop camera if it is already running
    dc1394switch_t is_iso_on = DC1394_OFF;
    dc1394_video_get_transmission(camera, &is_iso_on);
    if (is_iso_on==DC1394_ON) {
        dc1394_video_set_transmission(camera, DC1394_OFF);
    }
    
    
    cout << "Using camera with GUID " << camera->guid << endl;
    
    //-----------------------------------------------------------------------
    //  setup capture
    //-----------------------------------------------------------------------
    
    if( iso_speed >= DC1394_ISO_SPEED_800)
    {
        err=dc1394_video_set_operation_mode(camera, DC1394_OPERATION_MODE_1394B);
        if( err != DC1394_SUCCESS )
            throw VideoException("Could not set DC1394_OPERATION_MODE_1394B");
    }
    
    err=dc1394_video_set_iso_speed(camera, iso_speed);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not set iso speed");
    
    err=dc1394_video_set_mode(camera, video_mode);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not set video mode");
    
    err=dc1394_video_set_framerate(camera, framerate);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not set framerate");
    
    err=dc1394_capture_setup(camera,dma_frames, DC1394_CAPTURE_FLAGS_DEFAULT);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not setup camera - check settings");
    
    //-----------------------------------------------------------------------
    //  initialise width and height from mode
    //-----------------------------------------------------------------------
    dc1394_get_image_size_from_video_mode(camera, video_mode, &width, &height);
    
    init_stream_info();
    Start();    
}


// Note:
// the following was tested on a IIDC camera over USB therefore might not work as
// well on a camera over proper firewire transport
void FirewireVideo::init_format7_camera(
        uint64_t guid, int dma_frames,
        dc1394speed_t iso_speed,
        dc1394video_mode_t video_mode,
        float framerate,
        uint32_t width, uint32_t height,
        uint32_t left, uint32_t top, bool reset_at_boot
        ) {
    
    if(video_mode< DC1394_VIDEO_MODE_FORMAT7_0)
        throw VideoException("roi can be specified only for format7 modes");
    
    camera = dc1394_camera_new (d, guid);
    if (!camera)
        throw VideoException("Failed to initialize camera");
    
    // Attempt to stop camera if it is already running
    dc1394switch_t is_iso_on = DC1394_OFF;
    dc1394_video_get_transmission(camera, &is_iso_on);
    if (is_iso_on==DC1394_ON) {
        dc1394_video_set_transmission(camera, DC1394_OFF);
    }
    
    cout << "Using camera with GUID " << camera->guid << endl;
    
    if(reset_at_boot){
        dc1394_camera_reset(camera);
    }
    
    //-----------------------------------------------------------------------
    //  setup mode and roi
    //-----------------------------------------------------------------------
    
    if(iso_speed >= DC1394_ISO_SPEED_800)
    {
        err=dc1394_video_set_operation_mode(camera, DC1394_OPERATION_MODE_1394B);
        if( err != DC1394_SUCCESS )
            throw VideoException("Could not set DC1394_OPERATION_MODE_1394B");
    }
    
    err=dc1394_video_set_iso_speed(camera, iso_speed);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not set iso speed");
    
    // check that the required mode is actually supported
    dc1394format7mode_t format7_info;
    
    err = dc1394_format7_get_mode_info(camera, video_mode, &format7_info);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not get format7 mode info");
    
    // safely set the video mode
    err=dc1394_video_set_mode(camera, video_mode);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not set format7 video mode");
    
    // set position to 0,0 so that setting any size within min and max is a valid command
    err = dc1394_format7_set_image_position(camera, video_mode,0,0);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not set format7 image position");
    
    // work out the desired image size
    width = nearest_value(width, format7_info.unit_pos_x, 0, format7_info.max_size_x - left);
    height = nearest_value(height, format7_info.unit_pos_y, 0, format7_info.max_size_y - top);
    
    // set size
    err = dc1394_format7_set_image_size(camera,video_mode,width,height);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not set format7 size");
    
    // get the info again since many parameters depend on image size
    err = dc1394_format7_get_mode_info(camera, video_mode, &format7_info);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not get format7 mode info");
    
    // work out position of roi
    left = nearest_value(left, format7_info.unit_size_x, format7_info.unit_size_x, format7_info.max_size_x - width);
    top = nearest_value(top, format7_info.unit_size_y, format7_info.unit_size_y, format7_info.max_size_y - height);
    
    // set roi position
    err = dc1394_format7_set_image_position(camera,video_mode,left,top);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not set format7 size");
    
    this->width = width;
    this->height = height;
    this->top = top;
    this->left = left;
    
    cout<<"roi: "<<left<<" "<<top<<" "<<width<<" "<<height<<"  ";
    
    
    //-----------------------------------------------------------------------
    //  setup frame rate
    //-----------------------------------------------------------------------
    
    err=dc1394_format7_set_packet_size(camera,video_mode, format7_info.max_packet_size);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not set format7 packet size");
    
    if((framerate != MAX_FR) && (framerate != EXT_TRIG)){
        //set the framerate by using the absolute feature as suggested by the
        //folks at PointGrey
        err = dc1394_feature_set_absolute_control(camera,DC1394_FEATURE_FRAME_RATE,DC1394_ON);
        if( err != DC1394_SUCCESS )
            throw VideoException("Could not turn on absolute frame rate control");
        
        err = dc1394_feature_set_mode(camera,DC1394_FEATURE_FRAME_RATE,DC1394_FEATURE_MODE_MANUAL);
        if( err != DC1394_SUCCESS )
            throw VideoException("Could not make frame rate manual ");
        
        err=dc1394_feature_set_absolute_value(camera,DC1394_FEATURE_FRAME_RATE,framerate);
        if( err != DC1394_SUCCESS )
            throw VideoException("Could not set format7 framerate ");
    }
    
    // ask the camera what is the resulting framerate (this assume that such a rate is actually
    // allowed by the shutter time)
    float value;
    err=dc1394_feature_get_absolute_value(camera,DC1394_FEATURE_FRAME_RATE,&value);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not get framerate");
    
    cout<<" framerate(shutter permitting):"<<value<<endl;
    
    //-----------------------------------------------------------------------
    //  setup capture
    //-----------------------------------------------------------------------
    
    err=dc1394_capture_setup(camera,dma_frames, DC1394_CAPTURE_FLAGS_DEFAULT);
    if( err != DC1394_SUCCESS )
        throw VideoException("Could not setup camera - check settings");
    
    init_stream_info();
    Start();
}

std::string Dc1394ColorCodingToString(dc1394color_coding_t coding)
{
    switch(coding)
    {
    case DC1394_COLOR_CODING_RGB8 :    return "RGB24";
    case DC1394_COLOR_CODING_MONO8 :   return "GRAY8";
        
    case DC1394_COLOR_CODING_MONO16 :  return "GRAY16LE";
    case DC1394_COLOR_CODING_RGB16 :   return "RGB48LE";
        
    case DC1394_COLOR_CODING_MONO16S : return "GRAY16BE";
    case DC1394_COLOR_CODING_RGB16S :  return "RGB48BE";
        
    case DC1394_COLOR_CODING_YUV411 :  return "YUV411P";
    case DC1394_COLOR_CODING_YUV422 :  return "YUV422P";
    case DC1394_COLOR_CODING_YUV444 :  return "YUV444P";
        
    case DC1394_COLOR_CODING_RAW8 :    return "GRAY8";
    case DC1394_COLOR_CODING_RAW16 :   return "GRAY16LE";
        
    default:
        throw VideoException("Unknown colour coding");
    }
}

dc1394color_coding_t Dc1394ColorCodingFromString(std::string coding)
{
    if(     !coding.compare("RGB24"))    return DC1394_COLOR_CODING_RGB8;
    else if(!coding.compare("GRAY8"))    return DC1394_COLOR_CODING_MONO8;
    
    else if(!coding.compare("GRAY16LE")) return DC1394_COLOR_CODING_MONO16;
    else if(!coding.compare("RGB48LE"))  return DC1394_COLOR_CODING_RGB16;
    else if(!coding.compare("GRAY16BE")) return DC1394_COLOR_CODING_MONO16S;
    else if(!coding.compare("RGB48BE"))  return DC1394_COLOR_CODING_RGB16S;
    
    else if(!coding.compare("YUV411P"))  return DC1394_COLOR_CODING_YUV411;
    else if(!coding.compare("YUV422P"))  return DC1394_COLOR_CODING_YUV422;
    else if(!coding.compare("YUV444P"))  return DC1394_COLOR_CODING_YUV444;
    //    else if(!coding.compare("RAW8"))     return DC1394_COLOR_CODING_RAW8;
    //    else if(!coding.compare("RAW16"))    return DC1394_COLOR_CODING_RAW16;
    throw VideoException("Unknown colour coding");
}

void Dc1394ModeDetails(dc1394video_mode_t mode, unsigned& w, unsigned& h, string& format )
{
    switch( mode )
    {
    // RGB Modes
    case DC1394_VIDEO_MODE_1024x768_RGB8:
        w=1024;    h=768;    format = "RGB24";
        break;
    case DC1394_VIDEO_MODE_640x480_RGB8:
        w=640;    h=480;    format = "RGB24";
        break;
    case DC1394_VIDEO_MODE_800x600_RGB8:
        w=800;    h=600;    format = "RGB24";
        break;
    case DC1394_VIDEO_MODE_1280x960_RGB8:
        w=1280;    h=960;    format = "RGB24";
        break;
    case DC1394_VIDEO_MODE_1600x1200_RGB8:
        w=1600;    h=1200;    format = "RGB24";
        break;
        
        // Greyscale modes
    case DC1394_VIDEO_MODE_640x480_MONO8:
        w=640;    h=480;    format = "GRAY8";
        break;
    case DC1394_VIDEO_MODE_800x600_MONO8:
        w=800;    h=600;    format = "GRAY8";
        break;
    case DC1394_VIDEO_MODE_1024x768_MONO8:
        w=1024;    h=768;    format = "GRAY8";
        break;
    case DC1394_VIDEO_MODE_1280x960_MONO8:
        w=1280;    h=960;    format = "GRAY8";
        break;
    case DC1394_VIDEO_MODE_1600x1200_MONO8:
        w=1600;    h=1200;    format = "GRAY8";
        break;
    case DC1394_VIDEO_MODE_640x480_MONO16:
        w=640;    h=480;    format = "GRAY16";
        break;
    case DC1394_VIDEO_MODE_800x600_MONO16:
        w=800;    h=600;    format = "GRAY16";
        break;
    case DC1394_VIDEO_MODE_1024x768_MONO16:
        w=1024;    h=768;    format = "GRAY16";
        break;
    case DC1394_VIDEO_MODE_1280x960_MONO16:
        w=1280;    h=960;    format = "GRAY16";
        break;
    case DC1394_VIDEO_MODE_1600x1200_MONO16:
        w=1600;    h=1200;    format = "GRAY16";
        break;
        
        // Chrome modes
    case DC1394_VIDEO_MODE_640x480_YUV411:
        w=640;    h=480;    format = "YUV411P";
        break;
    case DC1394_VIDEO_MODE_160x120_YUV444:
        w=160;    h=120;    format = "YUV444P";
        break;
    case DC1394_VIDEO_MODE_320x240_YUV422:
        w=320;    h=240;    format = "YUV422P";
        break;
    case DC1394_VIDEO_MODE_640x480_YUV422:
        w=640;    h=480;    format = "YUV422P";
        break;
    case DC1394_VIDEO_MODE_800x600_YUV422:
        w=800;    h=600;    format = "YUV422P";
        break;
    case DC1394_VIDEO_MODE_1024x768_YUV422:
        w=1024;    h=768;    format = "YUV422P";
        break;
    case DC1394_VIDEO_MODE_1600x1200_YUV422:
        w=1600;    h=1200;    format = "YUV422P";
        break;
    case DC1394_VIDEO_MODE_1280x960_YUV422:
        w=1280;    h=960;    format = "YUV422P";
        break;
    default:
        throw VideoException("Unknown colour coding");
    }
}

void FirewireVideo::init_stream_info()
{
    streams.clear();
    
    dc1394video_mode_t video_mode;
    dc1394color_coding_t color_coding;
    dc1394_video_get_mode(camera,&video_mode);
    dc1394_get_color_coding_from_video_mode(camera,video_mode,&color_coding);
    const std::string strformat = Dc1394ColorCodingToString(color_coding);
    const VideoPixelFormat fmt = VideoFormatFromString(strformat);
    
    StreamInfo stream(fmt, width, height, (width*fmt.bpp)/8, 0 );
    streams.push_back( stream );
    
    frame_size_bytes = stream.Pitch() * stream.Height();
}

const std::vector<StreamInfo>& FirewireVideo::Streams() const
{
    return streams;
}

size_t FirewireVideo::SizeBytes() const
{
    return frame_size_bytes;
}

void FirewireVideo::Start()
{
    if( !running )
    {
        err=dc1394_video_set_transmission(camera, DC1394_ON);
        if( err != DC1394_SUCCESS )
            throw VideoException("Could not start camera iso transmission");
        running = true;
    }
}

void FirewireVideo::Stop()
{
    if( running )
    {
        // Stop transmission
        err=dc1394_video_set_transmission(camera,DC1394_OFF);
        if( err != DC1394_SUCCESS )
            throw VideoException("Could not stop the camera");
        running = false;
    }
}

FirewireVideo::FirewireVideo(
        Guid guid,
        dc1394video_mode_t video_mode,
        dc1394framerate_t framerate,
        dc1394speed_t iso_speed,
        int dma_buffers
        ) :running(false),top(0),left(0)
{
    d = dc1394_new ();
    if (!d)
        throw VideoException("Failed to get 1394 bus");
    
    init_camera(guid.guid,dma_buffers,iso_speed,video_mode,framerate);
}

FirewireVideo::FirewireVideo(
        Guid guid,
        dc1394video_mode_t video_mode,
        float framerate,
        uint32_t width, uint32_t height,
        uint32_t left, uint32_t top,
        dc1394speed_t iso_speed,
        int dma_buffers, bool reset_at_boot
        ) :running(false)
{
    d = dc1394_new ();
    if (!d)
        throw VideoException("Failed to get 1394 bus");
    
    init_format7_camera(guid.guid,dma_buffers,iso_speed,video_mode,framerate,width,height,left,top, reset_at_boot);
}

FirewireVideo::FirewireVideo(
        unsigned deviceid,
        dc1394video_mode_t video_mode,
        dc1394framerate_t framerate,
        dc1394speed_t iso_speed,
        int dma_buffers
        ) :running(false),top(0),left(0)
{
    d = dc1394_new ();
    if (!d)
        throw VideoException("Failed to get 1394 bus");
    
    err=dc1394_camera_enumerate (d, &list);
    if( err != DC1394_SUCCESS )
        throw VideoException("Failed to enumerate cameras");
    
    if (list->num == 0)
        throw VideoException("No cameras found");
    
    if( deviceid >= list->num )
        throw VideoException("Invalid camera index");
    
    const uint64_t guid = list->ids[deviceid].guid;
    
    dc1394_camera_free_list (list);
    
    init_camera(guid,dma_buffers,iso_speed,video_mode,framerate);
    
}

FirewireVideo::FirewireVideo(
        unsigned deviceid,
        dc1394video_mode_t video_mode,
        float framerate,
        uint32_t width, uint32_t height,
        uint32_t left, uint32_t top,
        dc1394speed_t iso_speed,
        int dma_buffers, bool reset_at_boot
        ) :running(false)
{
    d = dc1394_new ();
    if (!d)
        throw VideoException("Failed to get 1394 bus");
    
    err=dc1394_camera_enumerate (d, &list);
    if( err != DC1394_SUCCESS )
        throw VideoException("Failed to enumerate cameras");
    
    if (list->num == 0)
        throw VideoException("No cameras found");
    
    if( deviceid >= list->num )
        throw VideoException("Invalid camera index");
    
    const uint64_t guid = list->ids[deviceid].guid;
    
    dc1394_camera_free_list (list);
    
    init_format7_camera(guid,dma_buffers,iso_speed,video_mode,framerate,width,height,left,top, reset_at_boot);
    
}

bool FirewireVideo::GrabNext( unsigned char* image, bool wait )
{
    const dc1394capture_policy_t policy =
            wait ? DC1394_CAPTURE_POLICY_WAIT : DC1394_CAPTURE_POLICY_POLL;
    
    dc1394video_frame_t *frame;
    err = dc1394_capture_dequeue(camera, policy, &frame);
    if( err != DC1394_SUCCESS)
        throw VideoException("Could not capture frame", dc1394_error_get_string(err) );
    
    if( frame )
    {
        memcpy(image,frame->image,frame->image_bytes);
        dc1394_capture_enqueue(camera,frame);
        return true;
    }
    return false;
}

bool FirewireVideo::GrabNewest( unsigned char* image, bool wait )
{
    dc1394video_frame_t *f;
    err = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_POLL, &f);
    if( err != DC1394_SUCCESS)
        throw VideoException("Could not capture frame", dc1394_error_get_string(err) );
    
    if( f ) {
        while( true )
        {
            dc1394video_frame_t *nf;
            err = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_POLL, &nf);
            if( err != DC1394_SUCCESS)
                throw VideoException("Could not capture frame", dc1394_error_get_string(err) );
            
            if( nf )
            {
                err=dc1394_capture_enqueue(camera,f);
                f = nf;
            }else{
                break;
            }
        }
        memcpy(image,f->image,f->image_bytes);
        err=dc1394_capture_enqueue(camera,f);
        return true;
    }else if(wait){
        return GrabNext(image,true);
    }
    return false;
}

FirewireFrame FirewireVideo::GetNext(bool wait)
{
    const dc1394capture_policy_t policy =
            wait ? DC1394_CAPTURE_POLICY_WAIT : DC1394_CAPTURE_POLICY_POLL;
    
    dc1394video_frame_t *frame;
    dc1394_capture_dequeue(camera, policy, &frame);
    return FirewireFrame(frame);
}

FirewireFrame FirewireVideo::GetNewest(bool wait)
{
    dc1394video_frame_t *f;
    err = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_POLL, &f);
    if( err != DC1394_SUCCESS)
        throw VideoException("Could not capture frame", dc1394_error_get_string(err) );
    
    if( f ) {
        while( true )
        {
            dc1394video_frame_t *nf;
            err = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_POLL, &nf);
            if( err != DC1394_SUCCESS)
                throw VideoException("Could not capture frame", dc1394_error_get_string(err) );
            
            if( nf )
            {
                err=dc1394_capture_enqueue(camera,f);
                f = nf;
            }else{
                break;
            }
        }
        return FirewireFrame(f);
    }else if(wait){
        return GetNext(true);
    }
    return FirewireFrame(0);
}

void FirewireVideo::PutFrame(FirewireFrame& f)
{
    if( f.frame )
    {
        dc1394_capture_enqueue(camera,f.frame);
        f.frame = 0;
    }
}

float FirewireVideo::GetGain() const
{
    float gain;
    err = dc1394_feature_get_absolute_value(camera,DC1394_FEATURE_GAIN,&gain);
    if( err != DC1394_SUCCESS )
        throw VideoException("Failed to read gain");
    
    return gain;
    
}

void FirewireVideo::SetAutoGain()
{
    
    dc1394error_t err = dc1394_feature_set_mode(camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO);
    if (err < 0) {
        throw VideoException("Could not set auto gain mode");
    }
}

void FirewireVideo::SetGain(float val)
{
    dc1394error_t err = dc1394_feature_set_mode(camera, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);
    if (err < 0) {
        throw VideoException("Could not set manual gain mode");
    }
    
    err = dc1394_feature_set_absolute_control(camera, DC1394_FEATURE_GAIN, DC1394_ON);
    if (err < 0) {
        throw VideoException("Could not set absolute control for gain");
    }
    
    err = dc1394_feature_set_absolute_value(camera, DC1394_FEATURE_GAIN, val);
    if (err < 0) {
        throw VideoException("Could not set gain value");
    }
}


float FirewireVideo::GetBrightness() const
{
    float brightness;
    err = dc1394_feature_get_absolute_value(camera,DC1394_FEATURE_BRIGHTNESS,&brightness);
    if( err != DC1394_SUCCESS )
        throw VideoException("Failed to read brightness");
    
    return brightness;
    
}

void FirewireVideo::SetAutoBrightness()
{
    dc1394error_t err = dc1394_feature_set_mode(camera, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_AUTO);
    if (err < 0) {
        throw VideoException("Could not set auto brightness mode");
    }
}

void FirewireVideo::SetBrightness(float val)
{
    dc1394error_t err = dc1394_feature_set_mode(camera, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL);
    if (err < 0) {
        throw VideoException("Could not set manual brightness mode");
    }
    
    err = dc1394_feature_set_absolute_control(camera, DC1394_FEATURE_BRIGHTNESS, DC1394_ON);
    if (err < 0) {
        throw VideoException("Could not set absolute control for brightness");
    }
    
    err = dc1394_feature_set_absolute_value(camera, DC1394_FEATURE_BRIGHTNESS, val);
    if (err < 0) {
        throw VideoException("Could not set brightness value");
    }
}

float FirewireVideo::GetShutterTime() const
{
    float shutter;
    err = dc1394_feature_get_absolute_value(camera,DC1394_FEATURE_SHUTTER,&shutter);
    if( err != DC1394_SUCCESS )
        throw VideoException("Failed to read shutter");
    
    return shutter;
}

void FirewireVideo::SetAutoShutterTime()
{
    dc1394error_t err = dc1394_feature_set_mode(camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO);
    if (err < 0) {
        throw VideoException("Could not set auto shutter mode");
    }
}

void FirewireVideo::SetShutterTime(float val)
{
    dc1394error_t err = dc1394_feature_set_mode(camera, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
    if (err < 0) {
        throw VideoException("Could not set manual shutter mode");
    }
    
    err = dc1394_feature_set_absolute_control(camera, DC1394_FEATURE_SHUTTER, DC1394_ON);
    if (err < 0) {
        throw VideoException("Could not set absolute control for shutter");
    }
    
    err = dc1394_feature_set_absolute_value(camera, DC1394_FEATURE_SHUTTER, val);
    if (err < 0) {
        throw VideoException("Could not set shutter value");
    }
}

void FirewireVideo::SetShutterTimeQuant(int shutter)
{
    // TODO: Set mode as well
    
    err = dc1394_feature_set_value(camera,DC1394_FEATURE_SHUTTER,shutter);
    
    if( err != DC1394_SUCCESS )
        throw VideoException("Failed to set shutter");
}

float FirewireVideo::GetGamma() const
{
    float gamma;
    err = dc1394_feature_get_absolute_value(camera,DC1394_FEATURE_GAMMA,&gamma);
    if( err != DC1394_SUCCESS )
        throw VideoException("Failed to read gamma");
    return gamma;
}

void FirewireVideo::SetInternalTrigger() 
{
    dc1394error_t err = dc1394_external_trigger_set_power(camera, DC1394_OFF);
    if (err < 0) {
        throw VideoException("Could not set internal trigger mode");
    }
}

void FirewireVideo::SetExternalTrigger(dc1394trigger_mode_t mode, dc1394trigger_polarity_t polarity, dc1394trigger_source_t source)
{
    dc1394error_t err = dc1394_external_trigger_set_polarity(camera, polarity);
    if (err < 0) {
        throw VideoException("Could not set external trigger polarity");
    }
    
    err = dc1394_external_trigger_set_mode(camera, mode);
    if (err < 0) {
        throw VideoException("Could not set external trigger mode");
    }
    
    err = dc1394_external_trigger_set_source(camera, source);
    if (err < 0) {
        throw VideoException("Could not set external trigger source");
    }
    
    err = dc1394_external_trigger_set_power(camera, DC1394_ON);
    if (err < 0) {
        throw VideoException("Could not set external trigger power");
    }
}


FirewireVideo::~FirewireVideo()
{
    Stop();
    
    // Close camera
    dc1394_video_set_transmission(camera, DC1394_OFF);
    dc1394_capture_stop(camera);
    dc1394_camera_free(camera);
    dc1394_free (d);
}

void FirewireVideo::SetRegister(uint64_t offset, uint32_t value){
    dc1394error_t err = dc1394_set_register (camera, offset, value);
    if (err < 0) {
        throw VideoException("Could not set camera register");
    }
}

uint32_t FirewireVideo::GetRegister(uint64_t offset)
{
    uint32_t value = 0;
    dc1394error_t err = dc1394_get_register (camera, offset, &value);
    if (err < 0) {
        throw VideoException("Could not get camera register");
    }
    return value;
}

void FirewireVideo::SetControlRegister(uint64_t offset, uint32_t value)
{
    dc1394error_t err = dc1394_set_control_register (camera, offset, value);
    if (err < 0) {
        throw VideoException("Could not set camera control register");
    }
}

uint32_t FirewireVideo::GetControlRegister(uint64_t offset)
{
    uint32_t value = 0;
    dc1394error_t err = dc1394_get_control_register(camera, offset, &value);
    if (err < 0) {
        throw VideoException("Could not get camera control register");
    }
    return value;
}

int FirewireVideo::nearest_value(int value, int step, int min, int max)
{
    int low, high;
    
    low=value-(value%step);
    high=value-(value%step)+step;
    if (low<min)
        low=min;
    if (high>max)
        high=max;
    
    if (abs(low-value)<abs(high-value))
        return low;
    else
        return high;
}

double FirewireVideo::bus_period_from_iso_speed(dc1394speed_t iso_speed)
{
    double bus_period;
    
    switch(iso_speed){
    case DC1394_ISO_SPEED_3200:
        bus_period = 15.625e-6;
        break;
    case DC1394_ISO_SPEED_1600:
        bus_period = 31.25e-6;
        break;
    case DC1394_ISO_SPEED_800:
        bus_period = 62.5e-6;
        break;
    case DC1394_ISO_SPEED_400:
        bus_period = 125e-6;
        break;
    case DC1394_ISO_SPEED_200:
        bus_period = 250e-6;
        break;
    case DC1394_ISO_SPEED_100:
        bus_period = 500e-6;
        break;
    default:
        throw VideoException("iso speed not valid");
    }
    
    return bus_period;
}

}

