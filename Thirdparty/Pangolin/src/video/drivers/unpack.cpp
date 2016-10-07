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

#include <pangolin/video/drivers/unpack.h>

#ifdef DEBUGUNPACK
  #include <pangolin/utils/timer.h>
  #define TSTART() pangolin::basetime start,last,now; start = pangolin::TimeNow(); last = start;
  #define TGRABANDPRINT(...)  now = pangolin::TimeNow(); fprintf(stderr,"UNPACK: "); fprintf(stderr, __VA_ARGS__); fprintf(stderr, " %fms.\n",1000*pangolin::TimeDiff_s(last, now)); last = now;
  #define DBGPRINT(...) fprintf(stderr,"UNPACK: "); fprintf(stderr, __VA_ARGS__); fprintf(stderr,"\n");
#else
  #define TSTART()
  #define TGRABANDPRINT(...)
  #define DBGPRINT(...)
#endif

namespace pangolin
{

UnpackVideo::UnpackVideo(VideoInterface* src, VideoPixelFormat out_fmt)
    : size_bytes(0), buffer(0)
{
    if(!src) {
        throw VideoException("UnpackVideo: VideoInterface in must not be null");
    }

    if( out_fmt.channels != 1) {
        delete src;
        throw VideoException("UnpackVideo: Only supports single channel output.");
    }

    videoin.push_back(src);

    for(size_t s=0; s< src->Streams().size(); ++s) {
        const size_t w = src->Streams()[s].Width();
        const size_t h = src->Streams()[s].Height();

        // Check compatibility of formats
        const VideoPixelFormat in_fmt = src->Streams()[s].PixFormat();
        if(in_fmt.channels > 1 || in_fmt.bpp > 16) {
            delete src;
            throw VideoException("UnpackVideo: Only supports one channel input.");
        }

        const size_t pitch = (w*out_fmt.bpp)/ 8;
        streams.push_back(pangolin::StreamInfo( out_fmt, w, h, pitch, (unsigned char*)0 + size_bytes ));
        size_bytes += h*pitch;
    }

    buffer = new unsigned char[src->SizeBytes()];
}

UnpackVideo::~UnpackVideo()
{
    delete videoin[0];
    delete[] buffer;
}

//! Implement VideoInput::Start()
void UnpackVideo::Start()
{
    videoin[0]->Start();
}

//! Implement VideoInput::Stop()
void UnpackVideo::Stop()
{
    videoin[0]->Stop();
}

//! Implement VideoInput::SizeBytes()
size_t UnpackVideo::SizeBytes() const
{
    return size_bytes;
}

//! Implement VideoInput::Streams()
const std::vector<StreamInfo>& UnpackVideo::Streams() const
{
    return streams;
}

template<typename T>
void ConvertFrom8bit(
    Image<unsigned char>& out,
    const Image<unsigned char>& in
) {
    for(size_t r=0; r<out.h; ++r) {
        T* pout = (T*)(out.ptr + r*out.pitch);
        uint8_t* pin = in.ptr + r*in.pitch;
        const uint8_t* pin_end = in.ptr + (r+1)*in.pitch;
        while(pin != pin_end) {
            *(pout++) = *(pin++);
        }
    }
}

template<typename T>
void ConvertFrom10bit(
    Image<unsigned char>& out,
    const Image<unsigned char>& in
) {
    for(size_t r=0; r<out.h; ++r) {
        T* pout = (T*)(out.ptr + r*out.pitch);
        uint8_t* pin = in.ptr + r*in.pitch;
        const uint8_t* pin_end = in.ptr + (r+1)*in.pitch;
        while(pin != pin_end) {
            uint64_t val = *(pin++);
            val |= uint64_t(*(pin++)) << 8;
            val |= uint64_t(*(pin++)) << 16;
            val |= uint64_t(*(pin++)) << 24;
            val |= uint64_t(*(pin++)) << 32;
            *(pout++) = T( val & 0x00000003FF);
            *(pout++) = T((val & 0x00000FFC00) >> 10);
            *(pout++) = T((val & 0x003FF00000) >> 20);
            *(pout++) = T((val & 0xFFC0000000) >> 30);
        }
    }
}

template<typename T>
void ConvertFrom12bit(
    Image<unsigned char>& out,
    const Image<unsigned char>& in
) {
    for(size_t r=0; r<out.h; ++r) {
        T* pout = (T*)(out.ptr + r*out.pitch);
        uint8_t* pin = in.ptr + r*in.pitch;
        const uint8_t* pin_end = in.ptr + (r+1)*in.pitch;
        while(pin != pin_end) {
            uint32_t val = *(pin++);
            val |= uint32_t(*(pin++)) << 8;
            val |= uint32_t(*(pin++)) << 16;
            *(pout++) = T( val & 0x000FFF);
            *(pout++) = T((val & 0xFFF000) >> 12);
        }
    }
}

void UnpackVideo::Process(unsigned char* image, const unsigned char* buffer)
{
    TSTART()
    for(size_t s=0; s<streams.size(); ++s) {
        const Image<unsigned char> img_in  = videoin[0]->Streams()[s].StreamImage(buffer);
        Image<unsigned char> img_out = Streams()[s].StreamImage(image);

        const int bits_in  = videoin[0]->Streams()[s].PixFormat().bpp;

        if(Streams()[s].PixFormat().format == "GRAY32F") {
            if( bits_in == 8) {
                ConvertFrom8bit<float>(img_out, img_in);
            }else if( bits_in == 10) {
                ConvertFrom10bit<float>(img_out, img_in);
            }else if( bits_in == 12){
                ConvertFrom12bit<float>(img_out, img_in);
            }else{
                throw pangolin::VideoException("Unsupported bitdepths.");
            }
        }else if(Streams()[s].PixFormat().format == "GRAY16LE") {
            if( bits_in == 8) {
                ConvertFrom8bit<uint16_t>(img_out, img_in);
            }else if( bits_in == 10) {
                ConvertFrom10bit<uint16_t>(img_out, img_in);
            }else if( bits_in == 12){
                ConvertFrom12bit<uint16_t>(img_out, img_in);
            }else{
                throw pangolin::VideoException("Unsupported bitdepths.");
            }
        }else{
        }
    }
    TGRABANDPRINT("Unpacking took ")
}

//! Implement VideoInput::GrabNext()
bool UnpackVideo::GrabNext( unsigned char* image, bool wait )
{    
    if(videoin[0]->GrabNext(buffer,wait)) {
        Process(image,buffer);
        return true;
    }else{
        return false;
    }
}

//! Implement VideoInput::GrabNewest()
bool UnpackVideo::GrabNewest( unsigned char* image, bool wait )
{
    if(videoin[0]->GrabNewest(buffer,wait)) {
        Process(image,buffer);
        return true;
    }else{
        return false;
    }
}

std::vector<VideoInterface*>& UnpackVideo::InputStreams()
{
    return videoin;
}

unsigned int UnpackVideo::AvailableFrames() const
{
    BufferAwareVideoInterface* vpi = dynamic_cast<BufferAwareVideoInterface*>(videoin[0]);
    if(!vpi)
    {
        pango_print_warn("Unpack: child interface is not buffer aware.");
        return 0;
    }
    else
    {
        return vpi->AvailableFrames();
    }
}

bool UnpackVideo::DropNFrames(uint32_t n)
{
    BufferAwareVideoInterface* vpi = dynamic_cast<BufferAwareVideoInterface*>(videoin[0]);
    if(!vpi)
    {
        pango_print_warn("Unpack: child interface is not buffer aware.");
        return false;
    }
    else
    {
        return vpi->DropNFrames(n);
    }
}

}

#undef TSTART
#undef TGRABANDPRINT
#undef DBGPRINT
