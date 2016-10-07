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

#include <pangolin/video/drivers/debayer.h>

#ifdef HAVE_DC1394
#   include <dc1394/conversions.h>
    const bool have_dc1394 = true;
#else
    const bool have_dc1394 = false;
#endif

namespace pangolin
{

pangolin::StreamInfo BayerOutputFormat( const StreamInfo& stream_in, bayer_method_t method, size_t start_offset)
{
    const bool downsample = (method == BAYER_METHOD_DOWNSAMPLE) || (method == BAYER_METHOD_DOWNSAMPLE_MONO);

    const size_t w = downsample ? stream_in.Width() / 2 : stream_in.Width();
    const size_t h = downsample ? stream_in.Height() / 2 : stream_in.Height();

    pangolin::VideoPixelFormat fmt =
        (method == BAYER_METHOD_NONE) ?
            stream_in.PixFormat() :
            pangolin::VideoFormatFromString(
                (stream_in.PixFormat().bpp == 16) ?
                (method == BAYER_METHOD_DOWNSAMPLE_MONO ? "GRAY16LE" : "RGB48") :
                (method == BAYER_METHOD_DOWNSAMPLE_MONO ? "GRAY8" : "RGB24")
            );

    return pangolin::StreamInfo( fmt, w, h, w*fmt.bpp / 8, (unsigned char*)0 + start_offset );
}

DebayerVideo::DebayerVideo(VideoInterface* src, const std::vector<bayer_method_t>& bayer_method, color_filter_t tile)
    : size_bytes(0), buffer(0), methods(bayer_method), tile(tile)
{
    if(!src) {
        throw VideoException("DebayerVideo: VideoInterface in must not be null");
    }
    videoin.push_back(src);

    while(methods.size() < src->Streams().size()) {
        methods.push_back(BAYER_METHOD_NONE);
    }

    for(size_t s=0; s< src->Streams().size(); ++s) {
        if( (methods[s] < BAYER_METHOD_NONE) && (!have_dc1394 || src->Streams()[0].IsPitched()) ) {
            pango_print_warn("debayer: Switching to simple downsampling method because No DC1394 or image is pitched.\n");
            methods[s] = BAYER_METHOD_DOWNSAMPLE;
        }

        const StreamInfo& stin = src->Streams()[s];
        streams.push_back(BayerOutputFormat(stin, methods[s], size_bytes));
        size_bytes += streams.back().SizeBytes();
    }

    buffer = new unsigned char[src->SizeBytes()];
}

DebayerVideo::~DebayerVideo()
{
    delete[] buffer;
    delete videoin[0];
}

//! Implement VideoInput::Start()
void DebayerVideo::Start()
{
    videoin[0]->Start();
}

//! Implement VideoInput::Stop()
void DebayerVideo::Stop()
{
    videoin[0]->Stop();
}

//! Implement VideoInput::SizeBytes()
size_t DebayerVideo::SizeBytes() const
{
    return size_bytes;
}

//! Implement VideoInput::Streams()
const std::vector<StreamInfo>& DebayerVideo::Streams() const
{
    return streams;
}


unsigned int DebayerVideo::AvailableFrames() const
{
    BufferAwareVideoInterface* vpi = dynamic_cast<BufferAwareVideoInterface*>(videoin[0]);
    if(!vpi)
    {
        pango_print_warn("Debayer: child interface is not buffer aware.");
        return 0;
    }
    else
    {
        return vpi->AvailableFrames();
    }
}

bool DebayerVideo::DropNFrames(uint32_t n)
{
    BufferAwareVideoInterface* vpi = dynamic_cast<BufferAwareVideoInterface*>(videoin[0]);
    if(!vpi)
    {
        pango_print_warn("Debayer: child interface is not buffer aware.");
        return false;
    }
    else
    {
        return vpi->DropNFrames(n);
    }
}

template<typename Tup, typename Tout, typename Tin>
void DownsampleToMono(Image<Tout>& out, const Image<Tin>& in)
{
    for(int y=0; y< (int)out.h; ++y) {
      Tout* pixout = out.RowPtr(y);
      const Tin* irow0 = in.RowPtr(2*y);
      const Tin* irow1 = in.RowPtr(2*y+1);
      for(size_t x=0; x<out.w; ++x) {
          Tup val = ((Tup)irow0[0] + (Tup)irow0[1] + (Tup)irow1[0] + (Tup)irow1[1]) / 4;
          *(pixout++) = (Tout)std::min(std::max(static_cast<Tup>(0), val), static_cast<Tup>(std::numeric_limits<Tout>::max()));
          irow0 += 2;
          irow1 += 2;
      }
    }
}

template<typename Tout, typename Tin>
void DownsampleDebayer(Image<Tout>& out, const Image<Tin>& in, color_filter_t tile)
{
    switch(tile) {
      case DC1394_COLOR_FILTER_RGGB:
        for(int y=0; y< (int)out.h; ++y) {
          Tout* pixout = out.RowPtr(y);
          const Tin* irow0 = in.RowPtr(2*y);
          const Tin* irow1 = in.RowPtr(2*y+1);
          for(size_t x=0; x<out.w; ++x) {
              *(pixout++) = irow0[2*x];
              *(pixout++) = (irow0[2*x+1] + irow1[2*x]) >> 1;
              *(pixout++) = irow1[2*x+1];
          }
        }
        break;
      case DC1394_COLOR_FILTER_GBRG:
        for(int y=0; y< (int)out.h; ++y) {
          Tout* pixout = out.RowPtr(y);
          const Tin* irow0 = in.RowPtr(2*y);
          const Tin* irow1 = in.RowPtr(2*y+1);
          for(size_t x=0; x<out.w; ++x) {
             *(pixout++) = irow1[2*x];
             *(pixout++) = (irow0[2*x] + irow1[2*x+1]) >> 1;
             *(pixout++) = irow0[2*x+1];
          }
        }
        break;
      case DC1394_COLOR_FILTER_GRBG:
        for(int y=0; y< (int)out.h; ++y) {
          Tout* pixout = out.RowPtr(y);
          const Tin* irow0 = in.RowPtr(2*y);
          const Tin* irow1 = in.RowPtr(2*y+1);
          for(size_t x=0; x<out.w; ++x) {
             *(pixout++) = irow0[2*x+1];
             *(pixout++) = (irow0[2*x] + irow1[2*x+1]) >> 1;
             *(pixout++) = irow1[2*x];
          }
        }
        break;
      case DC1394_COLOR_FILTER_BGGR:
        for(int y=0; y< (int)out.h; ++y) {
          Tout* pixout = out.RowPtr(y);
          const Tin* irow0 = in.RowPtr(2*y);
          const Tin* irow1 = in.RowPtr(2*y+1);
          for(size_t x=0; x<out.w; ++x) {
             *(pixout++) = irow1[2*x+1];
             *(pixout++) = (irow0[2*x+1] + irow1[2*x]) >> 1;
             *(pixout++) = irow0[2*x];
          }
        }
        break;
    }
}

template<typename T>
void PitchedImageCopy( Image<T>& img_out, const Image<T>& img_in ) {
    if( img_out.w != img_in.w || img_out.h != img_in.h || sizeof(T) * img_in.w > img_out.pitch) {
        throw std::runtime_error("PitchedImageCopy: Incompatible image sizes");
    }

    for(size_t y=0; y < img_out.h; ++y) {
        std::memcpy(img_out.RowPtr((int)y), img_in.RowPtr((int)y), sizeof(T) * img_in.w);
    }
}

template<typename Tout, typename Tin>
void ProcessImage(Image<Tout>& img_out, const Image<Tin>& img_in, bayer_method_t method, color_filter_t tile)
{
    if(method == BAYER_METHOD_NONE) {
        PitchedImageCopy(img_out, img_in.template Reinterpret<Tout>() );
    }else if(method == BAYER_METHOD_DOWNSAMPLE_MONO) {
        if( sizeof(Tout) == 1) {
            DownsampleToMono<int,Tout, Tin>(img_out, img_in);
        }else{
            DownsampleToMono<double,Tout, Tin>(img_out, img_in);
        }
    }else if(method == BAYER_METHOD_DOWNSAMPLE) {
        DownsampleDebayer(img_out, img_in, tile);
    }else{
#ifdef HAVE_DC1394
        if(sizeof(Tout) == 1) {
            dc1394_bayer_decoding_8bit(
                (uint8_t*)img_in.ptr, (uint8_t*)img_out.ptr, img_in.w, img_in.h,
                (dc1394color_filter_t)tile, (dc1394bayer_method_t)method
            );
        }else if(sizeof(Tout) == 2) {
            dc1394_bayer_decoding_16bit(
                (uint16_t*)img_in.ptr, (uint16_t*)img_out.ptr, img_in.w, img_in.h,
                (dc1394color_filter_t)tile, (dc1394bayer_method_t)method,
                16
            );
        }
#endif
    }
}

void DebayerVideo::ProcessStreams(unsigned char* out, const unsigned char *in)
{
    for(size_t s=0; s<streams.size(); ++s) {
        const StreamInfo& stin = videoin[0]->Streams()[s];

        if(stin.PixFormat().bpp == 8) {
            Image<unsigned char> img_in  = stin.StreamImage(in);
            Image<unsigned char> img_out = Streams()[s].StreamImage(out);
            ProcessImage(img_out, img_in, methods[s], tile);
        }else if(stin.PixFormat().bpp == 16){
            Image<uint16_t> img_in = stin.StreamImage(in).Reinterpret<uint16_t>();
            Image<uint16_t> img_out = Streams()[s].StreamImage(out).Reinterpret<uint16_t>();
            ProcessImage(img_out, img_in, methods[s], tile);
        }else{
            throw std::runtime_error("debayer: unhandled format combination: " + stin.PixFormat().format );
        }
    }
}

//! Implement VideoInput::GrabNext()
bool DebayerVideo::GrabNext( unsigned char* image, bool wait )
{    
    if(videoin[0]->GrabNext(buffer,wait)) {
        ProcessStreams(image, buffer);
        return true;
    }else{
        return false;
    }
}

//! Implement VideoInput::GrabNewest()
bool DebayerVideo::GrabNewest( unsigned char* image, bool wait )
{
    if(videoin[0]->GrabNewest(buffer,wait)) {
        ProcessStreams(image, buffer);
        return true;
    }else{
        return false;
    }
}

std::vector<VideoInterface*>& DebayerVideo::InputStreams()
{
    return videoin;
}

color_filter_t DebayerVideo::ColorFilterFromString(std::string str)
{
  if(!str.compare("rggb") || !str.compare("RGGB")) return DC1394_COLOR_FILTER_RGGB;
  else if(!str.compare("gbrg") || !str.compare("GBRG")) return DC1394_COLOR_FILTER_GBRG;
  else if(!str.compare("grbg") || !str.compare("GRBG")) return DC1394_COLOR_FILTER_GRBG;
  else if(!str.compare("bggr") || !str.compare("BGGR")) return DC1394_COLOR_FILTER_BGGR;
  else {
     pango_print_error("Debayer error, %s is not a valid tile type using RGGB\n", str.c_str());
     return DC1394_COLOR_FILTER_RGGB;
  }
}

bayer_method_t DebayerVideo::BayerMethodFromString(std::string str)
{
  if(!str.compare("nearest")) return BAYER_METHOD_NEAREST;
  else if(!str.compare("simple")) return BAYER_METHOD_SIMPLE;
  else if(!str.compare("bilinear")) return BAYER_METHOD_BILINEAR;
  else if(!str.compare("hqlinear")) return BAYER_METHOD_HQLINEAR;
  else if(!str.compare("downsample")) return BAYER_METHOD_DOWNSAMPLE;
  else if(!str.compare("edgesense")) return BAYER_METHOD_EDGESENSE;
  else if(!str.compare("vng")) return BAYER_METHOD_VNG;
  else if(!str.compare("ahd")) return BAYER_METHOD_AHD;
  else if(!str.compare("mono")) return BAYER_METHOD_DOWNSAMPLE_MONO;
  else if(!str.compare("none")) return BAYER_METHOD_NONE;
  else {
     pango_print_error("Debayer error, %s is not a valid debayer method using downsample\n", str.c_str());
     return BAYER_METHOD_DOWNSAMPLE;
  }
}

}
