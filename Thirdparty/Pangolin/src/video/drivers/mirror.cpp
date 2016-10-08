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

#include <pangolin/video/drivers/mirror.h>

namespace pangolin
{

MirrorVideo::MirrorVideo(VideoInterface* src, const std::vector<MirrorOptions>& flips)
    : videoin(src), flips(flips), size_bytes(0),buffer(0)
{
    if(!src) {
        throw VideoException("MirrorVideo: VideoInterface in must not be null");
    }

    inputs.push_back(videoin);

    streams = src->Streams();
    size_bytes = src->SizeBytes();
    buffer = new unsigned char[size_bytes];
}

MirrorVideo::~MirrorVideo()
{
    delete[] buffer;
    delete videoin;
}

//! Implement VideoInput::Start()
void MirrorVideo::Start()
{
    videoin->Start();
}

//! Implement VideoInput::Stop()
void MirrorVideo::Stop()
{
    videoin->Stop();
}

//! Implement VideoInput::SizeBytes()
size_t MirrorVideo::SizeBytes() const
{
    return size_bytes;
}

//! Implement VideoInput::Streams()
const std::vector<StreamInfo>& MirrorVideo::Streams() const
{
    return streams;
}

void PitchedImageCopy(
    Image<unsigned char>& img_out,
    const Image<unsigned char>& img_in,
    size_t bytes_per_pixel
) {
    if( img_out.w != img_in.w || img_out.h != img_in.h ) {
        throw std::runtime_error("PitchedImageCopy: Incompatible image sizes");
    }

    for(size_t y=0; y < img_out.h; ++y) {
        std::memcpy(img_out.RowPtr((int)y), img_in.RowPtr((int)y), bytes_per_pixel * img_in.w);
    }
}

void FlipY(
    Image<unsigned char>& img_out,
    const Image<unsigned char>& img_in,
    size_t bytes_per_pixel
) {
    if( img_out.w != img_in.w || img_out.h != img_in.h ) {
        throw std::runtime_error("FlipY: Incompatible image sizes");
    }

    for(size_t y_out=0; y_out < img_out.h; ++y_out) {
        const size_t y_in = (img_in.h-1) - y_out;
        std::memcpy(img_out.RowPtr((int)y_out), img_in.RowPtr((int)y_in), bytes_per_pixel * img_in.w);
    }
}

void FlipX(
    Image<unsigned char>& img_out,
    const Image<unsigned char>& img_in,
    size_t bytes_per_pixel
) {
    for(size_t y=0; y < img_out.h; ++y) {
        for(size_t x=0; x < img_out.w; ++x) {
            memcpy(
                img_out.RowPtr((int)y) + (img_out.w-1-x)*bytes_per_pixel,
                img_in.RowPtr((int)y)  + x*bytes_per_pixel,
                bytes_per_pixel
            );
        }
    }
}

void FlipXY(
    Image<unsigned char>& img_out,
    const Image<unsigned char>& img_in,
    size_t bytes_per_pixel
) {
    for(size_t y_out=0; y_out < img_out.h; ++y_out) {
        for(size_t x=0; x < img_out.w; ++x) {
            const size_t y_in = (img_in.h-1) - y_out;
            memcpy(
                img_out.RowPtr((int)y_out) + (img_out.w-1-x)*bytes_per_pixel,
                img_in.RowPtr((int)y_in)   + x*bytes_per_pixel,
                bytes_per_pixel
            );
        }
    }
}

void MirrorVideo::Process(unsigned char* buffer_out, const unsigned char* buffer_in)
{
    for(size_t s=0; s<streams.size(); ++s) {
        Image<unsigned char> img_out = Streams()[s].StreamImage(buffer_out);
        const Image<unsigned char> img_in  = videoin[0].Streams()[s].StreamImage(buffer_in);
        const size_t bytes_per_pixel = Streams()[s].PixFormat().bpp / 8;

        switch (flips[s]) {
        case MirrorOptionsFlipX:
            FlipX(img_out, img_in, bytes_per_pixel);
            break;
        case MirrorOptionsFlipY:
            FlipY(img_out, img_in, bytes_per_pixel);
            break;
        case MirrorOptionsFlipXY:
            FlipXY(img_out, img_in, bytes_per_pixel);
            break;
        default:
            pango_print_warn("MirrorVideo::Process(): Invalid enum %i.\n", flips[s]);
        case MirrorOptionsNone:
            PitchedImageCopy(img_out, img_in, bytes_per_pixel);
            break;
        }
    }
}

//! Implement VideoInput::GrabNext()
bool MirrorVideo::GrabNext( unsigned char* image, bool wait )
{    
    if(videoin->GrabNext(buffer,wait)) {
        Process(image, buffer);
        return true;
    }else{
        return false;
    }
}

//! Implement VideoInput::GrabNewest()
bool MirrorVideo::GrabNewest( unsigned char* image, bool wait )
{
    if(videoin->GrabNewest(buffer,wait)) {
        Process(image, buffer);
        return true;
    }else{
        return false;
    }
}

std::vector<VideoInterface*>& MirrorVideo::InputStreams()
{
    return inputs;
}

unsigned int MirrorVideo::AvailableFrames() const
{
    BufferAwareVideoInterface* vpi = dynamic_cast<BufferAwareVideoInterface*>(videoin);
    if(!vpi)
    {
        pango_print_warn("Mirror: child interface is not buffer aware.");
        return 0;
    }
    else
    {
        return vpi->AvailableFrames();
    }
}

bool MirrorVideo::DropNFrames(uint32_t n)
{
    BufferAwareVideoInterface* vpi = dynamic_cast<BufferAwareVideoInterface*>(videoin);
    if(!vpi)
    {
        pango_print_warn("Mirror: child interface is not buffer aware.");
        return false;
    }
    else
    {
        return vpi->DropNFrames(n);
    }
}

}
