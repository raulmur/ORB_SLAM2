/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2013 Steven Lovegrove
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

#include <pangolin/video/drivers/firewire_deinterlace.h>
#include <dc1394/conversions.h>

namespace pangolin
{

FirewireDeinterlace::FirewireDeinterlace(VideoInterface* videoin)
    : videoin(videoin), buffer(0)
{
    if(videoin->Streams().size() != 1)
        throw VideoException("FirewireDeinterlace input must have exactly one stream");
    
    const StreamInfo& stmin = videoin->Streams()[0];

    StreamInfo stm1(VideoFormatFromString("GRAY8"), stmin.Width(), stmin.Height(), stmin.Width(), 0);
    StreamInfo stm2(VideoFormatFromString("GRAY8"), stmin.Width(), stmin.Height(), stmin.Width(), (unsigned char*)0 + stmin.Width()*stmin.Height());
    streams.push_back(stm1);
    streams.push_back(stm2);

    buffer = new unsigned char[videoin->SizeBytes()];

    std::cout << videoin->Streams()[0].Width() << ", " << videoin->Streams()[0].Height() << std::endl;
}

FirewireDeinterlace::~FirewireDeinterlace()
{
    delete[] buffer;
    delete videoin;
}

size_t FirewireDeinterlace::SizeBytes() const
{
    return videoin->SizeBytes();
}

const std::vector<StreamInfo>& FirewireDeinterlace::Streams() const
{
    return streams;
}

void FirewireDeinterlace::Start()
{
    videoin->Start();
}

void FirewireDeinterlace::Stop()
{
    videoin->Stop();
}

bool FirewireDeinterlace::GrabNext( unsigned char* image, bool wait )
{
    if(videoin->GrabNext(buffer, wait)) {
        return ( dc1394_deinterlace_stereo(buffer,image, videoin->Streams()[0].Width(), 2*videoin->Streams()[0].Height() ) == DC1394_SUCCESS );
    }
    return false;
}

bool FirewireDeinterlace::GrabNewest( unsigned char* image, bool wait )
{
    if(videoin->GrabNewest(buffer, wait)) {
        return ( dc1394_deinterlace_stereo(buffer,image, videoin->Streams()[0].Width(), 2*videoin->Streams()[0].Height() ) == DC1394_SUCCESS );
    }
    return false;
}



}
