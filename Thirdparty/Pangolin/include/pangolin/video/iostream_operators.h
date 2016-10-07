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

#ifndef PANGOLIN_IOSTREAM_OPERATORS_H
#define PANGOLIN_IOSTREAM_OPERATORS_H


#include <iostream>

#include <pangolin/image/image_common.h>

namespace pangolin
{

inline std::istream& operator>> (std::istream &is, ImageDim &dim)
{
    if(std::isdigit(is.peek()) ) {
        // Expect 640x480, 640*480, ...
        is >> dim.x; is.get(); is >> dim.y;
    }else{
        // Expect 'VGA', 'QVGA', etc
        std::string sdim;
        is >> sdim;
        ToUpper(sdim);

        if( !sdim.compare("QQVGA") ) {
            dim = ImageDim(160,120);
        }else if( !sdim.compare("HQVGA") ) {
            dim = ImageDim(240,160);
        }else if( !sdim.compare("QVGA") ) {
            dim = ImageDim(320,240);
        }else if( !sdim.compare("WQVGA") ) {
            dim = ImageDim(360,240);
        }else if( !sdim.compare("HVGA") ) {
            dim = ImageDim(480,320);
        }else if( !sdim.compare("VGA") ) {
            dim = ImageDim(640,480);
        }else if( !sdim.compare("WVGA") ) {
            dim = ImageDim(720,480);
        }else if( !sdim.compare("SVGA") ) {
            dim = ImageDim(800,600);
        }else if( !sdim.compare("DVGA") ) {
            dim = ImageDim(960,640);
        }else if( !sdim.compare("WSVGA") ) {
            dim = ImageDim(1024,600);
        }else{
            throw VideoException("Unrecognised image-size string.");
        }
    }
    return is;
}

inline std::istream& operator>> (std::istream &is, ImageRoi &roi)
{
    is >> roi.x; is.get(); is >> roi.y; is.get();
    is >> roi.w; is.get(); is >> roi.h;
    return is;
}

inline std::istream& operator>> (std::istream &is, VideoPixelFormat& fmt)
{
    std::string sfmt;
    is >> sfmt;
    fmt = VideoFormatFromString(sfmt);
    return is;
}

inline std::istream& operator>> (std::istream &is, Image<unsigned char>& img)
{
    size_t offset;
    is >> offset; is.get();
    img.ptr = (unsigned char*)0 + offset;
    is >> img.w; is.get();
    is >> img.h; is.get();
    is >> img.pitch;
    return is;
}

inline std::istream& operator>> (std::istream &is, StreamInfo &stream)
{
    VideoPixelFormat fmt;
    Image<unsigned char> img_offset;
    is >> img_offset; is.get();
    is >> fmt;
    stream = StreamInfo(fmt, img_offset);
    return is;
}

}

#endif // PANGOLIN_IOSTREAM_OPERATORS_H
