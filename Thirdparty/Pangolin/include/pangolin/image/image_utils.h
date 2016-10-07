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

#ifndef PANGOLIN_IMAGE_UTILS_H
#define PANGOLIN_IMAGE_UTILS_H

#include <limits>
#include <utility>

#include <pangolin/image/image.h>
#include <pangolin/plot/range.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/glpixformat.h>

namespace pangolin
{

template<typename T>
std::pair<float,float> GetOffsetScale(const pangolin::Image<T>& img, size_t channels, float type_max, float format_max)
{
    const size_t max_channels = 3;
    const size_t colour_channels = std::min(channels, max_channels);
    std::pair<float,float> chan_mm[max_channels];
    for(size_t c=0; c < max_channels; ++c) {
        chan_mm[c].first  = +std::numeric_limits<float>::max();
        chan_mm[c].second = -std::numeric_limits<float>::max();
    }

    for(size_t y=0; y < img.h; ++y) {
        T* pix = (T*)((char*)img.ptr + y*img.pitch);
        for(size_t x=0; x < img.w; ++x) {
            for(size_t c=0; c < colour_channels; ++c)
            {
                if(pix[c] < chan_mm[c].first)  chan_mm[c].first  = (float)pix[c];
                if(pix[c] > chan_mm[c].second) chan_mm[c].second = (float)pix[c];
            }

            pix += channels;
        }
    }

    // Find min / max of all channels, ignoring 4th alpha channel
    std::pair<float,float> mm = chan_mm[0];
    for(size_t c=1; c < colour_channels; ++c) {
        mm.first  = std::min(mm.first, chan_mm[c].first);
        mm.second = std::max(mm.second, chan_mm[c].second);
    }

    const float type_scale = format_max / type_max;
    const float offset = -type_scale* mm.first;
    const float scale = type_max / (mm.second - mm.first);
    return std::pair<float,float>(offset, scale);
}

template<typename T>
pangolin::Image<T> GetImageRoi( pangolin::Image<T> img, size_t channels, const pangolin::XYRangei& roi )
{
    const int xmin = std::min(roi.x.min,roi.x.max);
    const int ymin = std::min(roi.y.min,roi.y.max);
    return pangolin::Image<T>(
        roi.x.AbsSize(), roi.y.AbsSize(),
        img.pitch, img.RowPtr(ymin) + channels*xmin
    );
}

template<typename T>
std::pair<float,float> GetOffsetScale(const pangolin::Image<T>& img, pangolin::XYRangei iroi, const pangolin::GlPixFormat& glfmt)
{
    iroi.Clamp(0, (int)img.w-1, 0, (int)img.h-1 );

    const size_t num_channels = pangolin::GlFormatChannels(glfmt.glformat);

    if(glfmt.gltype == GL_UNSIGNED_BYTE) {
        return GetOffsetScale(GetImageRoi(img.template Reinterpret<unsigned char>(), num_channels, iroi), num_channels, 255.0f, 1.0f);
    }else if(glfmt.gltype == GL_UNSIGNED_SHORT) {
        return GetOffsetScale(GetImageRoi(img.template Reinterpret<unsigned short>(), num_channels, iroi), num_channels, 65535.0f, 1.0f);
    }else if(glfmt.gltype == GL_FLOAT) {
        return GetOffsetScale(GetImageRoi(img.template Reinterpret<float>(), num_channels, iroi), num_channels, 1.0f, 1.0f);
    }else if(glfmt.gltype == GL_DOUBLE) {
        return GetOffsetScale(GetImageRoi(img.template Reinterpret<double>(), num_channels, iroi), num_channels, 1.0f, 1.0f);
    }else{
        return std::pair<float,float>(0.0f, 1.0f);
    }
}

}

#endif // PANGOLIN_IMAGE_UTILS_H
