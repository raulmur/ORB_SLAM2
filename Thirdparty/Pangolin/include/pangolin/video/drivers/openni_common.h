/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2014 Steven Lovegrove
 *               2015 Richard Newcombe
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

#ifndef PANGOLIN_OPENNI_COMMON_H
#define PANGOLIN_OPENNI_COMMON_H

#include <pangolin/image/image_common.h>

namespace pangolin
{

enum OpenNiSensorType
{
    OpenNiUnassigned = -1,
    OpenNiRgb = 0,
    OpenNiIr,
    OpenNiDepth_1mm,
    OpenNiDepth_1mm_Registered,
    OpenNiDepth_100um,
    OpenNiIr8bit,
    OpenNiIr24bit,
    OpenNiIrProj,
    OpenNiIr8bitProj,
    OpenNiGrey
};

struct PANGOLIN_EXPORT OpenNiStreamMode
{
    OpenNiStreamMode(
        OpenNiSensorType sensor_type=OpenNiUnassigned,
        ImageDim dim=ImageDim(640,480), int fps=30, int device=0
    )
        : sensor_type(sensor_type), dim(dim), fps(fps), device(device)
    {

    }

    OpenNiSensorType sensor_type;
    ImageDim dim;
    int fps;
    int device;
};

}

#endif // PANGOLIN_OPENNI_COMMON_H
