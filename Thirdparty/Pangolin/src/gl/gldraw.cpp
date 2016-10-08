/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove, Richard Newcombe
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


#include <pangolin/gl/gldraw.h>
#include <pangolin/utils/timer.h>

namespace pangolin
{

void glRecordGraphic(float x, float y, float radius)
{
    const int ticks = static_cast<int>(TimeNow_s());
    if( ticks % 2 )
    {
        // now, render a little red "recording" dot
        glPushAttrib(GL_ENABLE_BIT);
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glColor3ub( 255, 0, 0 );
        glDrawCircle( x, y, radius );
        glPopAttrib();
    }

}

}
