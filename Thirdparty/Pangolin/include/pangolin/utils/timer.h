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

#ifndef PANGOLIN_TIMER_H
#define PANGOLIN_TIMER_H

#include <pangolin/platform.h>
#include <stdint.h>

#if defined(_UNIX_)
#  include <sys/time.h>
#endif

namespace pangolin
{

#if defined(_UNIX_)
    typedef timeval basetime;
#elif defined(_WIN_)
    typedef int64_t basetime;
#endif

PANGOLIN_EXPORT
basetime TimeNow();

PANGOLIN_EXPORT
double Time_s(basetime t);

PANGOLIN_EXPORT
int64_t Time_us(basetime t);

PANGOLIN_EXPORT
double TimeDiff_s(basetime start, basetime end);

PANGOLIN_EXPORT
int64_t TimeDiff_us(basetime start, basetime end);

PANGOLIN_EXPORT
basetime TimeFromSeconds(double seconds);

PANGOLIN_EXPORT
basetime TimeAdd(basetime t1, basetime t2);

inline double TimeNow_s()
{
    return Time_s(TimeNow());
}

inline basetime WaitUntil(basetime t)
{
    // TODO: use smarter sleep!
    basetime currtime = TimeNow();
    while( TimeDiff_s(currtime,t) > 0 )
        currtime = TimeNow();
    return currtime;
}

struct Timer
{
    Timer() {
        Reset();
    }
    
    void Reset()
    {
        start = TimeNow();
    }
    
    double Elapsed_s()
    {
        basetime currtime = TimeNow();
        return TimeDiff_s(start,currtime);
    }
    
    basetime start;
};

}

#endif //PANGOLIN_TIMER_H
