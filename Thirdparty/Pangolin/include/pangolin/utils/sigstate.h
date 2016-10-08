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

#ifndef PANGOLIN_SIGSTATE_H
#define PANGOLIN_SIGSTATE_H

#include <map>
#include <vector>
#include <pangolin/platform.h>
#include <pangolin/utils/file_utils.h>
#include <csignal>

#ifndef SIGPIPE
#  define SIGPIPE 13
#endif // !SIGPIPE

namespace pangolin
{

typedef void (*SigCallbackFn)(int);

struct PANGOLIN_EXPORT SigCallback
{
    SigCallback(const int & sig, SigCallbackFn fn, void* data)
     : sig(sig), fn(fn), data(data), value(false)
    {
        std::signal(sig, fn);
    }

    int sig;
    SigCallbackFn fn;
    void * data;
    volatile sig_atomic_t value;
};

class PANGOLIN_EXPORT SigState
{
public:
    static SigState& I();

    SigState();
    ~SigState();

    void Clear();

    std::map<int, SigCallback> sig_callbacks;
};

PANGOLIN_EXPORT
void RegisterNewSigCallback(SigCallbackFn callback, void* data, const int signal);

}

#endif // PANGOLIN_SIGSTATE_H
