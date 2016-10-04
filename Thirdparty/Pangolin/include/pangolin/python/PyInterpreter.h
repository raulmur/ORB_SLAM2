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

#pragma once

#include <Python.h>
#include <pangolin/var/varextra.h>
#include <pangolin/python/PyUniqueObj.h>
#include <pangolin/console/ConsoleInterpreter.h>
#include <pangolin/compat/thread.h>
#include <queue>
#include <set>

namespace pangolin
{

class PyInterpreter : public ConsoleInterpreter
{
public:
    PyInterpreter();

    ~PyInterpreter() PANGOLIN_OVERRIDE;

    void PushCommand(const std::string &cmd) PANGOLIN_OVERRIDE;

    bool PullLine(ConsoleLine& line) PANGOLIN_OVERRIDE;

    std::vector<std::string> Complete(
        const std::string& cmd, int max_options
    ) PANGOLIN_OVERRIDE;

    static void AttachPrefix(void* data, const std::string& name, VarValueGeneric& var, bool brand_new );

private:
    PyObject* pycompleter;
    PyObject* pycomplete;

    std::string ToString(PyObject* py);
    void CheckPrintClearError();
    PyUniqueObj EvalExec(const std::string& cmd);

    std::queue<ConsoleLine> line_queue;
    std::set<std::string> base_prefixes;
};

}
