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
#include <pangolin/platform.h>

namespace pangolin
{

/// Class represents a reference counted PythonObject.
/// PythonObject is appropriately Py_INCREF'd and Py_DECREF'd
class PyUniqueObj
{
public:
    inline
    PyUniqueObj()
        : obj(0)
    {
    }

    /// Assumption: PythonObject has already been appropriately INCREF'd.
    inline
    PyUniqueObj(PyObject* obj)
        : obj(obj)
    {
    }

    inline
    PyUniqueObj(const PyUniqueObj& other)
        :obj(other.obj)
    {
        if(obj) Py_INCREF(obj);
    }

    inline
    ~PyUniqueObj()
    {
        if(obj) Py_DECREF(obj);
    }

#ifdef CALLEE_HAS_RVALREF
    inline
    PyUniqueObj(PyUniqueObj&& other)
        : obj(other.obj)
    {
        other.obj = 0;
    }

    inline
    void operator=(PyUniqueObj&& other)
    {
        Release();
        obj = other.obj;
        other.obj = 0;
    }
#endif // CALLEE_HAS_RVALREF

    inline
    void operator=(PyObject* obj)
    {
        Release();
        this->obj = obj;
    }

    inline
    void Release() {
        if(obj) {
            Py_DECREF(obj);
            obj = 0;
        }
    }

    inline
    PyObject* operator*() {
        return obj;
    }

    inline
    operator PyObject*() {
        return obj;
    }

private:
    PyObject* obj;
};

}
