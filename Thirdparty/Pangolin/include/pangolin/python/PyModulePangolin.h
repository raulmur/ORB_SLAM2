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
#include <pangolin/python/PyVar.h>
#include <pangolin/var/var.h>
#include <pangolin/var/varextra.h>

namespace pangolin
{

inline PyObject * pangolin_save(PyObject* /*self*/, PyObject* args)
{
    Var<std::string> default_filename("pango.console.default_filename");
    const char *filename = default_filename.Get().c_str();
    const char *prefix = "";

    if (!PyArg_ParseTuple(args, "|ss", &filename, &prefix))
        return NULL;

    SaveJsonFile(filename, prefix);

    Py_RETURN_NONE;
}

inline PyObject * pangolin_load(PyObject* /*self*/, PyObject* args)
{
    Var<std::string> default_filename("pango.console.default_filename");
    const char *filename = default_filename.Get().c_str();
    const char *prefix = "";

    if (!PyArg_ParseTuple(args, "|ss", &filename, &prefix))
        return NULL;

    LoadJsonFile(filename, prefix);

    Py_RETURN_NONE;
}

static PyMethodDef PangoMethods[] = {
    {"save",  pangolin_save, METH_VARARGS, "Save Pangolin Variables to a file."},
    {"load",  pangolin_load, METH_VARARGS, "Load Pangolin Variables to a file."},
    {NULL}
};

#if PY_MAJOR_VERSION >= 3
static struct PyModuleDef PangoModule = {
    PyModuleDef_HEAD_INIT,
    "pangolin",
    NULL,
    -1,
    PangoMethods
};
#endif

PyMODINIT_FUNC
InitPangoModule()
{
    // Default settings
    Var<std::string>("pango.console.default_filename","vars.json");

    PyObject *m = 0;
#if PY_MAJOR_VERSION >= 3
        m = PyModule_Create(&PangoModule);
#else
        m = Py_InitModule("pangolin", PangoMethods);
#endif
    if(m) {
        if (PyType_Ready(&PyVar::Py_type) >= 0) {
            Py_INCREF(&PyVar::Py_type);
            PyModule_AddObject(m, "Var", (PyObject *)&PyVar::Py_type);
        }else{
            pango_print_error("Unable to create pangolin Python objects.\n");
        }
    }else{
        pango_print_error("Unable to initialise pangolin Python module.\n");
    }

#if PY_MAJOR_VERSION >= 3
    return m;
#endif
}

}
