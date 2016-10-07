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

// This definition is required on older compilers for portable printf format macros.
// Included here to ensure this is defined before Python includes it.
#define __STDC_FORMAT_MACROS

#include <pangolin/python/PyModulePangolin.h>
#include <pangolin/console/ConsoleView.h>

#ifdef BUILD_PANGOLIN_VARS
#   include <pangolin/python/PyVar.h>
#   include <pangolin/var/var.h>
#   include <pangolin/var/varextra.h>
#endif // BUILD_PANGOLIN_VARS

#ifdef BUILD_PANGOLIN_VIDEO
#   include <pangolin/display/display.h>
#   include <pangolin/display/display_internal.h>
#   include <pangolin/display/view.h>
#endif // BUILD_PANGOLIN_VIDEO


namespace pangolin
{

#ifdef BUILD_PANGOLIN_VARS
PyObject * pangolin_save(PyObject* /*self*/, PyObject* args)
{
    Var<std::string> default_filename("pango.console.default_filename");
    const char *filename = default_filename.Get().c_str();
    const char *prefix = "";

    if (!PyArg_ParseTuple(args, "|ss", &filename, &prefix)) {
        return NULL;
    }

    SaveJsonFile(filename, prefix);

    Py_RETURN_NONE;
}

PyObject * pangolin_load(PyObject* /*self*/, PyObject* args)
{
    Var<std::string> default_filename("pango.console.default_filename");
    const char *filename = default_filename.Get().c_str();
    const char *prefix = "";

    if (!PyArg_ParseTuple(args, "|ss", &filename, &prefix)) {
        return NULL;
    }

    LoadJsonFile(filename, prefix);

    Py_RETURN_NONE;
}

#if defined(BUILD_PANGOLIN_GUI) && defined(BUILD_PANGOLIN_VIDEO)
extern __thread PangolinGl* context;

PyObject * pangolin_record_window(PyObject* /*self*/, PyObject* args)
{
    Var<std::string> default_record_uri("pango.console.default_record_uri");
    const char *record_uri = default_record_uri.Get().c_str();

    if (!PyArg_ParseTuple(args, "|s", &record_uri)) {
        return NULL;
    }

    DisplayBase().RecordOnRender(record_uri);
    if(context->console_view) {
        context->console_view->ShowWithoutAnimation(false);
    }
    Py_RETURN_NONE;
}
#endif // defined(BUILD_PANGOLIN_GUI) && defined(BUILD_PANGOLIN_VIDEO)

static PyMethodDef PangoMethods[] = {
#ifdef BUILD_PANGOLIN_VARS
    {"save",  pangolin_save, METH_VARARGS, "Save Pangolin Variables to a file."},
    {"load",  pangolin_load, METH_VARARGS, "Load Pangolin Variables to a file."},
#endif // BUILD_PANGOLIN_VARS
#if defined(BUILD_PANGOLIN_GUI) && defined(BUILD_PANGOLIN_VIDEO)
    {"record_window",  pangolin_record_window, METH_VARARGS, "Record window contents to video file."},
#endif // defined(BUILD_PANGOLIN_GUI) && defined(BUILD_PANGOLIN_VIDEO)
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
#endif // PY_MAJOR_VERSION >= 3

#endif // BUILD_PANGOLIN_VARS

PyMODINIT_FUNC
InitPangoModule()
{
#ifdef BUILD_PANGOLIN_VARS
    // Default settings
    Var<std::string>("pango.console.default_filename", "vars.json");
    Var<std::string>("pango.console.default_record_uri", "ffmpeg:[fps=50,bps=8388608,unique_filename]//screencap.avi");

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

#endif // BUILD_PANGOLIN_VARS
}

}
