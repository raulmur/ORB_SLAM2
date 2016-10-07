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
#include <iomanip>
#include <queue>

#include <structmember.h>

#include <pangolin/var/var.h>
#include <pangolin/console/ConsoleInterpreter.h>

namespace pangolin
{

struct PyPangoIO {
    PyObject_HEAD

    static PyTypeObject Py_type;
    static PyMethodDef Py_methods[];

    PyPangoIO(PyTypeObject *type, std::queue<ConsoleLine>& line_queue, ConsoleLineType line_type)
        : line_queue(line_queue), line_type(line_type)
    {
#if PY_MAJOR_VERSION >= 3
        ob_base.ob_refcnt = 1;
        ob_base.ob_type = type;
#else
        ob_refcnt = 1;
        ob_type = type;
#endif
    }

    static void Py_dealloc(PyPangoIO* self)
    {
        delete self;
    }

    static PyObject * Py_new(PyTypeObject */*type*/, PyObject */*args*/, PyObject */*kwds*/)
    {
        // Failure. Can only new in c++
        return 0;
    }

    static int Py_init(PyPangoIO* /*self*/, PyObject* /*args*/, PyObject* /*kwds*/)
    {
        return 0;
    }

    static PyObject* Py_getattr(PyPangoIO *self, char* name)
    {
#if PY_MAJOR_VERSION >= 3
        PyObject* pystr = PyUnicode_FromString(name);
#else
        PyObject* pystr = PyString_FromString(name);
#endif
        return PyObject_GenericGetAttr((PyObject*)self, pystr );
    }

    static int Py_setattr(PyPangoIO *self, char* name, PyObject* val)
    {
#if PY_MAJOR_VERSION >= 3
        PyObject* pystr = PyUnicode_FromString(name);
#else
        PyObject* pystr = PyString_FromString(name);
#endif
        return PyObject_GenericSetAttr((PyObject*)self, pystr, val);
    }

    static PyObject* Py_write(PyPangoIO* self, PyObject *args)
    {
        const char *text = 0;
        if (PyArg_ParseTuple(args, "s", &text)) {
            self->buffer += std::string(text);
            size_t nl = self->buffer.find_first_of('\n');
            while(nl != std::string::npos) {
                const std::string line = self->buffer.substr(0,nl);
                self->line_queue.push(ConsoleLine(line,self->line_type));
                self->buffer = self->buffer.substr(nl+1);
                nl = self->buffer.find_first_of('\n');
            }
        }
        Py_RETURN_NONE;
    }

    std::string buffer;
    std::queue<ConsoleLine>& line_queue;
    ConsoleLineType line_type;
};

PyMethodDef PyPangoIO::Py_methods[] = {
    {"write", (PyCFunction)PyPangoIO::Py_write, METH_VARARGS, "Write to console" },
    {NULL}
};

PyTypeObject PyPangoIO::Py_type = {
    PyVarObject_HEAD_INIT(NULL,0)
    "pangolin.PangoIO",                       /* tp_name*/
    sizeof(PyPangoIO),                        /* tp_basicsize*/
    0,                                        /* tp_itemsize*/
    (destructor)PyPangoIO::Py_dealloc,        /* tp_dealloc*/
    0,                                        /* tp_print*/
    (getattrfunc)PyPangoIO::Py_getattr,       /* tp_getattr*/
    (setattrfunc)PyPangoIO::Py_setattr,       /* tp_setattr*/
    0,                                        /* tp_compare*/
    0,                                        /* tp_repr*/
    0,                                        /* tp_as_number*/
    0,                                        /* tp_as_sequence*/
    0,                                        /* tp_as_mapping*/
    0,                                        /* tp_hash */
    0,                                        /* tp_call*/
    0,                                        /* tp_str*/
    0,                                        /* tp_getattro*/
    0,                                        /* tp_setattro*/
    0,                                        /* tp_as_buffer*/
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE, /* tp_flags*/
    "PyPangoIO object",                       /* tp_doc */
    0,		                              /* tp_traverse */
    0,		                              /* tp_clear */
    0,		                              /* tp_richcompare */
    0,		                              /* tp_weaklistoffset */
    0,		                              /* tp_iter */
    0,		                              /* tp_iternext */
    PyPangoIO::Py_methods,                    /* tp_methods */
    0,                                        /* tp_members */
    0,                                        /* tp_getset */
    0,                                        /* tp_base */
    0,                                        /* tp_dict */
    0,                                        /* tp_descr_get */
    0,                                        /* tp_descr_set */
    0,                                        /* tp_dictoffset */
    (initproc)PyPangoIO::Py_init,             /* tp_init */
    0,                                        /* tp_alloc */
    (newfunc)PyPangoIO::Py_new,               /* tp_new */
};

}
