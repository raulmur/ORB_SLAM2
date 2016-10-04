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

#include <structmember.h>
#include <iomanip>
#include <pangolin/var/var.h>
#include <pangolin/python/PyUniqueObj.h>

namespace pangolin
{

PyObject* GetPangoVarAsPython(const std::string& name)
{
    VarState::VarStoreContainer::iterator i = VarState::I().vars.find(name);
    if(i != VarState::I().vars.end()) {
        VarValueGeneric* var = i->second;

        try{
            if( !strcmp(var->TypeId(), typeid(bool).name() ) ) {
                const bool val = Var<bool>(*var).Get();
                return PyBool_FromLong( val );
            }else if( !strcmp(var->TypeId(), typeid(short).name() ) ||
                      !strcmp(var->TypeId(), typeid(int).name() ) ||
                      !strcmp(var->TypeId(), typeid(long).name() ) ) {
                const long val = Var<long>(*var).Get();
                return PyLong_FromLong( val );
            }else if( !strcmp(var->TypeId(), typeid(double).name() ) ||
                      !strcmp(var->TypeId(), typeid(float).name() ) ) {
                const double val = Var<double>(*var).Get();
                return PyFloat_FromDouble(val);
            }else{
                const std::string val = var->str->Get();
#if PY_MAJOR_VERSION >= 3
                return PyUnicode_FromString(val.c_str());
#else
                return PyString_FromString(val.c_str());
#endif
            }
        }catch(std::exception) {
        }
    }

    Py_RETURN_NONE;
}

void SetPangoVarFromPython(const std::string& name, PyObject* val)
{
    try{
        if (PyFloat_Check(val)) {
            pangolin::Var<double> pango_var(name);
            pango_var = PyFloat_AsDouble(val);
        }else if (PyLong_Check(val)) {
            pangolin::Var<long> pango_var(name);
            pango_var = PyLong_AsLong(val);
        }else if (PyBool_Check(val)) {
            pangolin::Var<bool> pango_var(name);
            pango_var = (val == Py_True) ? true : false;
        }
#if PY_MAJOR_VERSION >= 3
        else if (PyUnicode_Check(val)) {
            pangolin::Var<std::string> pango_var(name);
            pango_var = PyUnicode_AsUTF8(val);
        }
#else
        if (PyString_Check(val)) {
            pangolin::Var<std::string> pango_var(name);
            pango_var = PyString_AsString(val);
        } else if (PyInt_Check(val)) {
            pangolin::Var<int> pango_var(name);
            pango_var = PyInt_AsLong(val);
        }
#endif
        else {
            PyUniqueObj pystr = PyObject_Repr(val);
#if PY_MAJOR_VERSION >= 3
            const std::string str = PyUnicode_AsUTF8(pystr);
#else
            const std::string str = PyString_AsString(pystr);
#endif
            pangolin::Var<std::string> pango_var(name);
            pango_var = str;
        }
    }catch(std::exception e) {
        pango_print_error("%s\n", e.what());
    }
}

struct PyVar {
    static PyTypeObject Py_type;
    PyObject_HEAD

    PyVar(PyTypeObject *type)
    {
#if PY_MAJOR_VERSION >= 3
        ob_base.ob_refcnt = 1;
        ob_base.ob_type = type;
#else
        ob_refcnt = 1;
        ob_type = type;
#endif
    }

    static void Py_dealloc(PyVar* self)
    {
        delete self;
    }

    static PyObject * Py_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
    {
        PyVar* self = new PyVar(type);
        return (PyObject *)self;
    }

    static int Py_init(PyVar *self, PyObject *args, PyObject *kwds)
    {
        char* cNamespace = 0;
        if (!PyArg_ParseTuple(args, "s", &cNamespace))
            return -1;

        self->ns = std::string(cNamespace);

        return 0;
    }

    static PyObject* Py_getattr(PyVar *self, char* name)
    {
        const std::string prefix = self->ns + ".";
        const std::string full_name = self->ns.empty() ? name : prefix + std::string(name);

        if( !strcmp(name, "__call__") ||
                !strcmp(name, "__dict__") ||
                !strcmp(name, "__methods__") ||
                !strcmp(name, "__class__") )
        {
            // Default behaviour
#if PY_MAJOR_VERSION >= 3
            return PyObject_GenericGetAttr((PyObject*)self, PyUnicode_FromString(name));
#else
            return PyObject_GenericGetAttr((PyObject*)self, PyString_FromString(name));
#endif
        } else if( !strcmp(name, "__members__") ) {
            const int nss = prefix.size();
            PyObject* l = PyList_New(0);
            for(const std::string& s : VarState::I().var_adds) {
                if(!s.compare(0, nss, prefix)) {
                    size_t dot = s.find_first_of('.', nss);
                    std::string val = (dot != std::string::npos) ? s.substr(nss, dot - nss) : s.substr(nss);
#if PY_MAJOR_VERSION >= 3
                    PyList_Append(l, PyUnicode_FromString(val.c_str()));
#else
                    PyList_Append(l, PyString_FromString(val.c_str()));
#endif
                }
            }

            return l;
        }else if( pangolin::VarState::I().Exists(full_name) ) {
            return GetPangoVarAsPython(full_name);
        }else{
            PyVar* obj = (PyVar*)PyVar::Py_new(&PyVar::Py_type,NULL,NULL);
            if(obj) {
                obj->ns = full_name;
                return PyObject_Init((PyObject *)obj,&PyVar::Py_type);
            }
            return (PyObject *)obj;
        }

        Py_RETURN_NONE;
    }

    static int Py_setattr(PyVar *self, char* name, PyObject* val)
    {
        const std::string full_name = self->ns.empty() ? name : self->ns + "." + std::string(name);
        SetPangoVarFromPython(full_name, val);
        return 0;
    }

    std::string ns;
};

 PyTypeObject PyVar::Py_type = {
     PyVarObject_HEAD_INIT(NULL,0)
    "pangolin.Var",                           /* tp_name*/
    sizeof(PyVar),                            /* tp_basicsize*/
    0,                                        /* tp_itemsize*/
    (destructor)PyVar::Py_dealloc,            /* tp_dealloc*/
    0,                                        /* tp_print*/
    (getattrfunc)PyVar::Py_getattr,           /* tp_getattr*/
    (setattrfunc)PyVar::Py_setattr,           /* tp_setattr*/
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
    "PyVar object",                           /* tp_doc */
    0,		                                  /* tp_traverse */
    0,		                                  /* tp_clear */
    0,		                                  /* tp_richcompare */
    0,		                                  /* tp_weaklistoffset */
    0,		                                  /* tp_iter */
    0,		                                  /* tp_iternext */
    0,                                        /* tp_methods */
    0,                                        /* tp_members */
    0,                                        /* tp_getset */
    0,                                        /* tp_base */
    0,                                        /* tp_dict */
    0,                                        /* tp_descr_get */
    0,                                        /* tp_descr_set */
    0,                                        /* tp_dictoffset */
    (initproc)PyVar::Py_init,                 /* tp_init */
    0,                                        /* tp_alloc */
    (newfunc)PyVar::Py_new,                   /* tp_new */
};

}
