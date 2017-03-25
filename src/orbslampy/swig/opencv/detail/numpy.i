/* Copyright (c) 2015 The OpenCV-SWIG Library Developers. See the AUTHORS file at the
 * top-level directory of this distribution and at
 * https://github.com/renatoGarcia/opencv-swig/blob/master/AUTHORS.
 *
 * This file is part of OpenCV-SWIG Library. It is subject to the 3-clause BSD license
 * terms as in the LICENSE file found in the top-level directory of this distribution and
 * at https://github.com/renatoGarcia/opencv-swig/blob/master/LICENSE. No part of
 * OpenCV-SWIG Library, including this file, may be copied, modified, propagated, or
 * distributed except according to the terms contained in the LICENSE file.
 */

%pythoncode
{
    import sys as _sys
    if _sys.byteorder == 'little':
        _cv_numpy_endianess = '<'
    else:
        _cv_numpy_endianess = '>'

    _cv_numpy_typestr_map = {}
    _cv_numpy_bla = {}
}

%inline
%{
    namespace cv
    {
        template <typename T>
        struct _SizeOf
        {
            enum {value = sizeof(T)};
        };
    }
%}

%define %cv_numpy_add_type(typ, np_basic_type)
    #if not _CV_NUMPY_##typ##_
        namespace cv
        {
            %template(_cv_numpy_sizeof_##typ) _SizeOf< typ >;
        }
        %pythoncode
        {
            if _cv_numpy_sizeof_##typ##.value == 1:
                _cv_numpy_typestr_map[#typ] = "|" + #np_basic_type + "1"
            else:
                _cv_numpy_typestr_map[#typ] = _cv_numpy_endianess  + #np_basic_type + str(_cv_numpy_sizeof_##typ##.value)
        }
        #define _CV_NUMPY_##typ##_
    #endif
%enddef
