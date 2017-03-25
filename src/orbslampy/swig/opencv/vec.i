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

%include <stdint.i>
%include <opencv/matx.i>

%include <opencv/detail/vec.i>

/* %cv_vec_instantiate(type, d1, type_alias, np_basic_type)
 *
 *  Generete the wrapper code to a specific cv::Vec template instantiation.
 *
 *  type - The cv::Vec value type.
 *  d1 - The number of rows.
 *  type_alias - The value type alias used at the cv::Vec typedefs.
 *  np_basic_type - The character code[0] describing the numpy array item type.
 *
 *  For instance, the C++ type cv::Vec<double, 2> would be instantiated with:
 *
 *      %cv_vec_instantiate(double, 2, d, f)
 *
 *  which would generate a wrapper Python class Vec2d.
 *
 *  [0]: http://docs.scipy.org/doc/numpy/reference/arrays.interface.html#__array_interface__
 */
%define %cv_vec_instantiate(type, d1, type_alias, np_basic_type)
    #if !_CV_VEC_##type##_##d1##_INSTANTIATED_
        %cv_matx_instantiate(type, d1, 1, type_alias, np_basic_type)
        %template(_Vec_##type##_##d1) cv::Vec< type, d1>;
        %template(_DataType_Vec_##type##_##d1) cv::DataType<cv::Vec< type, d1> >;
        %pythoncode
        %{
            Vec##d1##type_alias = _Vec_##type##_##d1
            DataType_Vec##d1##type_alias = _DataType_Vec_##type##_##d1
        %}
        #define _CV_VEC_##type##_##d1##_INSTANTIATED_
    #endif
%enddef

%extend cv::Vec
{
    %pythoncode
     {
         import re
         _re_pattern = re.compile("^_Vec_(?P<value_type>[a-zA-Z_][a-zA-Z0-9_]*)_(?P<rows>[0-9]+)$")
     }

    Vec(std::vector<value_type> arg)
    {
        return Factory< $parentclassname >::construct(arg);
    }

    %pythonprepend Vec(std::vector<value_type> arg)
    {
        ma = self._re_pattern.match(self.__class__.__name__)
        value_type = ma.group("value_type")
        rows = int(ma.group("rows"))

        array = _array_map[value_type](rows)
        for i in range(len(args)):
            array[i] = args[i]

        args = [array]
    }

    %pythoncode
    {
        def __getattribute__(self, name):
            if name == "__array_interface__":
                ma = self._re_pattern.match(self.__class__.__name__)
                value_type = ma.group("value_type")
                rows = int(ma.group("rows"))
                return {"shape": (rows, 1),
                        "typestr": _cv_numpy_typestr_map[value_type],
                        "data": (int(self.val), False)}
            else:
                return object.__getattribute__(self, name)

        def __getitem__(self, key):
            ma = self._re_pattern.match(self.__class__.__name__)
            rows = int(ma.group("rows"))

            if not isinstance(key, int):
                raise TypeError

            if key >= rows:
                raise IndexError

            return self(key)
     }

    std::string __str__()
    {
        std::ostringstream s;
        s << *$self;
        return s.str();
    }
}


/* %cv_vec_instantiate_defaults
 *
 * Generate a wrapper class to all cv::Vec which has a typedef on OpenCV header file.
 */
%define %cv_vec_instantiate_defaults
    %cv_vec_instantiate(uint8_t, 2, b, u)
    %cv_vec_instantiate(uint8_t, 3, b, u)
    %cv_vec_instantiate(uint8_t, 4, b, u)

    %cv_vec_instantiate(short, 2, s, i)
    %cv_vec_instantiate(short, 3, s, i)
    %cv_vec_instantiate(short, 4, s, i)

    %cv_vec_instantiate(ushort, 2, w, u)
    %cv_vec_instantiate(ushort, 3, w, u)
    %cv_vec_instantiate(ushort, 4, w, u)

    %cv_vec_instantiate(int, 2, i, i)
    %cv_vec_instantiate(int, 3, i, i)
    %cv_vec_instantiate(int, 4, i, i)
    %cv_vec_instantiate(int, 6, i, i)
    %cv_vec_instantiate(int, 8, i, i)

    %cv_vec_instantiate(float, 2, f, f)
    %cv_vec_instantiate(float, 3, f, f)
    %cv_vec_instantiate(float, 4, f, f)
    %cv_vec_instantiate(float, 6, f, f)

    %cv_vec_instantiate(double, 2, d, f)
    %cv_vec_instantiate(double, 3, d, f)
    %cv_vec_instantiate(double, 4, d, f)
    %cv_vec_instantiate(double, 6, d, f)

    %cv_vec_instantiate(double, 6, d, f)
%enddef
