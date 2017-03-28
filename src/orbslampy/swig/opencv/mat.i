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

%include <opencv/detail/numpy.i>
%include <opencv/range.i>
%include <opencv/rect.i>
%include <opencv/scalar.i>
%include <opencv/size.i>
%include <opencv/vec.i>

%include <std_string.i>

namespace cv
{
    %ignore Mat::operator=;
    %ignore Mat::Mat(const Mat&, const Range*);
}

%include <opencv/detail/mat.i>

%cv_vec_instantiate_defaults

/* %cv_mat__instantiate(type, type_alias, np_basic_type)
 *
 *  Generete the wrapper code to a specific cv::Mat_<> template instantiation.
 *
 *  type - The cv::Mat_<> value type.
 *  type_alias - The value type alias used at the cv::Mat_<> typedefs.
 *  np_basic_type - The character code[0] describing the numpy array item type.
 *
 *  For instance, the C++ type cv::Mat_<cv::Vec3b> would be instantiated with:
 *
 *      %cv_mat__instantiate(Vec3b, 3b, f)
 *
 *  which would generate a Python wrapper class Mat3b.
 *
 *  [0]: http://docs.scipy.org/doc/numpy/reference/arrays.interface.html#__array_interface__
 */
%define %cv_mat__instantiate(type, type_alias, np_basic_type)

    %cv_numpy_add_type(type, np_basic_type)

    #if !_CV_MAT__##type##_INSTANTIATED_
        namespace cv
        {
            %template(_Mat__##type) Mat_< type >;
        }
        %pythoncode
        %{
            Mat##type_alias = _Mat__##type
        %}
        #define _CV_MAT__##type##_INSTANTIATED_
    #endif
%enddef

%header
%{
    #include <opencv2/core/core.hpp>
    #include <sstream>
%}

%inline
%{
    struct _mat__np_array_constructor
    {};

    char const* _depthToDtype(int depth)
    {
        switch(depth)
        {
        case CV_8U:
            return "u1";
        case CV_8S:
            return "i1";
        case CV_16U:
            return "u2";
        case CV_16S:
            return "i2";
        case CV_32S:
            return "i4";
        case CV_32F:
            return "f4";
        case CV_64F:
            return "f8";
        }

        return NULL;
    }

    int _toCvType(std::string const& dtype, int nChannel)
    {
        int depth;
        if (dtype == "u1")
        {
            depth = CV_8U;
        }
        else if (dtype == "i1")
        {
            depth = CV_8S;
        }
        else if (dtype == "u2")
        {
            depth = CV_16U;
        }
        else if (dtype == "i2")
        {
            depth = CV_16S;
        }
        else if (dtype == "i4")
        {
            depth = CV_32S;
        }
        else if (dtype == "f4")
        {
            depth = CV_32F;
        }
        else if (dtype == "f8")
        {
            depth = CV_64F;
        }

        return CV_MAKETYPE(depth, nChannel);
    }
%}


%extend cv::Mat
{
    Mat(int rows, int cols, int type, ptrdiff_t data)
    {
        return new cv::Mat(rows, cols, type, (void*)data);
    }

    %pythoncode
    %{
        def _typestr(self):
            typestr = _depthToDtype(self.depth())
            if typestr[-1] == '1':
                typestr = '|' + typestr
            else:
                typestr = _cv_numpy_endianess + typestr

            return typestr


        @classmethod
        def __get_channels(cls, array):
            if len(array.shape) == 3:
                n_channel = array.shape[2]
                if n_channel == 1:
                    raise ValueError("{} expects an one channel numpy ndarray be 2-dimensional.".format(cls))
            elif len(array.shape) == 2:
                n_channel = 1
            else:
                raise ValueError("{} supports only 2 or 3-dimensional numpy ndarray.".format(cls))

            return n_channel


        def __getattribute__(self, name):
            if name == "__array_interface__":
                n_channels = self.channels()
                if n_channels == 1:
                    shape = (self.rows, self.cols)
                else:
                    shape = (self.rows, self.cols, n_channels)

                return {"shape": shape,
                        "typestr": self._typestr(),
                        "data": (int(self.data), False)}

            else:
                return object.__getattribute__(self, name)

        @classmethod
        def from_array(cls, array):
            import numpy as np
            array = np.asarray(array)

            dtype = array.__array_interface__['typestr']
            dtype = dtype[1:]

            n_channel = cls.__get_channels(array)

            new_mat = Mat(array.shape[0],
                          array.shape[1],
                          _toCvType(dtype, n_channel),
                          array.__array_interface__['data'][0])

            # Holds an internal reference to keep the image buffer alive
            new_mat._array = array

            return new_mat
    %}

    std::string __str__()
    {
        std::ostringstream s;
        s << *$self;
        return s.str();
    }
}

%extend cv::Mat_
{
    Mat_(_mat__np_array_constructor, int rows, int cols, ptrdiff_t data)
    {
        return new $parentclassname(rows, cols, ($parentclassname::value_type*)data);
    }

    %pythoncode
    %{
        @classmethod
        def __check_channels_compatibility(cls, array):
            obj = cls()
            n_channel = obj.channels()

            if n_channel == 1:
                if len(array.shape) != 2:
                    raise ValueError("{} expects a 2-dimensional numpy ndarray.".format(cls))
            else:
                if len(array.shape) != 3:
                    raise ValueError("{} expects a 3-dimensional numpy ndarray.".format(cls))
                elif array.shape[2] != n_channel:
                    raise ValueError("{} expects the last ndarray dimension to have a size of {}".format(cls, n_channel))

        @classmethod
        def from_array(cls, array):
            import numpy as np
            array = np.asarray(array)

            if cls()._typestr() != array.__array_interface__['typestr']:
                raise ValueError("{} expects a {} datatype.".format(cls, cls()._typestr()))

            cls.__check_channels_compatibility(array)

            new_mat = cls(_mat__np_array_constructor(),
                          array.shape[0],
                          array.shape[1],
                          array.__array_interface__['data'][0])

            # Holds an internal reference to keep the image buffer alive
            new_mat._array = array

            return new_mat
    %}

    std::string __str__()
    {
        std::ostringstream s;
        s << *$self;
        return s.str();
    }
}

/* %cv_mat__instantiate_defaults
 *
 * Generate a wrapper class to all cv::Mat_<> which has a typedef on OpenCV header file.
 */
%define %cv_mat__instantiate_defaults
    %cv_mat__instantiate(uchar, 1b, u)
    %cv_mat__instantiate(Vec2b, 2b, u)
    %cv_mat__instantiate(Vec3b, 3b, u)
    %cv_mat__instantiate(Vec4b, 4b, u)

    %cv_mat__instantiate(short, 1s, i)
    %cv_mat__instantiate(Vec2s, 2s, i)
    %cv_mat__instantiate(Vec3s, 3s, i)
    %cv_mat__instantiate(Vec4s, 4s, i)

    %cv_mat__instantiate(ushort, 1w, u)
    %cv_mat__instantiate(Vec2w, 2w, u)
    %cv_mat__instantiate(Vec3w, 3w, u)
    %cv_mat__instantiate(Vec4w, 4w, u)

    %cv_mat__instantiate(int, 1i, i)
    %cv_mat__instantiate(Vec2i, 2i, i)
    %cv_mat__instantiate(Vec3i, 3i, i)
    %cv_mat__instantiate(Vec4i, 4i, i)

    %cv_mat__instantiate(float, 1f, f)
    %cv_mat__instantiate(Vec2f, 2f, f)
    %cv_mat__instantiate(Vec3f, 3f, f)
    %cv_mat__instantiate(Vec4f, 4f, f)

    %cv_mat__instantiate(double, 1d, f)
    %cv_mat__instantiate(Vec2d, 2d, f)
    %cv_mat__instantiate(Vec3d, 3d, f)
    %cv_mat__instantiate(Vec4d, 4d, f)
%enddef
