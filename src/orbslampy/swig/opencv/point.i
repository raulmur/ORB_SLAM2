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

%include <opencv/vec.i>

%include <std_string.i>

%include <opencv/detail/point.i>

%define %cv_point_instantiate(type, type_alias)
    #if !_CV_POINT_##type##_INSTANTIATED_
        %template(_Point__##type) cv::Point_< type >;
        %pythoncode
        %{
            Point2##type_alias = _Point__##type
        %}
        #define _CV_POINT_##type##_INSTANTIATED_
    #endif
%enddef

%header
%{
    #include <opencv2/core/core.hpp>
    #include <sstream>
%}

%extend cv::Point_
{
    %pythoncode
    {
        def __iter__(self):
            return iter((self.x, self.y))
    }

    std::string __str__()
    {
        std::ostringstream s;
        s << *$self;
        return s.str();
    }
}

/* %cv_point_instantiate_defaults
 *
 * Generate a wrapper class to all cv::Point_ which has a typedef on OpenCV header file.
 */
%define %cv_point_instantiate_defaults
    %cv_point_instantiate(int, i)
    %cv_point_instantiate(float, f)
    %cv_point_instantiate(double, d)
    %pythoncode
    {
        Point = Point2i
    }
%enddef
