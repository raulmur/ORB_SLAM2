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

%include <opencv/point.i>
%include <opencv/size.i>

%define %cv_rect_instantiate(type, type_alias)
    #if !_CV_RECT_##type##_INSTANTIATED_
        %template(_Rect__##type) cv::Rect_< type >;
        %pythoncode
        %{
            Rect2##type_alias = _Rect__##type
        %}
        #define _CV_RECT_##type##_INSTANTIATED_
    #endif
%enddef

%header
%{
    #include <opencv2/core/core.hpp>
    #include <sstream>
%}

%include <opencv/detail/rect.i>

%extend cv::Rect_
{
    %pythoncode
    {
        def __iter__(self):
            return iter((self.x, self.y, self.width, self.height))
    }

    std::string __str__()
    {
        std::ostringstream s;
        s << *$self;
        return s.str();
    }
}

/* %cv_rect_instantiate_defaults
 *
 * Generate a wrapper class to all cv::Rect_ which has a typedef on OpenCV header file.
 */
%define %cv_rect_instantiate_defaults
    %cv_rect_instantiate(int, i)
    %cv_rect_instantiate(float, f)
    %cv_rect_instantiate(double, d)
    %pythoncode
    {
        Rect = Rect2i
    }
%enddef
