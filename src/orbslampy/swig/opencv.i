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

%include <opencv/mat.i>
%include <opencv/matx.i>
%include <opencv/point.i>
%include <opencv/range.i>
%include <opencv/rect.i>
%include <opencv/scalar.i>
%include <opencv/size.i>
%include <opencv/vec.i>

%define %cv_instantiate_all_defaults
    %cv_mat__instantiate_defaults
    %cv_matx_instantiate_defaults
    %cv_point_instantiate_defaults
    %cv_rect_instantiate_defaults
    %cv_scalar_instantiate_defaults
    %cv_size_instantiate_defaults
    %cv_vec_instantiate_defaults
%enddef
