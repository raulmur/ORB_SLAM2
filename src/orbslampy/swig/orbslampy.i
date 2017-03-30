%module orbslampy


%rename(__len__) std::vector::size();
%rename(append) std::vector::push_back();


%include <std_string.i>
%include <std_vector.i>
%include opencv.i
%cv_instantiate_all_defaults

%{
#include "_orbslampy/pyvector.h"
#include "_orbslampy/orbslammer.h"
%}

%rename(__len__) PyVector::size();
%rename(__getitem__) PyVector::operator[](unsigned int i);

%extend std::vector<cv::Mat>
{
    cv::Mat __getitem__ (unsigned int i)
    {
        return (*($self))[i];
    }
}

// %include std_vector.i
%include "./../include/_orbslampy/pyvector.h"
%include "./../include/_orbslampy/orbslammer.h"

%template(PointCloud) std::vector<cv::Mat>;
//%template(MapPointList) PyVector<PyMapPoint>;
//%template(KeyFrameList) PyVector<cv::KeyFrame>;
