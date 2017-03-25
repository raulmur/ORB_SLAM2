%module orbslampy


%include opencv.i
%cv_instantiate_all_defaults

%{
#include "_orbslampy/orbslammer.h"
%}

// %include std_vector.i
%include "./../include/_orbslampy/orbslammer.h"
