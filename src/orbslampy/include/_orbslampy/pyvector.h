#pragma once
#ifndef ORBSLAMPY_PYVECTOR_H
#define ORBSLAMPY_PYVECTOR_H

// SYSTEM INCLUDES
#include <string>
#include <vector>

// C++ PROJECT INCLUDES


template<typename TYPE>
class PyVector
{
public:

    PyVector() : _internal_vec() {}
    PyVector(std::vector<TYPE>& vec) : _internal_vec(vec) {}

    virtual ~PyVector() {}

    unsigned int size() { return this->_internal_vec.size(); }
    TYPE operator[](unsigned int i) { return this->_internal_vec[i]; }


protected:

private:
    std::vector<TYPE>   _internal_vec;
};


#endif // end of ORBSLAMPY_PYVECTOR_H
