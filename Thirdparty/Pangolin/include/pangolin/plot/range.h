/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2014 Steven Lovegrove
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

#ifndef PANGOLIN_RANGE_H
#define PANGOLIN_RANGE_H

#include <pangolin/platform.h>

#include <limits>
#include <algorithm>
#include <cmath>

//prevent including Eigen in cuda files
#if defined(HAVE_EIGEN) && !defined(__CUDACC__)
#  define USE_EIGEN
#endif

#ifdef USE_EIGEN
#  include <Eigen/Core>
#  include <Eigen/src/Geometry/AlignedBox.h>
#endif // USE_EIGEN

namespace pangolin
{

template<typename T>
struct Range
{
    Range()
        : min(+std::numeric_limits<T>::max()),
          max(-std::numeric_limits<T>::max())
    {
    }

    Range(T rmin, T rmax)
        : min(rmin), max(rmax)
    {
    }

    Range operator+(T v)
    {
        return Range(min+v, max+v);
    }

    Range operator-(T v)
    {
        return Range(min-v, max-v);
    }

    Range& operator+=(T v)
    {
        min += v;
        max += v;
        return *this;
    }

    Range& operator-=(T v)
    {
        min -= v;
        max -= v;
        return *this;
    }

    Range& operator*=(T v)
    {
        min *= v;
        max *= v;
        return *this;
    }

    Range& operator/=(T v)
    {
        min /= v;
        max /= v;
        return *this;
    }

    Range& operator+=(const Range& o)
    {
        min += o.min;
        max += o.max;
        return *this;
    }

    Range& operator-=(const Range& o)
    {
        min -= o.min;
        max -= o.max;
        return *this;
    }

    Range operator+(const Range& o) const
    {
        return Range(min + o.min, max + o.max);
    }

    Range operator-(const Range& o) const
    {
        return Range(min - o.min, max - o.max);
    }

    Range operator*(float s) const
    {
        return Range(T(s*min), T(s*max));
    }

    T Size() const
    {
        return max - min;
    }

    T AbsSize() const
    {
        return std::abs(Size());
    }

    T Mid() const
    {
        return (min + max) / (T)2.0f;
    }

    void Scale(float s, float center = 0.0f)
    {
        min = T(s*(min-center) + center);
        max = T(s*(max-center) + center);
    }

    void Insert(T v)
    {
        min = std::min(min,v);
        max = std::max(max,v);
    }

    void Clamp(T vmin, T vmax)
    {
        min = std::min(std::max(vmin, min), vmax);
        max = std::min(std::max(vmin, max), vmax);
    }

    void Clamp(const Range& o)
    {
        Clamp(o.min, o.max);
    }

    void Clear()
    {
        min = +std::numeric_limits<T>::max();
        max = -std::numeric_limits<T>::max();
    }

    bool Contains(T v) const
    {
        return min <= v && v <= max;
    }

    bool ContainsWeak(T v) const
    {
        return (min <= v && v <= max)
            || (max <= v && v <= min);
    }

    template<typename To>
    Range<To> Cast() const
    {
        return Range<To>(To(min), To(max));
    }

    T min;
    T max;
};

template<typename T>
struct XYRange
{
    XYRange()
    {
    }

    XYRange(const Range<T>& xrange, const Range<T>& yrange)
        : x(xrange), y(yrange)
    {
    }

    XYRange(T xmin, T xmax, T ymin, T ymax)
        : x(xmin,xmax), y(ymin,ymax)
    {
    }

    XYRange operator-(const XYRange& o) const
    {
        return XYRange(x - o.x, y - o.y);
    }

    XYRange operator*(float s) const
    {
        return XYRange(x*s, y*s);
    }

    XYRange& operator+=(const XYRange& o)
    {
        x += o.x;
        y += o.y;
        return *this;
    }

    void Scale(float sx, float sy, float centerx, float centery)
    {
        x.Scale(sx, centerx);
        y.Scale(sy, centery);
    }

    void Clear()
    {
        x.Clear();
        y.Clear();
    }

    void Clamp(T xmin, T xmax, T ymin, T ymax)
    {
        x.Clamp(xmin,xmax);
        y.Clamp(ymin,ymax);
    }

    void Clamp(const XYRange& o)
    {
        x.Clamp(o.x);
        y.Clamp(o.y);
    }


    float Area() const
    {
        return x.Size() * y.Size();
    }

    bool Contains(float px, float py) const
    {
        return x.Contains(px) && y.Contains(py);
    }

    bool ContainsWeak(float px, float py) const
    {
        return x.ContainsWeak(px) && y.ContainsWeak(py);
    }

    template<typename To>
    XYRange<To> Cast() const
    {
        return XYRange<To>(
            x.template Cast<To>(),
            y.template Cast<To>()
        );
    }

#ifdef USE_EIGEN
    operator Eigen::AlignedBox<T,2>() const {
        return Eigen::AlignedBox<T,2>(
            Eigen::Matrix<T,2,1>(x.min, y.min),
            Eigen::Matrix<T,2,1>(x.max, y.max)
        );
    }
#endif

    Range<T> x;
    Range<T> y;
};

typedef Range<int> Rangei;
typedef Range<float> Rangef;
typedef Range<double> Ranged;

typedef XYRange<int> XYRangei;
typedef XYRange<float> XYRangef;
typedef XYRange<double> XYRanged;

template<typename T> inline
Rangei Round(const Range<T>& r)
{
    return Rangei( int(r.min+0.5), int(r.max+0.5) );
}

template<typename T> inline
XYRangei Round(const XYRange<T>& r)
{
    return XYRangei( Round(r.x), Round(r.y) );
}

}

#endif //PANGOLIN_RANGE_H
