/*
This file contains the necessary serialize functions for complex datatypes
used by ORBSLAM2 and which are not available in boost serialization library
*/

#ifndef SERIALIZE_H
#define SERIALIZE_H
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>
// KeyFrame::mConnectedKeyFrameWeights
#include <boost/serialization/map.hpp>
// #include <boost/serialization/split_free.hpp>
// DBoW2::BowVector and DBoW2::FeatureVector
#include <boost/serialization/base_object.hpp>
// KeyFrame::mspChildrens
#include <boost/serialization/set.hpp>
#include <opencv2/core/core.hpp>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive &ar, DBoW2::BowVector &BowVec,
               const unsigned int file_version) {
  // Derived classes should include serializations of their base classes.
  ar &boost::serialization::base_object<DBoW2::BowVector::super>(BowVec);
}

template <class Archive>
void serialize(Archive &ar, DBoW2::FeatureVector &FeatVec,
               const unsigned int file_version) {
  ar &boost::serialization::base_object<DBoW2::FeatureVector::super>(FeatVec);
}

template <class Archive>
void save(Archive &ar, const ::cv::Mat &m, const unsigned int file_version) {
  cv::Mat m_ = m;
  if (!m.isContinuous()) m_ = m.clone();
  size_t elem_size = m_.elemSize();
  size_t elem_type = m_.type();
  ar &m_.cols;
  ar &m_.rows;
  ar &elem_size;
  ar &elem_type;

  const size_t data_size = m_.cols * m_.rows * elem_size;

  ar &boost::serialization::make_array(m_.ptr(), data_size);
}

template <class Archive>
void load(Archive &ar, ::cv::Mat &m, const unsigned int version) {
  int cols, rows;
  size_t elem_size, elem_type;

  ar &cols;
  ar &rows;
  ar &elem_size;
  ar &elem_type;

  m.create(rows, cols, elem_type);
  size_t data_size = m.cols * m.rows * elem_size;

  ar &boost::serialization::make_array(m.ptr(), data_size);
}

template <class Archive>
void serialize(Archive &ar, ::cv::KeyPoint &kf,
               const unsigned int file_version) {
  ar &kf.angle;
  ar &kf.class_id;
  ar &kf.octave;
  ar &kf.response;
  ar &kf.pt.x;
  ar &kf.pt.y;
}
}
}
#endif  // SERIALIZE_H