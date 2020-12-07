cimport numpy as np
from libcpp cimport bool
from libcpp.vector cimport vector

# For cv::Mat usage
cdef extern from "core/core.hpp":
  cdef int CV_8UC3
  cdef int CV_32F
  cdef int CV_32FC1

cdef extern from "core/core.hpp" namespace "cv":
  cdef cppclass Mat:
    Mat() except +
    void create(int, int, int)
    void* data
    int rows
    int cols
    int channels()
    int depth()
    size_t elemSize()
    bool empty()

# For Buffer usage
cdef extern from "Python.h":
    ctypedef struct PyObject
    object PyMemoryView_FromBuffer(Py_buffer *view)
    int PyBuffer_FillInfo(Py_buffer *view, PyObject *obj, void *buf, Py_ssize_t len, int readonly, int infoflags)
    enum:
        PyBUF_FULL_RO


# For ORB_SLAM2 usage
cpdef enum eSensor:
  MONOCULAR,
  STEREO,
  RGBD

cdef extern from "include/System.h" namespace "ORB_SLAM2":
  cdef cppclass System:
    ctypedef enum eSensor:
      MONOCULAR,
      STEREO,
      RGBD
    System() except +
    System(char*, char*, eSensor, bool, Mat) except +
    Mat TrackStereo(Mat, Mat, double)
    Mat TrackRGBD(Mat, Mat, double)
    Mat TrackMonocular(Mat, double)
    void ActivateLocalizationMode()
    void DeactivateLocalizationMode()
    bool MapChanged()
    void Reset()
    void Shutdown()
    void SaveTrajectoryTUM(char*)
    void SaveKeyFrameTrajectoryTUM(char*)
    void SaveTrajectoryKITTI(char*)
    int GetTrackingState()
    Mat GetWorldPose()
    vector[Mat] PyGetTrackedMapPoints()
    vector[Mat] PyGetAllMapPoints()
    # std::vector<cv::KeyPoint> GetTrackedKeyPointsUn()
