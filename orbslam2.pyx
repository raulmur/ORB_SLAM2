import numpy as np
cimport numpy as np  # for np.ndarray
from libc.string cimport memcpy
from orbslam2 cimport *

cdef Mat np2Mat3D(np.ndarray ary):
    assert ary.ndim==3 and ary.shape[2]==3, "ASSERT::3channel RGB only!!"
    ary = np.dstack((ary[...,2], ary[...,1], ary[...,0])) #RGB -> BGR

    cdef np.ndarray[np.uint8_t, ndim=3, mode ='c'] np_buff = np.ascontiguousarray(ary, dtype=np.uint8)
    cdef unsigned int* im_buff = <unsigned int*> np_buff.data
    cdef int r = ary.shape[0]
    cdef int c = ary.shape[1]
    cdef Mat m
    m.create(r, c, CV_8UC3)
    memcpy(m.data, im_buff, r*c*3)
    return m

cdef Mat np2Mat(np.ndarray ary):
    cdef Mat out
    out = np2Mat3D(ary)
    return out

cdef object Mat2np(Mat m):
    cdef Py_buffer buf_info
    cdef size_t len = m.rows * m.cols * m.elemSize() # m.elemSize() = m.channels() * sizeof(CV_8UC3)
    PyBuffer_FillInfo(&buf_info, NULL, m.data, len, 1, PyBUF_FULL_RO)
    Pydata  = PyMemoryView_FromBuffer(&buf_info)

    if m.channels() >1 :
        shape_array = (m.rows, m.cols, m.channels())
    else:
        shape_array = (m.rows, m.cols)

    if m.depth() == CV_32F:
        ary = np.ndarray(shape=shape_array, buffer=Pydata, order='c', dtype=np.float32)
    else:
        ary = np.ndarray(shape=shape_array, buffer=Pydata, order='c', dtype=np.uint8)
    
    if m.channels() == 3:
        ary = np.dstack((ary[...,2], ary[...,1], ary[...,0]))

    pyarr = np.asarray(ary)
    return pyarr


cdef class SLAM:
    cdef System *sys

    def __cinit__(self, vocab_file, settings_file):
        self.sys = new System(vocab_file, settings_file, MONOCULAR, True)

    def track_monocular(self, np_arr, timestamp):
        self.sys.TrackMonocular(np2Mat(np_arr), timestamp)

    def shutdown(self):
        self.sys.Shutdown()

    def save_keyframe_trajectory(self, traj_file):
        self.sys.SaveKeyFrameTrajectoryTUM(traj_file)
