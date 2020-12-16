import numpy as np
cimport numpy as np  # for np.ndarray
from libc.string cimport memcpy
from orbslam2 cimport *


cdef Mat np2Mat(np.ndarray np_array):
    assert np_array.dtype == np.float32 and np_array.ndim == 2, "ASSERT::input must be a 2D np.float32 array"

    cdef np.ndarray[np.float32_t, ndim=2, mode ='c'] np_buff = np.ascontiguousarray(np_array, dtype=np.float32)
    cdef float* im_buff = <float*> np_buff.data
    cdef int r = np_array.shape[0]
    cdef int c = np_array.shape[1]

    cdef Mat m
    m.create(r, c, CV_32FC1)
    memcpy(m.data, im_buff, r*c*sizeof(float))
    return m

cdef Mat rgb2Mat(np.ndarray frame):
    assert frame.dtype == np.uint8 and frame.ndim == 3 and frame.shape[2] == 3, "ASSERT::frame must be an RGB image (3-channel np.uint8 array)"

    frame = np.flip(frame, axis=2) # RGB to BGR
    cdef np.ndarray[np.uint8_t, ndim=3, mode ='c'] np_buff = np.ascontiguousarray(frame, dtype=np.uint8)
    cdef unsigned int* im_buff = <unsigned int*> np_buff.data
    cdef int r = frame.shape[0]
    cdef int c = frame.shape[1]

    cdef Mat m
    m.create(r, c, CV_8UC3)
    memcpy(m.data, im_buff, r*c*3)
    return m

cdef object Mat2np(Mat m):
    if m.empty(): return None

    cdef Py_buffer buf_info
    cdef size_t len = m.rows * m.cols * m.elemSize()
    PyBuffer_FillInfo(&buf_info, NULL, m.data, len, 1, PyBUF_FULL_RO)
    Pydata  = PyMemoryView_FromBuffer(&buf_info)

    if m.channels() > 1:
        shape_array = (m.rows, m.cols, m.channels())
    else:
        shape_array = (m.rows, m.cols)

    arr = np.ndarray(shape=shape_array,
                     buffer=Pydata,
                     order='c',
                     dtype=(np.float32 if m.depth() == CV_32F else np.uint8))

    return np.asarray(arr)

cdef class SLAM:
    cdef System *sys
    cdef eSensor sensor

    def __cinit__(self, vocab_file, settings_file, sensor, init_pose, use_viewer):
        vocab_file = vocab_file.encode('utf-8')
        settings_file = settings_file.encode('utf-8')
        self.sensor = {'monocular' : MONOCULAR, 'stereo' : STEREO, 'rgbd' : RGBD}[sensor]
        self.sys = new System(vocab_file, settings_file, self.sensor, use_viewer, np2Mat(init_pose))

    def track(self, *inputs, timestamp=0.0):
        if self.sensor == MONOCULAR:
            frame = inputs[0]
            self.sys.TrackMonocular(rgb2Mat(frame), timestamp)
        elif self.sensor == STEREO:
            frame_l, frame_r = inputs
            self.sys.TrackStereo(rgb2Mat(frame_l), rgb2Mat(frame_r), timestamp)
        elif self.sensor == RGBD:
            frame, depth_frame = inputs
            self.sys.TrackRGBD(rgb2Mat(frame), np2Mat(depth_frame), timestamp)

    def activateLocalizationMode(self):
        self.sys.ActivateLocalizationMode()

    def deactivateLocalizationMode(self):
        self.sys.DeactivateLocalizationMode()

    def mapChanged(self):
        self.sys.MapChanged()
    
    def reset(self, new_pose=None):
        if new_pose is None:
            self.sys.Reset()
        else:
            self.sys.Reset(np2Mat(new_pose))

    def shutdown(self):
        self.sys.Shutdown()

    def save_trajectory_tum(self, out_file):
        out_file = out_file.encode('utf-8')
        self.sys.SaveTrajectoryTUM(out_file)

    def save_trajectory_kitti(self, out_file):
        out_file = out_file.encode('utf-8')
        self.sys.SaveTrajectoryKITTI(out_file)

    def save_keyframe_trajectory(self, out_file):
        out_file = out_file.encode('utf-8')
        self.sys.SaveKeyFrameTrajectoryTUM(out_file)

    def get_tracking_state(self):
        return self.sys.GetTrackingState()

    def get_world_pose(self):
        pose = self.sys.GetWorldPose()
        return Mat2np(pose)

    def get_tracked_map_points(self):
        map_points = self.sys.PyGetTrackedMapPoints()
        return [Mat2np(mp) for mp in map_points]

    def get_all_map_points(self):
        map_points = self.sys.PyGetAllMapPoints()
        return [Mat2np(mp) for mp in map_points]
