import numpy as np
cimport numpy as np  # for np.ndarray
from libc.string cimport memcpy
from orbslam2 cimport *


cdef Mat pose2Mat(np.ndarray pose):
    assert pose.dtype == np.float32 and pose.ndim == 2, "ASSERT::pose must be a 2D np.float32 array"

    cdef np.ndarray[np.float32_t, ndim=2, mode ='c'] np_buff = np.ascontiguousarray(pose, dtype=np.float32)
    cdef float* im_buff = <float*> np_buff.data

    cdef Mat m
    m.create(4, 4, CV_32F)
    memcpy(m.data, im_buff, 4*4*sizeof(float))
    return m

cdef Mat frame2Mat(np.ndarray frame):
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

cdef Mat depth2Mat(np.ndarray depth_frame):
    assert depth_frame.dtype == np.float32 and depth_frame.ndim == 2, "ASSERT::depth_frame must be a 2D np.float32 array"

    cdef np.ndarray[np.float32_t, ndim=2, mode ='c'] np_buff = np.ascontiguousarray(depth_frame, dtype=np.float32)
    cdef float* im_buff = <float*> np_buff.data
    cdef int r = depth_frame.shape[0]
    cdef int c = depth_frame.shape[1]

    cdef Mat m
    m.create(r, c, CV_32FC1)
    memcpy(m.data, im_buff, r*c*sizeof(float))
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
        self.sys = new System(vocab_file, settings_file, self.sensor, use_viewer, pose2Mat(init_pose))

    def track(self, *inputs, timestamp=0):
        if self.sensor == MONOCULAR:
            frame = inputs[0]
            self.sys.TrackMonocular(frame2Mat(frame), timestamp)
        elif self.sensor == STEREO:
            frame_l, frame_r = inputs
            self.sys.TrackStereo(frame2Mat(frame_l), frame2Mat(frame_r), timestamp)
        elif self.sensor == RGBD:
            frame, depth_frame = inputs
            self.sys.TrackRGBD(frame2Mat(frame), depth2Mat(depth_frame), timestamp)
        return self.get_tracked_pose()

    def activateLocalizationMode(self):
        self.sys.ActivateLocalizationMode()

    def deactivateLocalizationMode(self):
        self.sys.DeactivateLocalizationMode()

    def mapChanged(self):
        self.sys.MapChanged()
    
    def reset(self):
        self.sys.Reset()

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

    def get_tracked_pose(self):
        pose = self.sys.GetTrackedPose()
        return Mat2np(pose)
