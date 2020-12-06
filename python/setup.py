from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
from Cython.Build import cythonize
import numpy as np
from os.path import join, sep
from glob import glob


incl_dirs = ['.', np.get_include(), '/usr/local/include/opencv2', '/usr/local/include/eigen3', '/usr/include/GL']

opencv_lib_dir = '/usr/local/lib'
lib_dirs = ['lib', opencv_lib_dir, '/usr/lib/x86_64-linux-gnu']

libs = set([])
for file in glob(join(opencv_lib_dir, 'libopencv_*')):
    libs.add(file.split('.')[0])
libs = [lib.split(sep)[-1][3:] for lib in libs]

libs.append('GLEW')
libs.append('pangolin')
libs.append('ORB_SLAM2')

setup(
    cmdclass={'build_ext': build_ext},
    build_dir="lib",
    ext_modules=cythonize(Extension(
        "orbslam2",
        sources=["python/orbslam2.pyx"],
        language="c++",
        include_dirs=incl_dirs,
        library_dirs=lib_dirs,
        libraries=libs
    ))
)
