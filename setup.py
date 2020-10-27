from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
from Cython.Build import cythonize
import numpy as np
import sys
from os.path import join, sep
from glob import glob

lib_dirs = ['/usr/local/lib', '/usr/lib/x86_64-linux-gnu']

libs = set([])
for file in glob(join(lib_dirs[0], 'libopencv_*')):
    libs.add(file.split('.')[0])
libs = [lib.split(sep)[-1][3:] for lib in libs]

libs.append('GLEW')
libs.append('pangolin')

setup(
    cmdclass={'build_ext': build_ext},
    ext_modules=cythonize(Extension("orbslam2",
                                    sources=["orbslam2.pyx"],
                                    language="c++",
                                    include_dirs=[np.get_include(),
                                                  '/usr/local/include/opencv2',
                                                  '/usr/local/include/eigen3',
                                                  '/usr/include/GL'
                                                 ],
                                    library_dirs=lib_dirs,
                                    libraries=libs
                                   )
                          )
)
