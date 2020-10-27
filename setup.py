from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
from Cython.Build import cythonize
import numpy
import sys
import os
import glob

lib_folder = os.path.join(sys.prefix, 'local', 'lib')
lib_dirs = [lib_folder, '/usr/lib/x86_64-linux-gnu']

libs = []
for file in glob.glob(os.path.join(lib_folder, 'libopencv_*')):
    libs.append(file.split('.')[0])
libs = list(set(libs))
libs = ['opencv_{}'.format(lib.split(os.path.sep)[-1].split('libopencv_')[-1]) for lib in libs]

libs.append('GLEW')
libs.append('pangolin')

setup(
    cmdclass={'build_ext': build_ext},
    ext_modules=cythonize(Extension("orbslam2",
                                    sources=["orbslam2.pyx"],
                                    language="c++",
                                    include_dirs=[numpy.get_include(),
                                                  os.path.join(sys.prefix, 'local', 'include', 'opencv2'),
                                                  os.path.join(sys.prefix, 'local', 'include', 'eigen3'),
                                                  os.path.join(sys.prefix, 'include', 'GL')
                                                 ],
                                    library_dirs=lib_dirs,
                                    libraries=libs,
                                    )
                          )
)
