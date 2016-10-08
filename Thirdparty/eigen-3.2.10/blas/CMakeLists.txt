
project(EigenBlas CXX)

include("../cmake/language_support.cmake")

workaround_9220(Fortran EIGEN_Fortran_COMPILER_WORKS)

if(EIGEN_Fortran_COMPILER_WORKS)
  enable_language(Fortran OPTIONAL)
  if(NOT CMAKE_Fortran_COMPILER)
    set(EIGEN_Fortran_COMPILER_WORKS OFF)
  endif()
endif()

add_custom_target(blas)

set(EigenBlas_SRCS single.cpp double.cpp complex_single.cpp complex_double.cpp xerbla.cpp)

if(EIGEN_Fortran_COMPILER_WORKS)

set(EigenBlas_SRCS ${EigenBlas_SRCS}
    complexdots.f
    srotm.f srotmg.f drotm.f drotmg.f
    lsame.f  dspmv.f ssbmv.f
    chbmv.f  sspmv.f
    zhbmv.f  chpmv.f dsbmv.f
    zhpmv.f
    dtbmv.f stbmv.f ctbmv.f ztbmv.f
)
else()

message(WARNING " No fortran compiler has been detected, the blas build will be incomplete.")

endif()

add_library(eigen_blas_static ${EigenBlas_SRCS})
add_library(eigen_blas SHARED ${EigenBlas_SRCS})

if(EIGEN_STANDARD_LIBRARIES_TO_LINK_TO)
  target_link_libraries(eigen_blas_static ${EIGEN_STANDARD_LIBRARIES_TO_LINK_TO})
  target_link_libraries(eigen_blas        ${EIGEN_STANDARD_LIBRARIES_TO_LINK_TO})
endif()

add_dependencies(blas eigen_blas eigen_blas_static)

install(TARGETS eigen_blas eigen_blas_static
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

if(EIGEN_Fortran_COMPILER_WORKS)

if(EIGEN_LEAVE_TEST_IN_ALL_TARGET)
  add_subdirectory(testing) # can't do EXCLUDE_FROM_ALL here, breaks CTest
else()
  add_subdirectory(testing EXCLUDE_FROM_ALL)
endif()

endif()

