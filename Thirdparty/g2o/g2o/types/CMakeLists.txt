ADD_LIBRARY(types ${G2O_LIB_TYPE}
  types_sba.h
  types_six_dof_expmap.h
  types_sba.cpp
  types_six_dof_expmap.cpp
  types_seven_dof_expmap.cpp
  types_seven_dof_expmap.h
  se3quat.h
  se3_ops.h
  se3_ops.hpp
)

SET_TARGET_PROPERTIES(types PROPERTIES OUTPUT_NAME ${LIB_PREFIX}types)

TARGET_LINK_LIBRARIES(types core)
